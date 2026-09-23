"""Bounded single-joint calibration state machine; independent of ROS and CAN."""
from collections import deque
from dataclasses import asdict, dataclass
import math
import statistics


ORDER = (6, 5, 4, 2, 3, 1, 0)


@dataclass(frozen=True)
class SeekLimits:
    stall_torque: float = 0.5
    kp: float = 3.0
    kd: float = 1.0
    max_position_error: float = 0.20
    effort_abort: float = 1.0
    speed: float = 0.025
    max_speed: float = 0.15
    hard_max_speed: float = 0.30
    overspeed_time: float = 0.10
    max_travel: float = 3.5
    timeout: float = 180.0
    stall_time: float = 1.0
    stall_span: float = 0.004
    min_travel: float = 0.03
    repeat_tolerance: float = 0.015
    target_tolerance: float = 0.01
    return_min_fraction: float = 0.5
    feedback_timeout: float = 0.12

    def validate(self, motor_torque):
        for value in vars(self).values():
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError('All motion settings must be finite positive numbers')
        if not self.stall_torque < self.effort_abort <= motor_torque * 0.25:
            raise ValueError('Require stall_torque < effort_abort <=25% of the motor range')
        if self.kp > 70 or self.kd > 5 or self.max_position_error > 0.25:
            raise ValueError('Require kp<=70, kd<=5, max_position_error<=0.25 rad')
        if self.kp * self.max_position_error <= self.stall_torque:
            raise ValueError('PD position-error budget cannot reach the stall detection threshold')
        # Bound commanded spring/feed-forward effort. Velocity-dependent braking
        # is not a hardware torque cap; measured-effort faults remain independent.
        envelope = self.kp * self.max_position_error + min(self.kd * self.speed, 0.2)
        if envelope >= self.effort_abort:
            raise ValueError('PD command effort estimate must be below effort_abort')
        if not self.speed < self.max_speed <= 1.5 or self.speed > 1.0:
            raise ValueError('Require seek speed <=1.0 rad/s and speed < max_speed <=1.5')
        if not self.max_speed < self.hard_max_speed <= 2.0 or not 0 < self.overspeed_time <= 0.1:
            raise ValueError('Require soft < hard speed <=2.0 rad/s and overspeed_time <=0.1 s')
        if not 0.5 <= self.stall_time <= 3 or self.feedback_timeout > 0.2:
            raise ValueError('Invalid stall time or feedback timeout')
        if not self.stall_span < self.min_travel <= 0.1:
            raise ValueError('Require stall_span < min_travel <=0.1 rad')
        if self.repeat_tolerance >= self.min_travel or self.max_travel > 2 * math.pi:
            raise ValueError('Invalid repeat tolerance or travel budget')
        if self.target_tolerance > 0.02:
            raise ValueError('Require target_tolerance <=0.02 rad')
        if not 0.1 <= self.return_min_fraction <= 1.0:
            raise ValueError('Require return_min_fraction between 0.1 and 1.0')


@dataclass(frozen=True)
class FrictionModel:
    coulomb_pos: float = 0.0
    coulomb_neg: float = 0.0
    viscous: float = 0.0
    gain: float = 0.0

    def validate(self):
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or
               not math.isfinite(v) or v < 0 for v in vars(self).values()):
            raise ValueError('Friction coefficients/gain must be finite nonnegative values')

    def torque(self, raw_speed):
        if raw_speed == 0:
            return 0.0
        coulomb = self.coulomb_pos if raw_speed > 0 else -self.coulomb_neg
        return self.gain * (coulomb + self.viscous * raw_speed)

    def maximum(self, speed):
        return max(abs(self.torque(speed)), abs(self.torque(-speed)))


@dataclass(frozen=True)
class PDCommand:
    position: float = 0.0
    velocity: float = 0.0
    kp: float = 0.0
    kd: float = 0.0
    torque: float = 0.0

    @property
    def active(self):
        return self.kp > 0 or self.kd > 0


class SeekMachine:
    """Advance a bounded position target; the motor performs the PD tracking."""
    MOVING = ('seek', 'return_probe', 'reseek', 'return')

    def __init__(self, sign, reference, side, limits, manual=False, return_duration=None, friction=None):
        if sign not in (-1, 1) or side not in ('positive', 'negative') or not math.isfinite(reference):
            raise ValueError('Invalid verified direction/reference')
        if not manual and (reference == 0 or (reference > 0) != (side == 'positive')):
            raise ValueError('Reference angle conflicts with seek side from near-zero start')
        self.sign, self.reference, self.limits = sign, reference, limits
        self.friction = friction if friction is not None else FrictionModel()
        self.friction.validate()
        budget = (limits.kp * limits.max_position_error + min(limits.kd * limits.speed, 0.2)
                  + self.friction.maximum(limits.speed))
        if self.friction.gain and budget >= limits.effort_abort:
            raise ValueError('PD plus friction command budget exceeds effort_abort; review settings')
        self.residual_effort = 0.0
        if return_duration is not None and (not math.isfinite(return_duration) or return_duration <= 0):
            raise ValueError('Return duration must be finite and positive')
        self.return_duration = return_duration
        self.manual = manual
        self.phase_speed = limits.speed
        self.direction = sign * (1 if side == 'positive' else -1)
        self.state = 'manual' if manual else 'ready'
        self.reason = ''
        self.history = deque()
        self.last_q = self.last_t = None
        self.first_stop = self.stop_raw = self.offset = None
        self.verify_origin = None
        self.verify_motion = 0.0
        self.phase_q = self.phase_t = 0.0
        self.phase_budget = limits.max_travel
        self.phase_timeout = limits.timeout
        self.goal = None
        self.command = PDCommand()
        self.target = None
        self.filtered_velocity = 0.0
        self.fault_snapshot = None
        self.last_sample = None
        self.position_velocity = self.sample_dt = self.position_delta = 0.0
        self.overspeed_since = None
        self.retreats = []

    def fault(self, reason):
        if self.state != 'fault':
            self.fault_snapshot = dict(phase=self.state, sample=self.last_sample,
                                       filtered_velocity=self.filtered_velocity,
                                       previous_command=asdict(self.command),
                                       residual_effort=self.residual_effort,
                                       position_delta=self.position_delta,
                                       sample_dt=self.sample_dt,
                                       position_velocity=self.position_velocity,
                                       phase_speed=self.phase_speed,
                                       phase_start_position=self.phase_q,
                                       seek_direction=self.direction,
                                       first_stop=self.first_stop,
                                       stop_raw=self.stop_raw,
                                       zero_offset=self.offset,
                                       min_travel=self.limits.min_travel,
                                       goal=self.goal,
                                       target_tolerance=self.limits.target_tolerance,
                                       retreat_progress=(self.retreat_progress(self.last_sample[0])
                                                         if self.goal is not None and self.last_sample else None),
                                       max_speed=self.limits.max_speed)
            self.fault_snapshot.update(hard_max_speed=self.limits.hard_max_speed,
                                       overspeed_since=self.overspeed_since)
        self.state, self.reason = 'fault', reason
        return self.passive()

    def passive(self):
        self.command = PDCommand()
        return self.command

    def start(self, state, now, q, goal=None):
        self.state, self.phase_t, self.phase_q, self.goal = state, now, q, goal
        self.history.clear()
        self.target = q
        self.passive()
        self.phase_speed = self.limits.speed
        if state in ('return_probe', 'return') and self.return_duration is not None:
            self.phase_speed = min(self.limits.speed, max(0.001, abs(goal - q) / self.return_duration))
        if goal is None:
            self.phase_budget = self.limits.max_travel
        else:
            self.phase_budget = abs(goal - q) + 0.1
        self.phase_timeout = min(self.limits.timeout,
                                10 + 3 * self.phase_budget / self.phase_speed)

    def _reference(self, raw):
        self.stop_raw = raw
        self.offset = raw - self.sign * self.reference

    def retreat_progress(self, q):
        distance = abs(self.goal - self.phase_q)
        direction = math.copysign(1.0, self.goal - self.phase_q)
        required = min(distance, max(self.limits.min_travel,
                                     distance * self.limits.return_min_fraction))
        return direction * (q - self.phase_q), required

    def update(self, now, sample, key=None):
        """sample=(raw position, raw velocity, measured torque); None means stale/offline."""
        if self.state in ('fault', 'done'):
            return self.passive()
        self.last_sample = sample
        if key == 'escape':
            return self.fault('operator_abort')
        if sample is None or not all(math.isfinite(x) for x in sample):
            return self.fault('feedback_lost_or_invalid')
        q, velocity, effort = sample
        # Feedback corresponds to the previously sent command, not this tick's
        # new direction. Use that exact feed-forward for contact detection.
        self.residual_effort = effort - self.command.torque
        if self.last_t is not None and (now <= self.last_t or now - self.last_t > self.limits.feedback_timeout):
            return self.fault('control_loop_delay')
        if self.last_q is not None and abs(q - self.last_q) > 0.08:
            return self.fault('encoder_jump_or_wrap')
        dt = now - self.last_t if self.last_t is not None else 0.01
        self.sample_dt = dt
        if self.last_q is not None:
            self.position_delta = q - self.last_q
            self.position_velocity = self.position_delta / dt
            alpha = dt / (0.05 + dt)
            self.filtered_velocity += alpha * (self.position_velocity - self.filtered_velocity)
        self.last_t, self.last_q = now, q
        # Passive manual verification is explicitly hand-driven. Apply motion
        # speed guards only while driving or accepting a request to start driving.
        motion_guard = (self.state in self.MOVING or
                        (key == 'enter' and self.state in ('ready', 'contact', 'initial_stall', 'return_confirm')))
        if motion_guard and abs(velocity) >= self.limits.hard_max_speed:
            return self.fault('overspeed_hard')
        speeding = motion_guard and abs(velocity) > self.limits.max_speed
        if speeding:
            if self.overspeed_since is None:
                self.overspeed_since = now
            if now - self.overspeed_since >= self.limits.overspeed_time:
                return self.fault('overspeed_sustained')
        else:
            self.overspeed_since = None
        if self.state in self.MOVING and abs(effort) > self.limits.effort_abort:
            return self.fault('measured_torque_limit')
        self.history.append((now, q, velocity, self.residual_effort))
        while self.history and now - self.history[0][0] > self.limits.stall_time + 0.05:
            self.history.popleft()

        if self.state == 'ready' and key == 'enter':
            self.start('seek', now, q)
        elif self.state == 'initial_stall' and key == 'r':
            self.state, self.first_stop, self.reason = 'ready', None, ''
            self.history.clear()
            return self.passive()
        elif self.state in ('contact', 'initial_stall') and key == 'enter':
            # Latch the boundary under load, as in gripper calibration. Passive
            # spring-back during confirmation must not overwrite/revalidate it.
            # This is only a provisional zero; do not commit an offset until
            # the second full approach confirms the first stop.
            candidate_zero = self.first_stop - self.sign * self.reference
            self.reason = ''
            self.start('return_probe', now, q, candidate_zero)
        elif self.state == 'return_confirm' and key == 'enter':
            self.reason = ''
            self.start('return', now, q, self.offset)
        elif self.state == 'manual' and key == 'enter':
            if not self._stationary():
                self.reason = 'Hold still for the sampling window, then confirm again'
            else:
                self._reference(statistics.median(h[1] for h in self.history))
                self.state = 'manual_preview'
                self.reason = ''
        elif self.state == 'manual_preview':
            if key == 'n':
                self.state = 'done'
            return self.passive()
        elif self.state == 'verify':
            self.verify_motion = max(self.verify_motion, abs(q - self.verify_origin))
            if key == 'enter' and self.verify_motion >= 0.08:
                self.state = 'done'
            elif key == 'enter':
                self.reason = 'Move the real joint at least 5 deg before accepting'
            return self.passive()

        if self.state not in self.MOVING:
            return self.passive()
        if now - self.phase_t > self.phase_timeout:
            return self.fault('motion_timeout')
        if abs(q - self.phase_q) > self.phase_budget:
            return self.fault('travel_budget_exceeded')

        if speeding:
            # Do not keep winding up the position ramp during a velocity excursion.
            self.target = q
            self.command = PDCommand(position=q, kp=self.limits.kp, kd=self.limits.kd)
            return self.command

        if self.state in ('seek', 'reseek'):
            travel = self.direction * (q - self.phase_q)
            if travel < -0.02:
                return self.fault('motion_opposite_to_seek_direction')
            if self._stationary() and all(self.direction * h[3] >= self.limits.stall_torque
                                           for h in self.history):
                minimum = self.limits.min_travel
                if travel < minimum:
                    if self.state == 'seek':
                        self.first_stop = statistics.median(h[1] for h in self.history)
                        self.state = 'initial_stall'
                        self.reason = (f'Early stall after {travel:.4f} rad, not a verified stop. '
                                       'Reposition then R to retry; ENTER only if you confirm the actual hard stop.')
                        return self.passive()
                    return self.fault('reseek_insufficient_motion')
                raw = statistics.median(h[1] for h in self.history)
                if self.state == 'seek':
                    self.first_stop, self.state = raw, 'contact'
                elif abs(raw - self.first_stop) > self.limits.repeat_tolerance:
                    return self.fault('stop_not_repeatable')
                else:
                    self._reference((raw + self.first_stop) / 2)
                    self.state = 'return_confirm'
                return self.passive()
            next_target = self.target + self.direction * self.phase_speed * dt
            friction_speed = self.direction * self.phase_speed
        else:
            error = self.goal - q
            clearance, required = self.retreat_progress(q)
            ramp_duration = abs(self.goal - self.phase_q) / self.phase_speed
            near_zero = abs(error) <= self.limits.target_tolerance
            withdrawn = clearance >= required and now - self.phase_t >= ramp_duration
            # A repeatable stop defines the offset. Retreat only needs clearance;
            # static tracking error must not redefine zero or block re-probing.
            if (near_zero or withdrawn) and self._stationary():
                self.retreats.append(dict(phase=self.state, actual=q, goal=self.goal,
                                         zero_error=error, clearance=clearance, required=required,
                                         completion='near_zero' if near_zero else 'clearance'))
                self.reason = (f'Retreat complete: clearance={clearance:.4f}/{required:.4f} rad; '
                               f'zero error={error:+.4f} rad (offset unchanged).')
                if self.state == 'return_probe':
                    self.start('reseek', now, q)
                else:
                    self.state, self.verify_origin = 'verify', q
                return self.passive()
            if (now - self.phase_t > 2 * self.limits.stall_time and self._stationary()
                    and abs(error) > self.limits.target_tolerance and all(abs(h[3]) >= self.limits.stall_torque
                                                for h in self.history)):
                return self.fault('return_blocked')
            step = self.phase_speed * dt
            next_target = self.target + max(-step, min(step, self.goal - self.target))
            friction_speed = (math.copysign(self.phase_speed, error)
                              if abs(error) > self.limits.target_tolerance else 0.0)
        # Stop accumulating position error at a stop; all motion phases share this bound.
        bounded_target = max(q - self.limits.max_position_error,
                             min(q + self.limits.max_position_error, next_target))
        ff_limit = min(self.phase_speed, 0.2 / self.limits.kd)
        # At the lead bound the target follows encoder quantization, not the
        # intended trajectory. Do not turn that jitter into velocity feed-forward.
        vel_ff = (0.0 if bounded_target != next_target else
                  max(-ff_limit, min(ff_limit, (next_target - self.target) / dt)))
        self.target = bounded_target
        self.command = PDCommand(position=self.target, velocity=vel_ff,
                                 kp=self.limits.kp, kd=self.limits.kd,
                                 torque=self.friction.torque(friction_speed))
        return self.command

    def _stationary(self):
        return (len(self.history) >= 10
                and self.history[-1][0] - self.history[0][0] >= self.limits.stall_time
                and max(h[1] for h in self.history) - min(h[1] for h in self.history) <= self.limits.stall_span
                and all(abs(h[2]) < 0.04 for h in self.history))
