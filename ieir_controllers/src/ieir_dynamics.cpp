#include "ieir_controllers/ieir_dynamics.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/spatial/explog.hpp>
#include <stdexcept>

namespace ieir_dynamics
{

bool IeirDynamics::initFromURDF(const std::string& urdf_string)
{
  if (urdf_string.empty()) {
    return false;
  }

  try {
    // Build model from URDF XML string
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    
    // Create data structure
    data_ = std::make_unique<pinocchio::Data>(model_);
    
    // Set gravity vector (standard: [0, 0, -9.81] in world frame)
    model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.81));
    
    // Initialize state vectors
    q_current_ = pinocchio::neutral(model_);
    v_current_ = Eigen::VectorXd::Zero(model_.nv);
    
    initialized_ = true;
    return true;
    
  } catch (const std::exception& e) {
    initialized_ = false;
    return false;
  }
}

void IeirDynamics::updateState(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (q.size() != model_.nq || v.size() != model_.nv) {
    throw std::runtime_error("State vector size mismatch");
  }
  
  q_current_ = q;
  v_current_ = v;
}

Eigen::VectorXd IeirDynamics::computeGravity()
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  // Compute generalized gravity g(q)
  pinocchio::computeGeneralizedGravity(model_, *data_, q_current_);
  
  return data_->g;
}

Eigen::VectorXd IeirDynamics::computeImpedance(
  const Eigen::VectorXd& q_desired,
  const Eigen::VectorXd& v_desired,
  const Eigen::VectorXd& stiffness,
  const Eigen::VectorXd& damping)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (q_desired.size() != model_.nq || v_desired.size() != model_.nv ||
      stiffness.size() != model_.nv || damping.size() != model_.nv) {
    throw std::runtime_error("Input vector size mismatch");
  }
  
  // Compute impedance torque: tau = Kp*(qd-q) + Kd*(vd-v)
  Eigen::VectorXd tau_impedance = Eigen::VectorXd::Zero(model_.nv);
  
  // Position error (for revolute joints, this is simple subtraction)
  Eigen::VectorXd q_error = q_desired - q_current_;
  
  // Velocity error
  Eigen::VectorXd v_error = v_desired - v_current_;
  
  // Compute torque with diagonal gains
  tau_impedance = stiffness.cwiseProduct(q_error) + damping.cwiseProduct(v_error);
  
  return tau_impedance;
}

std::string IeirDynamics::getJointName(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return "";
  }
  return model_.names[joint_id];
}

int IeirDynamics::getJointIdxQ(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return -1;
  }
  return model_.joints[joint_id].idx_q();
}

int IeirDynamics::getJointIdxV(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return -1;
  }
  return model_.joints[joint_id].idx_v();
}

double IeirDynamics::getEffortLimit(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return 0.0;
  }
  
  int idx_v = model_.joints[joint_id].idx_v();
  if (idx_v >= 0 && idx_v < model_.effortLimit.size()) {
    return model_.effortLimit[idx_v];
  }
  
  return 0.0;
}

bool IeirDynamics::isActuated(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return false;
  }
  return model_.joints[joint_id].nv() > 0;
}

Eigen::VectorXd IeirDynamics::getNeutralConfiguration() const
{
  if (!initialized_) {
    return Eigen::VectorXd();
  }
  return pinocchio::neutral(model_);
}

int IeirDynamics::getFrameId(const std::string& frame_name) const
{
  if (!initialized_) {
    return -1;
  }
  
  if (!model_.existFrame(frame_name)) {
    return -1;
  }
  
  return static_cast<int>(model_.getFrameId(frame_name));
}

pinocchio::SE3 IeirDynamics::computeFK(int frame_id)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (frame_id < 0 || frame_id >= static_cast<int>(model_.nframes)) {
    throw std::runtime_error("Invalid frame_id for FK");
  }
  
  // Compute forward kinematics
  pinocchio::forwardKinematics(model_, *data_, q_current_);
  pinocchio::updateFramePlacement(model_, *data_, static_cast<pinocchio::FrameIndex>(frame_id));
  
  return data_->oMf[frame_id];
}

Eigen::MatrixXd IeirDynamics::computeFrameJacobian(int frame_id)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (frame_id < 0 || frame_id >= static_cast<int>(model_.nframes)) {
    throw std::runtime_error("Invalid frame_id for Jacobian");
  }
  
  // Compute joint Jacobian in LOCAL_WORLD_ALIGNED frame
  Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, model_.nv);
  pinocchio::computeFrameJacobian(
    model_, *data_, q_current_,
    static_cast<pinocchio::FrameIndex>(frame_id),
    pinocchio::LOCAL_WORLD_ALIGNED, J);
  
  return J;
}

bool IeirDynamics::solveIK(
  int frame_id,
  const pinocchio::SE3& target_pose,
  const Eigen::VectorXd& q_init,
  Eigen::VectorXd& q_result,
  int max_iter,
  double eps,
  double damping,
  double dt)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (frame_id < 0 || frame_id >= static_cast<int>(model_.nframes)) {
    throw std::runtime_error("Invalid frame_id for IK");
  }
  
  if (q_init.size() != model_.nq) {
    throw std::runtime_error("q_init size mismatch");
  }
  
  q_result = q_init;
  pinocchio::FrameIndex fid = static_cast<pinocchio::FrameIndex>(frame_id);
  
  Eigen::MatrixXd J(6, model_.nv);
  
  for (int iter = 0; iter < max_iter; ++iter) {
    // Compute FK at current q
    pinocchio::forwardKinematics(model_, *data_, q_result);
    pinocchio::updateFramePlacement(model_, *data_, fid);
    
    // Compute pose error in body frame: current^{-1} * target
    pinocchio::SE3 iMd = data_->oMf[fid].actInv(target_pose);
    Eigen::Matrix<double, 6, 1> err = pinocchio::log6(iMd).toVector();
    
    if (err.norm() < eps) {
      return true;
    }
    
    // Compute Jacobian at current q (LOCAL frame to match log6 body-frame error)
    J.setZero();
    pinocchio::computeFrameJacobian(
      model_, *data_, q_result, fid,
      pinocchio::LOCAL, J);
    
    // Damped pseudoinverse: J_pinv = J^T * (J * J^T + lambda^2 * I)^{-1}
    Eigen::MatrixXd JJt = J * J.transpose();
    JJt.diagonal().array() += damping * damping;
    Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(Eigen::MatrixXd::Identity(6, 6));
    
    Eigen::VectorXd dq = J_pinv * err;
    
    // Update q with step size
    q_result = pinocchio::integrate(model_, q_result, dt * dq);
    
    // Clamp to joint limits
    for (int i = 0; i < model_.nq; ++i) {
      if (model_.lowerPositionLimit[i] < model_.upperPositionLimit[i]) {
        q_result[i] = std::max(model_.lowerPositionLimit[i],
                               std::min(q_result[i], model_.upperPositionLimit[i]));
      }
    }
  }
  
  // Did not converge, but return best result
  return false;
}

bool IeirDynamics::solveIKPosition(
  int frame_id,
  const Eigen::Vector3d& target_position,
  const Eigen::VectorXd& q_init,
  Eigen::VectorXd& q_result,
  int max_iter,
  double eps,
  double damping,
  double dt)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  
  if (frame_id < 0 || frame_id >= static_cast<int>(model_.nframes)) {
    throw std::runtime_error("Invalid frame_id for IK");
  }
  
  if (q_init.size() != model_.nq) {
    throw std::runtime_error("q_init size mismatch");
  }
  
  q_result = q_init;
  pinocchio::FrameIndex fid = static_cast<pinocchio::FrameIndex>(frame_id);
  
  Eigen::MatrixXd J_full(6, model_.nv);
  
  for (int iter = 0; iter < max_iter; ++iter) {
    // Compute FK at current q
    pinocchio::forwardKinematics(model_, *data_, q_result);
    pinocchio::updateFramePlacement(model_, *data_, fid);
    
    // Position error only (3D, in world frame)
    Eigen::Vector3d err = target_position - data_->oMf[fid].translation();
    
    // Check convergence
    if (err.norm() < eps) {
      return true;
    }
    
    // Compute full Jacobian, then extract linear part (top 3 rows) in world frame
    J_full.setZero();
    pinocchio::computeFrameJacobian(
      model_, *data_, q_result, fid,
      pinocchio::LOCAL_WORLD_ALIGNED, J_full);
    
    // Use only the linear (translation) part: rows 0-2
    Eigen::MatrixXd Jv = J_full.topRows(3);  // 3 x nv
    
    // Damped least-squares: dq = Jv^T * (Jv * Jv^T + lambda^2 * I)^{-1} * err
    Eigen::Matrix3d JJt = Jv * Jv.transpose();
    JJt.diagonal().array() += damping * damping;
    
    Eigen::VectorXd dq = Jv.transpose() * JJt.ldlt().solve(err);
    
    // Update q with step size
    q_result = pinocchio::integrate(model_, q_result, dt * dq);
    
    // Clamp to joint limits
    for (int i = 0; i < model_.nq; ++i) {
      if (model_.lowerPositionLimit[i] < model_.upperPositionLimit[i]) {
        q_result[i] = std::max(model_.lowerPositionLimit[i],
                               std::min(q_result[i], model_.upperPositionLimit[i]));
      }
    }
  }
  
  // Did not converge, but return best result
  return false;
}

Eigen::VectorXd IeirDynamics::applyNullSpaceMotion(
  int frame_id,
  const Eigen::VectorXd& q_in,
  const Eigen::VectorXd& null_bias,
  double alpha,
  double damping)
{
  if (!initialized_) {
    throw std::runtime_error("IeirDynamics not initialized");
  }
  if (frame_id < 0 || frame_id >= static_cast<int>(model_.nframes)) {
    throw std::runtime_error("Invalid frame_id for null-space motion");
  }
  if (q_in.size() != model_.nq || null_bias.size() != model_.nv) {
    throw std::runtime_error("Size mismatch in applyNullSpaceMotion");
  }

  pinocchio::FrameIndex fid = static_cast<pinocchio::FrameIndex>(frame_id);

  // Compute FK and Jacobian at q_in
  pinocchio::forwardKinematics(model_, *data_, q_in);
  pinocchio::updateFramePlacement(model_, *data_, fid);

  Eigen::MatrixXd J(6, model_.nv);
  J.setZero();
  pinocchio::computeFrameJacobian(
    model_, *data_, q_in, fid,
    pinocchio::LOCAL, J);  // Must match solveIK's frame for consistent null-space

  // Damped pseudoinverse: J_pinv = J^T * (J * J^T + lambda^2 * I)^{-1}
  Eigen::MatrixXd JJt = J * J.transpose();  // 6x6
  JJt.diagonal().array() += damping * damping;
  Eigen::MatrixXd J_pinv = J.transpose() * JJt.ldlt().solve(
    Eigen::MatrixXd::Identity(6, 6));  // nv x 6

  // Null-space projector: N = I - J_pinv * J   (nv x nv)
  Eigen::MatrixXd N = Eigen::MatrixXd::Identity(model_.nv, model_.nv) - J_pinv * J;

  // Project bias into null space and apply
  Eigen::VectorXd dq_null = alpha * N * null_bias;

  // Integrate
  Eigen::VectorXd q_out = pinocchio::integrate(model_, q_in, dq_null);

  // Clamp to joint limits
  for (int i = 0; i < model_.nq; ++i) {
    if (model_.lowerPositionLimit[i] < model_.upperPositionLimit[i]) {
      q_out[i] = std::max(model_.lowerPositionLimit[i],
                           std::min(q_out[i], model_.upperPositionLimit[i]));
    }
  }

  return q_out;
}

}  // namespace ieir_dynamics
