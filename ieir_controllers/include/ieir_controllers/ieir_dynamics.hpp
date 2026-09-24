#ifndef IEIR_CONTROLLERS__IEIR_DYNAMICS_HPP_
#define IEIR_CONTROLLERS__IEIR_DYNAMICS_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace ieir_dynamics
{

/**
 * @brief Pure algorithm class for robot dynamics computation using Pinocchio
 * 
 * This class encapsulates all Pinocchio-related computations (gravity, impedance, etc.)
 * and provides a clean interface for controllers to use.
 * It is designed to be real-time safe and independent of ROS infrastructure.
 */
class IeirDynamics
{
public:
  /**
   * @brief Constructor
   */
  IeirDynamics() = default;

  /**
   * @brief Initialize the dynamics model from URDF string
   * @param urdf_string URDF content as XML string
   * @return true if successful, false otherwise
   */
  bool initFromURDF(const std::string& urdf_string);

  /**
   * @brief Update the current joint state
   * @param q Joint positions (size must match model.nq)
   * @param v Joint velocities (size must match model.nv)
   */
  void updateState(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  /**
   * @brief Compute generalized gravity torque g(q)
   * @return Gravity torque vector (size model.nv)
   */
  Eigen::VectorXd computeGravity();

  /**
   * @brief Compute impedance control torque
   * @param q_desired Desired joint positions
   * @param v_desired Desired joint velocities
   * @param stiffness Diagonal stiffness matrix (Kp)
   * @param damping Diagonal damping matrix (Kd)
   * @return Impedance torque vector: tau = Kp*(qd-q) + Kd*(vd-v)
   */
  Eigen::VectorXd computeImpedance(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired,
    const Eigen::VectorXd& stiffness,
    const Eigen::VectorXd& damping);

  /**
   * @brief Get the number of configuration variables (nq)
   */
  int getNq() const { return model_.nq; }

  /**
   * @brief Get the number of velocity variables (nv)
   */
  int getNv() const { return model_.nv; }

  /**
   * @brief Get the number of joints (excluding universe)
   */
  int getNumJoints() const { return model_.njoints - 1; }

  /**
   * @brief Check if model is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get joint name by index
   * @param joint_id Joint index (1-based, 0 is universe)
   */
  std::string getJointName(int joint_id) const;

  /**
   * @brief Get joint configuration index
   * @param joint_id Joint index
   */
  int getJointIdxQ(int joint_id) const;

  /**
   * @brief Get joint velocity index
   * @param joint_id Joint index
   */
  int getJointIdxV(int joint_id) const;

  /**
   * @brief Get effort limit for a joint
   * @param joint_id Joint index
   */
  double getEffortLimit(int joint_id) const;

  /**
   * @brief Check if a joint has DoF
   * @param joint_id Joint index
   */
  bool isActuated(int joint_id) const;

  /**
   * @brief Get neutral configuration
   */
  Eigen::VectorXd getNeutralConfiguration() const;

  /**
   * @brief Get frame ID by name
   * @param frame_name Name of the frame (link name from URDF)
   * @return Frame index, or -1 if not found
   */
  int getFrameId(const std::string& frame_name) const;

  /**
   * @brief Compute forward kinematics for a specific frame
   * @param frame_id Frame index from getFrameId()
   * @return SE3 transform of the frame in world coordinates
   */
  pinocchio::SE3 computeFK(int frame_id);

  /**
   * @brief Compute the 6xN frame Jacobian in LOCAL_WORLD_ALIGNED convention
   * @param frame_id Frame index
   * @return 6xNv Jacobian matrix [linear; angular]
   */
  Eigen::MatrixXd computeFrameJacobian(int frame_id);

  /**
   * @brief Solve inverse kinematics using damped least-squares (CLIK)
   * @param frame_id Target frame index
   * @param target_pose Desired SE3 pose
   * @param q_init Initial joint configuration guess
   * @param q_result Output: solved joint configuration
   * @param max_iter Maximum IK iterations (default 100)
   * @param eps Convergence threshold (default 1e-4)
   * @param damping Damping factor for singularity robustness (default 1e-3)
   * @param dt Step size (default 1.0)
   * @return true if converged within max_iter
   */
  bool solveIK(
    int frame_id,
    const pinocchio::SE3& target_pose,
    const Eigen::VectorXd& q_init,
    Eigen::VectorXd& q_result,
    int max_iter = 100,
    double eps = 1e-4,
    double damping = 1e-3,
    double dt = 1.0);

  /**
   * @brief Solve position-only IK (ignores orientation)
   * @param frame_id Target frame index
   * @param target_position Desired 3D position in world frame
   * @param q_init Initial joint configuration guess
   * @param q_result Output: solved joint configuration
   * @param max_iter Maximum IK iterations (default 100)
   * @param eps Convergence threshold in meters (default 1e-4)
   * @param damping Damping factor (default 1e-3)
   * @param dt Step size (default 1.0)
   * @return true if converged within max_iter
   */
  bool solveIKPosition(
    int frame_id,
    const Eigen::Vector3d& target_position,
    const Eigen::VectorXd& q_init,
    Eigen::VectorXd& q_result,
    int max_iter = 100,
    double eps = 1e-4,
    double damping = 1e-3,
    double dt = 1.0);

  /**
   * @brief Apply null-space motion to a joint configuration without affecting EE pose.
   *
   * Computes the null-space projector N = I - J_pinv * J for the given frame,
   * then applies q_out = q + alpha * N * bias.
   *
   * @param frame_id    Target EE frame index
   * @param q_in        Current joint configuration
   * @param null_bias   Desired joint-space velocity (only the null-space component is kept)
   * @param alpha       Gain / step size
   * @param damping     Damping for pseudoinverse computation
   * @return            Updated joint configuration with null-space motion applied
   */
  Eigen::VectorXd applyNullSpaceMotion(
    int frame_id,
    const Eigen::VectorXd& q_in,
    const Eigen::VectorXd& null_bias,
    double alpha = 1.0,
    double damping = 1e-3);

  /**
   * @brief Get the Pinocchio model (const reference)
   */
  const pinocchio::Model& getModel() const { return model_; }
  pinocchio::Data* getData() { return data_.get(); }

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  
  Eigen::VectorXd q_current_;
  Eigen::VectorXd v_current_;
  
  bool initialized_ = false;
};

}  // namespace ieir_dynamics

#endif  // IEIR_CONTROLLERS__IEIR_DYNAMICS_HPP_
