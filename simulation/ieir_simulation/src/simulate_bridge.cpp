#include "ieir_simulation/simulate_bridge.hpp"

#include <algorithm>
#include <cctype>
#include <exception>

/**
 * @brief 仿真器类的构造函数
 * 
 * @param d mujoco模型数据指针
 * @param m mujoco模型指针
 * @param sim 仿真UI类的引用
 */
SimulateBridge::SimulateBridge(mjData* d, mjModel* m, mujoco::Simulate& sim) : Node("mujoco_simulator_node"),mj_sim_(sim)
{
    // FIXME:解决穿模问题
    // 将模型数据指针复制进来
    mj_data_ = d;
    mj_model_ = m;

    // 修改仿真参数
    mj_model_->opt.timestep = 0.001; // 修改仿真频率为1KHz

    // 从yaml中读取参数 (支持通过环境变量 IEIR_MUJOCO_CONFIG 切换配置文件)
    const std::string pkg_path = ament_index_cpp::get_package_share_directory("ieir_simulation"); // 获取包路径
    const char* config_env = std::getenv("IEIR_MUJOCO_CONFIG");
    const std::string config_filename = config_env ? config_env : "simulate.yaml";
    const std::string yaml_path = pkg_path + "/config/" + config_filename; // 拼接yaml路径
    YAML::Node config = YAML::LoadFile(yaml_path)["mujoco_simulator"];
    const char* panel_mode_env = std::getenv("IEIR_MUJOCO_PANEL_MODE");
    const std::string panel_mode = panel_mode_env ? panel_mode_env :
        (config["panelMode"] ? config["panelMode"].as<std::string>() : "simulate");
    panelMode_ = ParsePanelMode_(panel_mode);
    autoFreezeOnFirstRealState_ = (panelMode_ == PanelMode::TargetEditor);
    jointCommandsTopic_ = config["jointCommandsTopic"].as<std::string>();
    jointGainsTopic_ = config["jointGainsTopic"] ?
        config["jointGainsTopic"].as<std::string>() : "/ctrl/gains";
    UnPauseServiceService_ = config["unPauseService"].as<std::string>();
    realJointStatesTopic_ = config["realJointStatesTopic"] ?
        config["realJointStatesTopic"].as<std::string>() : "/joint_states";
    trajectoryCommandTopic_ = config["trajectoryCommandTopic"] ?
        config["trajectoryCommandTopic"].as<std::string>() : "/joint_position_command";
    trajectoryDuration_ = config["trajectoryDuration"] ?
        config["trajectoryDuration"].as<double>() : 5.0;
    if(const char* duration_env = std::getenv("IEIR_MUJOCO_TRAJECTORY_DURATION")) {
        try {
            trajectoryDuration_ = std::stod(duration_env);
        } catch(const std::exception&) {
            RCLCPP_WARN(
                this->get_logger(),
                "Invalid IEIR_MUJOCO_TRAJECTORY_DURATION='%s', using %.2f",
                duration_env,
                trajectoryDuration_);
        }
    }
    initPauseFlag_ = config["initPauseFlag"].as<bool>();
    modelTableFlag_ = config["modelTableFlag"].as<bool>();
    
    // 如有设置,则暂停仿真
    if(initPauseFlag_ || !PhysicsEnabled()) mj_sim_.run = 0;

    // 读取模型内容参数
    ReadModel();
    // 输出相关的ID
    ShowModel();

    // 创建话题通信接口
    if(PhysicsEnabled()) {
        jointCommandsSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            jointCommandsTopic_,
            1,
            std::bind(&SimulateBridge::JointCommandSubCallBack, this, _1)
        );
        jointGainsSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            jointGainsTopic_,
            1,
            std::bind(&SimulateBridge::JointGainsSubCallBack, this, _1)
        );
        jointStatePub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/joint_states",
            1
        );
    } else {
        realJointStateSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            realJointStatesTopic_,
            10,
            std::bind(&SimulateBridge::RealJointStateSubCallBack, this, _1)
        );
        trajectoryPub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
            trajectoryCommandTopic_,
            1
        );
        switchControllerClient_ =
            this->create_client<controller_manager_msgs::srv::SwitchController>(
                "/controller_manager/switch_controller");
        teleopPrepareClient_ =
            this->create_client<std_srvs::srv::Trigger>("/teleop/prepare");
        teleopToggleClient_ =
            this->create_client<std_srvs::srv::Trigger>("/teleop/toggle");
        teleopExitClient_ =
            this->create_client<std_srvs::srv::Trigger>("/teleop/exit");
        startEditServer_ = this->create_service<std_srvs::srv::Trigger>(
            "/mujoco_panel/start_edit",
            std::bind(&SimulateBridge::StartEditServiceCallBack, this, _1, _2)
        );
        sendTargetServer_ = this->create_service<std_srvs::srv::Trigger>(
            "/mujoco_panel/send_target",
            std::bind(&SimulateBridge::SendTargetServiceCallBack, this, _1, _2)
        );
        cancelEditServer_ = this->create_service<std_srvs::srv::Trigger>(
            "/mujoco_panel/cancel_edit",
            std::bind(&SimulateBridge::CancelEditServiceCallBack, this, _1, _2)
        );
        RCLCPP_INFO(
            this->get_logger(),
            "MuJoCo panel mode active. Follow topic: %s, trajectory topic: %s",
            realJointStatesTopic_.c_str(),
            trajectoryCommandTopic_.c_str());
    }
    worldFramePub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // 初始化服务通信接口
    UnPauseServer_ = this->create_service<std_srvs::srv::Empty>(
        UnPauseServiceService_,
        std::bind(&SimulateBridge::UnPauseServiceCallBack, this, _1, _2)
    );

    // 初始化通信vector长度
    const size_t num_joints = modelParam_.jointName.size();
    jointCommands_.name = modelParam_.jointName;
    jointCommands_.position.resize(num_joints, 0.0);
    jointCommands_.velocity.resize(num_joints, 0.0);
    jointCommands_.effort.resize(num_joints, 0.0);
    jointStiffnessCommands_.resize(num_joints, 0.0);
    jointDampingCommands_.resize(num_joints, 0.0);
}

SimulateBridge::PanelMode SimulateBridge::ParsePanelMode_(const std::string& mode) const {
    std::string normalized = mode;
    std::transform(normalized.begin(), normalized.end(), normalized.begin(),
        [](unsigned char c) { return static_cast<char>(std::tolower(c)); });

    if(normalized == "simulate" || normalized == "physics") return PanelMode::Simulate;
    if(normalized == "mirror_real" || normalized == "mirror") return PanelMode::MirrorReal;
    if(normalized == "target_editor" || normalized == "editor") return PanelMode::TargetEditor;

    RCLCPP_WARN(
        this->get_logger(),
        "Unknown panelMode '%s', falling back to simulate",
        mode.c_str());
    return PanelMode::Simulate;
}

bool SimulateBridge::IsArmJoint_(const std::string& joint_name) const {
    return joint_name.rfind("left_joint_", 0) == 0 ||
           joint_name.rfind("right_joint_", 0) == 0;
}

bool SimulateBridge::RequestControllerSwitch_(
    const std::vector<std::string>& activate,
    const std::vector<std::string>& deactivate,
    const std::string& label)
{
    if(PhysicsEnabled()) return false;
    if(!switchControllerClient_) return false;
    if(!switchControllerClient_->service_is_ready() &&
       !switchControllerClient_->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(
            this->get_logger(),
            "controller_manager switch service is not available; cannot %s",
            label.c_str());
        return false;
    }

    auto req =
        std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    req->activate_controllers = activate;
    req->deactivate_controllers = deactivate;
    req->strictness =
        controller_manager_msgs::srv::SwitchController::Request::BEST_EFFORT;
    req->activate_asap = false;

    switchControllerClient_->async_send_request(
        req,
        [logger = this->get_logger(), label](
            rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedFuture future) {
            const auto resp = future.get();
            if(resp && resp->ok) {
                RCLCPP_INFO(logger, "controller switch OK: %s", label.c_str());
            } else {
                RCLCPP_WARN(logger, "controller switch failed: %s", label.c_str());
            }
        });
    return true;
}

bool SimulateBridge::RequestTriggerService_(
    const rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr& client,
    const std::string& label)
{
    if(PhysicsEnabled()) return false;
    if(!client) return false;
    if(!client->service_is_ready() &&
       !client->wait_for_service(std::chrono::milliseconds(100))) {
        RCLCPP_WARN(
            this->get_logger(),
            "%s service is not available; is teleop:=true set in real_robot.launch.py?",
            label.c_str());
        return false;
    }

    auto req = std::make_shared<std_srvs::srv::Trigger::Request>();
    client->async_send_request(
        req,
        [logger = this->get_logger(), label](
            rclcpp::Client<std_srvs::srv::Trigger>::SharedFuture future) {
            const auto resp = future.get();
            if(resp && resp->success) {
                RCLCPP_INFO(logger, "%s OK: %s", label.c_str(), resp->message.c_str());
            } else if(resp) {
                RCLCPP_WARN(logger, "%s failed: %s", label.c_str(), resp->message.c_str());
            } else {
                RCLCPP_WARN(logger, "%s failed: no response", label.c_str());
            }
        });
    return true;
}

bool SimulateBridge::PublishHomeTrajectory_()
{
    if(!trajectoryPub_) return false;

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->get_clock()->now();
    trajectory_msgs::msg::JointTrajectoryPoint point;

    for(size_t i = 0; i < modelParam_.jointName.size(); i++) {
        if(!IsArmJoint_(modelParam_.jointName[i])) continue;
        trajectory.joint_names.push_back(modelParam_.jointName[i]);
        point.positions.push_back(0.0);
        point.velocities.push_back(0.0);
    }

    if(trajectory.joint_names.empty()) return false;

    const double duration = std::max(0.1, trajectoryDuration_);
    point.time_from_start.sec = static_cast<int32_t>(duration);
    point.time_from_start.nanosec =
        static_cast<uint32_t>((duration - point.time_from_start.sec) * 1.0e9);
    trajectory.points.push_back(point);
    trajectoryPub_->publish(trajectory);
    return true;
}

void SimulateBridge::WriteJointStateToModel_(
    size_t joint_idx,
    double position,
    double velocity)
{
    if(joint_idx >= modelParam_.jointMujocoIds.size()) return;
    if(!std::isfinite(position)) return;

    const int mj_joint_id = modelParam_.jointMujocoIds[joint_idx];
    if(mj_joint_id < 0 || mj_joint_id >= mj_model_->njnt) return;

    const int qpos_adr = mj_model_->jnt_qposadr[mj_joint_id];
    if(qpos_adr >= 0) {
        if(mj_model_->jnt_limited[mj_joint_id]) {
            const double lo = mj_model_->jnt_range[2 * mj_joint_id + 0];
            const double hi = mj_model_->jnt_range[2 * mj_joint_id + 1];
            position = std::clamp(position, lo, hi);
        }
        mj_data_->qpos[qpos_adr] = position;
    }

    const int qvel_adr = mj_model_->jnt_dofadr[mj_joint_id];
    if(qvel_adr >= 0) {
        mj_data_->qvel[qvel_adr] = std::isfinite(velocity) ? velocity : 0.0;
    }
}

/**
 * @brief 电机命令接收回调
 * 
 * @param jointCommand 接收到的电机命令
 */
void SimulateBridge::JointCommandSubCallBack(const sensor_msgs::msg::JointState::SharedPtr jointCommand){

    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;
    
    // 建立临时名字到ID映射，如果还没有的话 (或者可以存成成员变量优化)
    static std::map<std::string, int> jointNameToIdx;
    if(jointNameToIdx.empty()) {
        for(size_t i=0; i<modelParam_.jointName.size(); i++) {
            jointNameToIdx[modelParam_.jointName[i]] = i;
        }
    }

    // 遍历收到的命令
    for(size_t i=0; i<jointCommand->name.size(); i++) {
        std::string name = jointCommand->name[i];
        if(jointNameToIdx.find(name) == jointNameToIdx.end()) continue; // 未知关节
        
        int jointIdx = jointNameToIdx[name];
        int actuatorIdx = modelParam_.jointToActuatorIdx[jointIdx];
        
        if(actuatorIdx >= 0 && actuatorIdx < mj_model_->nu) {
            if(i < jointCommand->position.size()) {
                jointCommands_.position[jointIdx] = jointCommand->position[i];
            }
            if(i < jointCommand->velocity.size()) {
                jointCommands_.velocity[jointIdx] = jointCommand->velocity[i];
            }
            if(i < jointCommand->effort.size()) {
                jointCommands_.effort[jointIdx] = jointCommand->effort[i];
            }
        }
    }
}

void SimulateBridge::JointGainsSubCallBack(const sensor_msgs::msg::JointState::SharedPtr jointGains){

    if(modelParam_.readErrorFlag) return;

    static std::map<std::string, int> jointNameToIdx;
    if(jointNameToIdx.empty()) {
        for(size_t i=0; i<modelParam_.jointName.size(); i++) {
            jointNameToIdx[modelParam_.jointName[i]] = i;
        }
    }

    for(size_t i=0; i<jointGains->name.size(); i++) {
        std::string name = jointGains->name[i];
        if(jointNameToIdx.find(name) == jointNameToIdx.end()) continue;

        int jointIdx = jointNameToIdx[name];
        if(i < jointGains->position.size()) {
            jointStiffnessCommands_[jointIdx] = std::max(0.0, jointGains->position[i]);
        }
        if(i < jointGains->velocity.size()) {
            jointDampingCommands_[jointIdx] = std::max(0.0, jointGains->velocity[i]);
        }
    }
}

void SimulateBridge::RealJointStateSubCallBack(
    const sensor_msgs::msg::JointState::SharedPtr jointState)
{
    if(modelParam_.readErrorFlag) return;
    if(PhysicsEnabled()) return;
    if(editMode_) return;

    std::map<std::string, int> jointNameToIdx;
    for(size_t i = 0; i < modelParam_.jointName.size(); i++) {
        jointNameToIdx[modelParam_.jointName[i]] = i;
    }

    for(size_t i = 0; i < jointState->name.size(); i++) {
        const auto it = jointNameToIdx.find(jointState->name[i]);
        if(it == jointNameToIdx.end()) continue;
        if(i >= jointState->position.size()) continue;

        const double velocity =
            (i < jointState->velocity.size()) ? jointState->velocity[i] : 0.0;
        WriteJointStateToModel_(static_cast<size_t>(it->second), jointState->position[i], velocity);
    }
    mj_forward(mj_model_, mj_data_);

    if(autoFreezeOnFirstRealState_) {
        editMode_ = true;
        autoFreezeOnFirstRealState_ = false;
        RCLCPP_INFO(
            this->get_logger(),
            "Captured first real joint state; target editor is now frozen for slider editing");
    }
}

void SimulateBridge::StartEditServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    const std::string message = StartEdit();
    response->success = message.rfind("OK:", 0) == 0;
    response->message = message;
}

std::string SimulateBridge::StartEdit()
{
    if(PhysicsEnabled()) {
        return "ERROR: start_edit is only available in MuJoCo panel mode";
    }

    SwitchToJointPosition();
    mj_sim_.run = 0;
    editMode_ = true;
    return "OK: edit mode enabled; adjust MuJoCo joint sliders, then press S or call /mujoco_panel/send_target";
}

void SimulateBridge::SendTargetServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    const std::string message = SendTarget();
    response->success = message.rfind("OK:", 0) == 0;
    response->message = message;
}

std::string SimulateBridge::SendTarget()
{
    if(PhysicsEnabled()) {
        return "ERROR: send_target is only available in MuJoCo panel mode";
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->get_clock()->now();
    trajectory_msgs::msg::JointTrajectoryPoint point;

    for(size_t i = 0; i < modelParam_.jointName.size(); i++) {
        if(!IsArmJoint_(modelParam_.jointName[i])) continue;
        trajectory.joint_names.push_back(modelParam_.jointName[i]);
        point.positions.push_back(ReadJointPosition_(i));
        point.velocities.push_back(0.0);
    }

    if(trajectory.joint_names.empty()) {
        return "ERROR: no arm joints found in MuJoCo model";
    }

    const double duration = std::max(0.1, trajectoryDuration_);
    point.time_from_start.sec = static_cast<int32_t>(duration);
    point.time_from_start.nanosec =
        static_cast<uint32_t>((duration - point.time_from_start.sec) * 1.0e9);
    trajectory.points.push_back(point);
    trajectoryPub_->publish(trajectory);

    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    return "OK: target trajectory published; panel returned to real-state mirror mode";
}

void SimulateBridge::CancelEditServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
{
    (void)request;
    const std::string message = CancelEdit();
    response->success = message.rfind("OK:", 0) == 0;
    response->message = message;
}

std::string SimulateBridge::CancelEdit()
{
    if(PhysicsEnabled()) {
        return "ERROR: cancel_edit is only available in MuJoCo panel mode";
    }

    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    return "OK: edit mode cancelled; panel returned to real-state mirror mode";
}

std::string SimulateBridge::SwitchToJointPosition()
{
    if(PhysicsEnabled()) return "ERROR: controller switch is unavailable in simulation mode";
    const bool requested = RequestControllerSwitch_(
        {"gravity_compensation_controller", "joint_position_controller"},
        {"cartesian_position_controller"},
        "joint_position");
    return requested ? "OK: requested joint_position_controller"
                     : "ERROR: failed to request joint_position_controller";
}

std::string SimulateBridge::SwitchToGravity()
{
    if(PhysicsEnabled()) return "ERROR: controller switch is unavailable in simulation mode";
    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    const bool requested = RequestControllerSwitch_(
        {"gravity_compensation_controller"},
        {"joint_position_controller", "cartesian_position_controller"},
        "gravity");
    return requested ? "OK: requested gravity compensation mode"
                     : "ERROR: failed to request gravity compensation mode";
}

std::string SimulateBridge::SwitchToCartesianPosition()
{
    if(PhysicsEnabled()) return "ERROR: controller switch is unavailable in simulation mode";
    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    const bool requested = RequestControllerSwitch_(
        {"gravity_compensation_controller", "cartesian_position_controller"},
        {"joint_position_controller"},
        "cartesian_position");
    return requested ? "OK: requested cartesian_position_controller"
                     : "ERROR: failed to request cartesian_position_controller";
}

std::string SimulateBridge::GoHome()
{
    if(PhysicsEnabled()) return "ERROR: home is unavailable in simulation mode";
    SwitchToJointPosition();
    pendingHomePublish_ = true;
    homePublishAt_ = std::chrono::steady_clock::now() + std::chrono::milliseconds(500);
    return "OK: requested home trajectory after joint_position_controller switch";
}

std::string SimulateBridge::TeleopPrepare()
{
    if(PhysicsEnabled()) return "ERROR: teleop prepare is unavailable in simulation mode";
    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    const bool requested = RequestTriggerService_(teleopPrepareClient_, "teleop prepare");
    return requested ? "OK: requested teleop prepare/alignment"
                     : "ERROR: failed to request teleop prepare/alignment";
}

std::string SimulateBridge::TeleopToggle()
{
    if(PhysicsEnabled()) return "ERROR: teleop toggle is unavailable in simulation mode";
    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    const bool requested = RequestTriggerService_(teleopToggleClient_, "teleop toggle");
    return requested ? "OK: requested teleop enable/disable toggle"
                     : "ERROR: failed to request teleop enable/disable toggle";
}

std::string SimulateBridge::TeleopExit()
{
    if(PhysicsEnabled()) return "ERROR: teleop exit is unavailable in simulation mode";
    editMode_ = false;
    autoFreezeOnFirstRealState_ = false;
    const bool requested = RequestTriggerService_(teleopExitClient_, "teleop exit");
    return requested ? "OK: requested teleop exit"
                     : "ERROR: failed to request teleop exit";
}

void SimulateBridge::ProcessPanelTick()
{
    if(!pendingHomePublish_) return;
    if(std::chrono::steady_clock::now() < homePublishAt_) return;

    pendingHomePublish_ = false;
    if(PublishHomeTrajectory_()) {
        RCLCPP_INFO(this->get_logger(), "home trajectory published");
    } else {
        RCLCPP_WARN(this->get_logger(), "failed to publish home trajectory");
    }
}

double SimulateBridge::ReadJointPosition_(size_t joint_idx) const {
    if(joint_idx < modelParam_.jointPosDataAdr.size()) {
        const int adr = modelParam_.jointPosDataAdr[joint_idx];
        if(adr >= 0) return mj_data_->sensordata[adr];
    }
    if(joint_idx < modelParam_.jointMujocoIds.size()) {
        const int mj_joint_id = modelParam_.jointMujocoIds[joint_idx];
        const int qpos_adr = mj_model_->jnt_qposadr[mj_joint_id];
        if(qpos_adr >= 0) return mj_data_->qpos[qpos_adr];
    }
    return 0.0;
}

double SimulateBridge::ReadJointVelocity_(size_t joint_idx) const {
    if(joint_idx < modelParam_.jointVelDataAdr.size()) {
        const int adr = modelParam_.jointVelDataAdr[joint_idx];
        if(adr >= 0) return mj_data_->sensordata[adr];
    }
    if(joint_idx < modelParam_.jointMujocoIds.size()) {
        const int mj_joint_id = modelParam_.jointMujocoIds[joint_idx];
        const int qvel_adr = mj_model_->jnt_dofadr[mj_joint_id];
        if(qvel_adr >= 0) return mj_data_->qvel[qvel_adr];
    }
    return 0.0;
}

void SimulateBridge::ApplyMitCommand() {
    if(modelParam_.readErrorFlag) return;

    for(size_t joint_idx = 0; joint_idx < modelParam_.jointName.size(); ++joint_idx) {
        if(joint_idx >= modelParam_.jointToActuatorIdx.size()) continue;
        const int actuator_idx = modelParam_.jointToActuatorIdx[joint_idx];
        if(actuator_idx < 0 || actuator_idx >= mj_model_->nu) continue;

        const double q = ReadJointPosition_(joint_idx);
        const double qd = ReadJointVelocity_(joint_idx);
        const double q_des = jointCommands_.position[joint_idx];
        const double qd_des = jointCommands_.velocity[joint_idx];
        const double torque_ff = jointCommands_.effort[joint_idx];
        const double kp = jointStiffnessCommands_[joint_idx];
        const double kd = jointDampingCommands_[joint_idx];

        double torque = torque_ff + kp * (q_des - q) + kd * (qd_des - qd);
        if(!std::isfinite(torque)) torque = 0.0;

        if(mj_model_->actuator_ctrllimited[actuator_idx]) {
            const double lo = mj_model_->actuator_ctrlrange[2 * actuator_idx + 0];
            const double hi = mj_model_->actuator_ctrlrange[2 * actuator_idx + 1];
            torque = std::clamp(torque, lo, hi);
        }
        mj_data_->ctrl[actuator_idx] = torque;
    }
}

/**
 * @brief 继续mujoco物理仿真的服务回调
 * 
 * @param request 
 * @param response 
 */
void SimulateBridge::UnPauseServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
) {
    // 避免未使用变量警告
    (void)request; 
    (void)response;
    // 如果初始暂停了物理仿真,则继续
    if(initPauseFlag_ && mj_sim_.run == 0) mj_sim_.run = 1;

    // 应用第一个关键帧作为初始位置
    if(modelParam_.keyFrameCount > 0 ){ 
        mj_resetDataKeyframe(mj_model_, mj_data_, 0);
    }
    
}

void SimulateBridge::JointStatePublish() {
    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;
    if(!PhysicsEnabled()) return;

    // 发布关节信息
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = this->get_clock()->now();
    size_t num_joints = modelParam_.jointName.size();
    jointState.name.resize(num_joints);
    jointState.position.resize(num_joints);
    jointState.velocity.resize(num_joints);
    jointState.effort.resize(num_joints);

    for (size_t i = 0; i < num_joints; i++) {
        jointState.name[i] = modelParam_.jointName[i];
        
        // 使用映射表查找传感器索引
        int tor_adr = modelParam_.jointEffortDataAdr[i];
        double tor_scale = modelParam_.jointEffortDataScale[i];

        jointState.position[i] = ReadJointPosition_(i);
        jointState.velocity[i] = ReadJointVelocity_(i);

        if (tor_adr >= 0)
            jointState.effort[i]   = mj_data_->sensordata[tor_adr] * tor_scale;
        else 
            jointState.effort[i]   = 0;
    }
    jointStatePub_->publish(jointState);

    // 发布世界坐标信息
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    if(modelParam_.realPosHeadID != 99999) {
        transform.transform.translation.x = mj_data_->sensordata[modelParam_.realPosHeadID+0];
        transform.transform.translation.y = mj_data_->sensordata[modelParam_.realPosHeadID+1];
        transform.transform.translation.z = mj_data_->sensordata[modelParam_.realPosHeadID+2];
    }
    if(modelParam_.imuQuatHeadID != 99999) {
        transform.transform.rotation.w = mj_data_->sensordata[modelParam_.imuQuatHeadID + 0];
        transform.transform.rotation.x = mj_data_->sensordata[modelParam_.imuQuatHeadID + 1];
        transform.transform.rotation.y = mj_data_->sensordata[modelParam_.imuQuatHeadID + 2];
        transform.transform.rotation.z = mj_data_->sensordata[modelParam_.imuQuatHeadID + 3];
    }
    worldFramePub_->sendTransform(transform);
}

/**
 * @brief 输出读取到的MJCF文件信息
 * 
 */
void SimulateBridge::ShowModel(){

    using namespace tabulate;

    // 总表
    Table modelInfo;
    // 子项
    Table allJointsInfo;
    Table allLinksInfo;
    Table allSensorsInfo;
    // 子项中的表
    Table jointInfo;
    Table linkInfo;
    Table sensorInfo;

    // 总表头
    modelInfo.add_row({"model information"});
    modelInfo.add_row(RowStream{} << "model name: " + modelParam_.modelName
                                  + " time step: " + std::to_string(modelParam_.timeStep) + "s");
    modelInfo.format().font_align(FontAlign::center);
    modelInfo[1].format().hide_border_top().hide_border_bottom().font_align(FontAlign::center);


    // 输出joint信息
    allJointsInfo.add_row({"joints information"});
    allJointsInfo.format().font_align(FontAlign::center);
    allJointsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    jointInfo.add_row({"ID", "name", "posLimit(rad)", "torLimit(Nm)", "friction", "damping"});
    jointInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.jointName.size(); i++){
        // 组织限幅字符串
        std::ostringstream posLimitStr;
        std::ostringstream torLimitStr;
        posLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointPosRange[i].first << " ~ " << modelParam_.jointPosRange[i].second;
        torLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointTorqueRange[i].first << " ~ " << modelParam_.jointTorqueRange[i].second;
        jointInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.jointName[i] << posLimitStr.str() << torLimitStr.str() 
            << modelParam_.jointFri[i] << modelParam_.jointDamp[i]);
            jointInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) jointInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allJointsInfo.add_row({jointInfo});
    allJointsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出link信息
    allLinksInfo.add_row({"links information"});
    allLinksInfo.format().font_align(FontAlign::center);
    allLinksInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    linkInfo.add_row({"ID", "name", "mass(kg)"});
    linkInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.linkNames.size(); i++){
        linkInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.linkNames[i] << modelParam_.linkMass[i]);
        linkInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) linkInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allLinksInfo.add_row({linkInfo});
    allLinksInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出sensor信息
    allSensorsInfo.add_row({"sensors information"});
    allSensorsInfo.format().font_align(FontAlign::center);
    allSensorsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    sensorInfo.add_row({"ID", "name", "type", "attach", "head"});
    sensorInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.sensorType.size(); i++){
        std::string headIDName;
        if(i == modelParam_.jointPosHeadID) headIDName = "joint pos head";
        else if(i == modelParam_.jointVelHeadID) headIDName = "joint vel head";
        else if(i == modelParam_.jointTorHeadID) headIDName = "joint torque head";
        else if(i == modelParam_.imuQuatHeadID) headIDName = "imu quat head";
        else if(i == modelParam_.imuGyroHeadID) headIDName = "imu gyro head";
        else if(i == modelParam_.imuAccHeadId) headIDName = "imu acc head";
        else if(i == modelParam_.realPosHeadID) headIDName = "real pos head";
        else if(i == modelParam_.realVelHeadID) headIDName = "real vel head";
        else headIDName = "";
        sensorInfo.add_row(RowStream{} 
            << i << modelParam_.sensorType[i][0] << modelParam_.sensorType[i][1] << modelParam_.sensorType[i][2] << headIDName);
        sensorInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) sensorInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allSensorsInfo.add_row({sensorInfo});
    allSensorsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 将子表加入总表
    modelInfo.add_row({allJointsInfo});
    modelInfo.add_row({allLinksInfo});
    modelInfo.add_row({allSensorsInfo});

    // 输出模型信息
    if(modelTableFlag_){
        std::cout << "------------读取到的环境与模型信息如下------------" << std::endl;
        std::cout << modelInfo << std::endl;
        std::cout << "如果仿真遇到问题,请检查上述信息是否正确,物理仿真进行中..." << std::endl;
    }


    if(modelParam_.keyFrameCount == 0 ){ 
        RCLCPP_WARN(this->get_logger(), "未发现keyframe,请检查模型");
    }

    // 传感器错误检查
    if(modelParam_.jointPosHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节位置传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointVelHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointTorHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节力矩传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuQuatHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现四元数传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuGyroHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现角速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuAccHeadId==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现线加速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.readErrorFlag){
        RCLCPP_ERROR(this->get_logger(), "传感器参数缺失,将不会进行ROS通信");
    }


}

/**
 * @brief 读取加载的模型信息并保存到类成员中
 * 
 */
void SimulateBridge::ReadModel(){

    // 读取加载的模型名称
    modelParam_.modelName = mj_model_->names;
    // 读取加载的模型时间步长
    modelParam_.timeStep = mj_model_->opt.timestep;
    // 加载关键帧个数
    modelParam_.keyFrameCount = mj_model_->nkey;
    if(modelParam_.keyFrameCount > 0 ){ // 应用第一个关键帧作为初始位置
        mj_resetDataKeyframe(mj_model_, mj_data_, 0);
    }

    // 遍历所有joint,读取参数
    for(int i = 0; i<mj_model_->njnt; i++){
        if(mj_model_->jnt_type[i] == mjJNT_FREE) continue;// 注意:这里删去了free joint
        modelParam_.jointMujocoIds.push_back(i);
        modelParam_.jointName.push_back(mj_id2name(mj_model_,mjOBJ_JOINT,i)); // 关节名称
        modelParam_.jointPosRange.push_back(std::make_pair(mj_model_->jnt_range[2*i],mj_model_->jnt_range[2*i+1])); // 关节位置限幅
        modelParam_.jointTorqueRange.push_back(std::make_pair(mj_model_->jnt_actfrcrange[2*i],mj_model_->jnt_actfrcrange[2*i+1])); // 关节速度限幅
        int joint_dofadr = mj_model_->jnt_dofadr[i];
        modelParam_.jointFri.push_back(mj_model_->dof_frictionloss[joint_dofadr]); // 关节摩擦系数
        modelParam_.jointDamp.push_back(mj_model_->dof_damping[joint_dofadr]); // 关节阻尼
    }

    // 遍历所有link,读取参数
    for(int i = 0; i<mj_model_->nbody; i++){
        if(std::string(mj_id2name(mj_model_,mjOBJ_BODY,i)) == "world")  continue;// 忽略world link
        modelParam_.linkNames.push_back(mj_id2name(mj_model_,mjOBJ_BODY,i));
        modelParam_.linkMass.push_back(mj_model_->body_mass[i]);
    }

    // 遍历所有sensor,读取参数并建立映射
    const size_t local_joint_count = modelParam_.jointName.size();
    modelParam_.jointPosSensorIdx.assign(local_joint_count, -1);
    modelParam_.jointVelSensorIdx.assign(local_joint_count, -1);
    modelParam_.jointEffortSensorIdx.assign(local_joint_count, -1);
    modelParam_.jointPosDataAdr.assign(local_joint_count, -1);
    modelParam_.jointVelDataAdr.assign(local_joint_count, -1);
    modelParam_.jointEffortDataAdr.assign(local_joint_count, -1);
    modelParam_.jointEffortDataScale.assign(local_joint_count, 0.0);
    modelParam_.jointToActuatorIdx.assign(local_joint_count, -1);

    // 首先建立关节名到索引的临时映射，加速查找
    std::map<std::string, int> jointNameToIdx;
    for(size_t i=0; i<modelParam_.jointName.size(); i++) {
        jointNameToIdx[modelParam_.jointName[i]] = i;
    }

    // 建立执行器映射
    for(int i=0; i<mj_model_->nu; i++) {
        const int joint_id = mj_model_->actuator_trnid[2*i];
        if(joint_id >= 0 && joint_id < mj_model_->njnt) {
            const char * joint_name = mj_id2name(mj_model_, mjOBJ_JOINT, joint_id);
            if(joint_name != nullptr && jointNameToIdx.find(joint_name) != jointNameToIdx.end()) {
                modelParam_.jointToActuatorIdx[jointNameToIdx[joint_name]] = i;
            }
        }
    }

    for(size_t i = 0; i < static_cast<size_t>(mj_model_ -> nsensor); i++){
        // 创建临时变量
        std::string tempName;
        std::string tempType;
        std::string tempAttch;
        // 获取sensor名称
        tempName = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        // 根据不同类型的sensor进行不同的处理
        if(mj_model_->sensor_type[i] == mjSENS_JOINTPOS){ // 关节位置
            tempType = "joint pos";
            if(modelParam_.jointPosHeadID == 99999) modelParam_.jointPosHeadID = modelParam_.sensorType.size();
            // 记录索引
            tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
            if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                modelParam_.jointPosSensorIdx[jointNameToIdx[tempAttch]] = i;
                modelParam_.jointPosDataAdr[jointNameToIdx[tempAttch]] = mj_model_->sensor_adr[i];
            }
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_JOINTVEL) { // 关节速度
            tempType = "joint vel";
            if(modelParam_.jointVelHeadID == 99999) modelParam_.jointVelHeadID = modelParam_.sensorType.size();
            // 记录索引
            tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
            if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                modelParam_.jointVelSensorIdx[jointNameToIdx[tempAttch]] = i;
                modelParam_.jointVelDataAdr[jointNameToIdx[tempAttch]] = mj_model_->sensor_adr[i];
            }
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_ACTUATORFRC) { // 关节力矩 (actuator force)
            tempType = "joint torque";
            if(modelParam_.jointTorHeadID == 99999) modelParam_.jointTorHeadID = modelParam_.sensorType.size();
            
            // For actuator force sensors, the object ID refers to the actuator.
            // We need to find which joint this actuator drives.
            int actuator_id = mj_model_->sensor_objid[i];
            int joint_id = mj_model_->actuator_trnid[2*actuator_id]; // Assuming joint joint transmission
            
            // If trntype is mjTRN_JOINT (0) or mjTRN_JOINTINPARENT (1), trnid[0] is the joint id
            if (mj_model_->actuator_trntype[actuator_id] == mjTRN_JOINT) {
                 tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, joint_id);
                 if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                    int j_idx = jointNameToIdx[tempAttch];
                    modelParam_.jointEffortSensorIdx[j_idx] = i;
                    modelParam_.jointEffortDataAdr[j_idx] = mj_model_->sensor_adr[i];
                    modelParam_.jointEffortDataScale[j_idx] = 1.0;
                 }
            } else {
                tempAttch = mj_id2name(mj_model_, mjOBJ_ACTUATOR, actuator_id); // Fallback
            }
            
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
            continue;
        }

        else if(mj_model_->sensor_type[i] == mjSENS_TORQUE || mj_model_->sensor_type[i] == mjSENS_FORCE) { // 力矩/力传感器 (site torque/force)
            tempType = (mj_model_->sensor_type[i] == mjSENS_TORQUE) ? "joint torque (sensor)" : "joint force (sensor)";
            if(modelParam_.jointTorHeadID == 99999) modelParam_.jointTorHeadID = modelParam_.sensorType.size();
            
            int site_id = mj_model_->sensor_objid[i];
            int body_id = mj_model_->site_bodyid[site_id];
            
            // Attempt to find the joint associated with this body
            // We assume the joint connects the parent to this body
            // So we look for joints where body_id is the child
            
            // Iterate over joints to find one that belongs to this body
            int joint_id = -1;
            int jnt_adr = mj_model_->body_jntadr[body_id];
            int jnt_num = mj_model_->body_jntnum[body_id];
            
            if (jnt_num > 0) {
                joint_id = jnt_adr; // Take the first joint
            }

            if (joint_id != -1) {
                tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, joint_id);
                if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                    int j_idx = jointNameToIdx[tempAttch];
                    modelParam_.jointEffortSensorIdx[j_idx] = i; // Store sensor ID for reference
                    
                    // Determine which axis to use
                    // Get joint axis in body frame
                    // Get site orientation in body frame
                    // Project joint axis to site frame
                    
                    mjtNum* jnt_axis = mj_model_->jnt_axis + 3*joint_id;
                    mjtNum* site_quat = mj_model_->site_quat + 4*site_id;
                    
                    // Rotate joint axis by inverse of site quaternion to get it in site frame
                    mjtNum jnt_axis_site[3];
                    mjtNum site_rot[9];
                    mju_quat2Mat(site_rot, site_quat);
                    mju_mulMatTVec(jnt_axis_site, site_rot, jnt_axis, 3, 3); // R^T * v
                    
                    // Find the largest component
                    int best_axis = 0;
                    double max_val = 0;
                    for(int k=0; k<3; k++) {
                        if (std::abs(jnt_axis_site[k]) > std::abs(max_val)) {
                            max_val = jnt_axis_site[k];
                            best_axis = k;
                        }
                    }
                    
                    modelParam_.jointEffortDataAdr[j_idx] = mj_model_->sensor_adr[i] + best_axis;
                    // Check if aligned or anti-aligned
                    modelParam_.jointEffortDataScale[j_idx] = (max_val > 0) ? 1.0 : -1.0;
                }
            } else {
                 tempAttch = mj_id2name(mj_model_, mjOBJ_SITE, site_id);
            }
            
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
            continue;
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEQUAT) { // imu四元数
            tempType = "imu quat";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuQuatHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_w",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_GYRO) { // imu角速度
            tempType = "imu gyro";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuGyroHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_ACCELEROMETER) { // imu线加速度
            tempType = "imu linear acc";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuAccHeadId = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEPOS) { // 实际位置
            tempType = "real position";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realPosHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMELINVEL) { // 实际速度
            tempType = "real velocity";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realVelHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else tempType = "unknown";

        tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
        modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
    }

}

/**
 * @brief 析构函数,释放消息发布资源
 * 
 */
SimulateBridge::~SimulateBridge()
{
}
