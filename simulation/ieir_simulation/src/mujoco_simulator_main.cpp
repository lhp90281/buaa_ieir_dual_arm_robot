// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <cerrno>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>
#include <new>
#include <string>
#include <thread>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include "rclcpp/rclcpp.hpp"

#include <mujoco/mujoco.h>
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"

#include "ieir_simulation/simulate_bridge.hpp"

#define MUJOCO_PLUGIN_DIR "mujoco_plugin"

extern "C" {
#include <sys/errno.h>
#include <unistd.h>
}

namespace {
namespace mj = ::mujoco;
namespace mju = ::mujoco::sample_util;

// constants
const double syncMisalign = 0.1;        // maximum mis-alignment before re-sync (simulation seconds)
const double simRefreshFraction = 0.7;  // fraction of refresh available for simulation
const int kErrorLength = 1024;          // load error string length

std::string ResolveModelPath(const std::string& path) {
  constexpr const char* kPackagePrefix = "package://";
  if (path.rfind(kPackagePrefix, 0) == 0) {
    const std::string rest = path.substr(std::strlen(kPackagePrefix));
    const auto slash = rest.find('/');
    if (slash == std::string::npos) {
      return path;
    }
    const std::string package_name = rest.substr(0, slash);
    const std::string relative_path = rest.substr(slash + 1);
    return ament_index_cpp::get_package_share_directory(package_name) + "/" + relative_path;
  }
  return path;
}

// model and data
mjModel* m = nullptr;
mjData* d = nullptr;

// 声明全局类指针,此时还没有初始化
SimulateBridge::SharedPtr SimulateBridgeNodePtr = nullptr;


using Seconds = std::chrono::duration<double>;


//------------------------------------------- simulation -------------------------------------------

const char* Diverged(int disableflags, const mjData* d) {
  if (disableflags & mjDSBL_AUTORESET) {
    for (mjtWarning w : {mjWARN_BADQACC, mjWARN_BADQVEL, mjWARN_BADQPOS}) {
      if (d->warning[w].number > 0) {
        return mju_warningText(w, d->warning[w].lastinfo);
      }
    }
  }
  return nullptr;
}

mjModel* LoadModel(const char* file, mj::Simulate& sim) {
  // this copy is needed so that the mju::strlen call below compiles
  char filename[mj::Simulate::kMaxFilenameLength];
  mju::strcpy_arr(filename, file);

  // make sure filename is not empty
  if (!filename[0]) {
    return nullptr;
  }

  // load and compile
  char loadError[kErrorLength] = "";
  mjModel* mnew = 0;
  auto load_start = mj::Simulate::Clock::now();
  if (mju::strlen_arr(filename)>4 &&
      !std::strncmp(filename + mju::strlen_arr(filename) - 4, ".mjb",
                    mju::sizeof_arr(filename) - mju::strlen_arr(filename)+4)) {
    mnew = mj_loadModel(filename, nullptr);
    if (!mnew) {
      mju::strcpy_arr(loadError, "could not load binary model");
    }
  } else {
    mnew = mj_loadXML(filename, nullptr, loadError, kErrorLength);

    // remove trailing newline character from loadError
    if (loadError[0]) {
      int error_length = mju::strlen_arr(loadError);
      if (loadError[error_length-1] == '\n') {
        loadError[error_length-1] = '\0';
      }
    }
  }
  auto load_interval = mj::Simulate::Clock::now() - load_start;
  double load_seconds = Seconds(load_interval).count();

  if (!mnew) {
    std::printf("%s\n", loadError);
    mju::strcpy_arr(sim.load_error, loadError);
    return nullptr;
  }

  // compiler warning: print and pause
  if (loadError[0]) {
    // mj_forward() below will print the warning message
    std::printf("Model compiled, but simulation warning (paused):\n  %s\n", loadError);
    sim.run = 0;
  }

  // if no error and load took more than 1/4 seconds, report load time
  else if (load_seconds > 0.25) {
    mju::sprintf_arr(loadError, "Model loaded in %.2g seconds", load_seconds);
  }

  mju::strcpy_arr(sim.load_error, loadError);

  return mnew;
}

// simulate in background thread (while rendering in main thread)
void PhysicsLoop(mj::Simulate& sim) {
  // cpu-sim syncronization point
  std::chrono::time_point<mj::Simulate::Clock> syncCPU;
  mjtNum syncSim = 0;

  // run until asked to exit
  while (!sim.exitrequest.load()) {
    if (sim.droploadrequest.load()) {
      sim.LoadMessage(sim.dropfilename);
      mjModel* mnew = LoadModel(sim.dropfilename, sim);
      sim.droploadrequest.store(false);

      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.dropfilename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;

        // 给ros2节点指针赋值
        SimulateBridgeNodePtr = nullptr; // 先使资源释放
        SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, sim);
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    if (sim.uiloadrequest.load()) {
      sim.uiloadrequest.fetch_sub(1);
      sim.LoadMessage(sim.filename);
      mjModel* mnew = LoadModel(sim.filename, sim);
      mjData* dnew = nullptr;
      if (mnew) dnew = mj_makeData(mnew);
      if (dnew) {
        sim.Load(mnew, dnew, sim.filename);

        // lock the sim mutex
        const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

        mj_deleteData(d);
        mj_deleteModel(m);

        m = mnew;
        d = dnew;
        // 给ros2节点指针赋值
        SimulateBridgeNodePtr = nullptr;
        SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, sim);
        mj_forward(m, d);

      } else {
        sim.LoadMessageClear();
      }
    }

    // sleep for 1 ms or yield, to let main thread run
    //  yield results in busy wait - which has better timing but kills battery life
    if (sim.run && sim.busywait) {
      std::this_thread::yield();
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim.mtx);

      // run only if model is present
      if (m) {
        if (SimulateBridgeNodePtr) {
          if (sim.ieir_panel_start_edit_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->StartEdit().c_str());
          }
          if (sim.ieir_panel_send_target_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->SendTarget().c_str());
          }
          if (sim.ieir_panel_cancel_edit_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->CancelEdit().c_str());
          }
          if (sim.ieir_panel_joint_mode_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->SwitchToJointPosition().c_str());
          }
          if (sim.ieir_panel_gravity_mode_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->SwitchToGravity().c_str());
          }
          if (sim.ieir_panel_cartesian_mode_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->SwitchToCartesianPosition().c_str());
          }
          if (sim.ieir_panel_home_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->GoHome().c_str());
          }
          if (sim.ieir_panel_teleop_prepare_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->TeleopPrepare().c_str());
          }
          if (sim.ieir_panel_teleop_toggle_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->TeleopToggle().c_str());
          }
          if (sim.ieir_panel_teleop_exit_request.exchange(false)) {
            RCLCPP_INFO(
                SimulateBridgeNodePtr->get_logger(),
                "%s",
                SimulateBridgeNodePtr->TeleopExit().c_str());
          }
          SimulateBridgeNodePtr->ProcessPanelTick();
        }

        if (SimulateBridgeNodePtr && !SimulateBridgeNodePtr->PhysicsEnabled()) {
          sim.run = 0;
        }

        // running
        if (sim.run) {
          bool stepped = false;

          // record cpu time at start of iteration
          const auto startCPU = mj::Simulate::Clock::now();

          // elapsed CPU and simulation time since last sync
          const auto elapsedCPU = startCPU - syncCPU;
          double elapsedSim = d->time - syncSim;

          // requested slow-down factor
          double slowdown = 100 / sim.percentRealTime[sim.real_time_index];

          // misalignment condition: distance from target sim time is bigger than syncmisalign
          bool misaligned =
              std::abs(Seconds(elapsedCPU).count()/slowdown - elapsedSim) > syncMisalign;

          // out-of-sync (for any reason): reset sync times, step
          if (elapsedSim < 0 || elapsedCPU.count() < 0 || syncCPU.time_since_epoch().count() == 0 ||
              misaligned || sim.speed_changed) {
            // re-sync
            syncCPU = startCPU;
            syncSim = d->time;
            sim.speed_changed = false;

            // run single step, let next iteration deal with timing
            mj_step(m, d);

            const char* message = Diverged(m->opt.disableflags, d);
            if (message) {
              sim.run = 0;
              mju::strcpy_arr(sim.load_error, message);
            } else {
              stepped = true;
            }
          }

          // in-sync: step until ahead of cpu
          else {
            bool measured = false;
            mjtNum prevSim = d->time;

            double refreshTime = simRefreshFraction/sim.refresh_rate;

            // step while sim lags behind cpu and within refreshTime
            while (Seconds((d->time - syncSim)*slowdown) < mj::Simulate::Clock::now() - syncCPU &&
                   mj::Simulate::Clock::now() - startCPU < Seconds(refreshTime)) {
              // measure slowdown before first step
              if (!measured && elapsedSim) {
                sim.measured_slowdown =
                    std::chrono::duration<double>(elapsedCPU).count() / elapsedSim;
                measured = true;
              }

              // inject noise
              sim.InjectNoise();

              // call mj_step
              mj_step(m, d);

              const char* message = Diverged(m->opt.disableflags, d);
              if (message) {
                sim.run = 0;
                mju::strcpy_arr(sim.load_error, message);
              } else {
                stepped = true;
              }

              // break if reset
              if (d->time < prevSim) {
                break;
              }
            }
          }

          // save current state to history buffer
          if (stepped) {
            sim.AddToHistory();
          }
        }

        // paused
        else {
          // run mj_forward, to update rendering and joint sliders
          mj_forward(m, d);
          sim.speed_changed = true;
        }
        // 执行ros2回调以执行unPause回调
        rclcpp::spin_some(SimulateBridgeNodePtr);
      } 
    }  // release std::lock_guard<std::mutex>
    
  }
}
}  // namespace

//-------------------------------------- physics_thread --------------------------------------------

void PhysicsThread(mj::Simulate* sim, const char* filename) {
  // request loadmodel if file given (otherwise drag-and-drop)
  if (filename != nullptr) {
    sim->LoadMessage(filename);
    m = LoadModel(filename, *sim);
    if (m) {
      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      d = mj_makeData(m);
    }
    if (d) {
      sim->Load(m, d, filename);

      // lock the sim mutex
      const std::unique_lock<std::recursive_mutex> lock(sim->mtx);

      // 给ros2节点指针赋值
      SimulateBridgeNodePtr = std::make_shared<SimulateBridge>(d, m, *sim);
      mj_forward(m, d);

    } else {
      sim->LoadMessageClear();
    }
  }

  PhysicsLoop(*sim);

  // delete everything we allocated
  mj_deleteData(d);
  mj_deleteModel(m);
}

// 控制回调函数
void PDControllerCallBack(const mjModel* m, mjData* d){
  // 如果没有实例化类,就返回
  if(SimulateBridgeNodePtr == nullptr) return;
  // MuJoCo panel modes are display/target editors only; physics and control
  // stay disabled so the window can mirror or edit real robot joint states.
  if(!SimulateBridgeNodePtr->PhysicsEnabled()) return;
  // 暂停的时候不执行控制
  if(SimulateBridgeNodePtr->GetSimRun() == 0) return;
  // 执行ros2回调获取命令值
  rclcpp::spin_some(SimulateBridgeNodePtr);
  (void)m;
  (void)d;
  SimulateBridgeNodePtr->ApplyMitCommand();

  // 发布当前状态
  SimulateBridgeNodePtr->JointStatePublish();
}

//------------------------------------------ main --------------------------------------------------

// run event loop
int main(int argc, char** argv) {

  // 初始化ros2上下文
  rclcpp::init(argc, argv);
  // 声明一个临时节点输出消息
  auto tempNode = rclcpp::Node::make_shared("mujoco_simulator_node");

  // print version, check compatibility
  RCLCPP_INFO(tempNode->get_logger(),"MuJoCo version %s\n", mj_versionString());
  if (mjVERSION_HEADER!=mj_version()) {
    RCLCPP_ERROR(tempNode->get_logger(),"Headers and library have different versions");
  }
  // 释放临时节点
  tempNode = nullptr;

  mjvCamera cam;
  mjv_defaultCamera(&cam);

  mjvOption opt;
  mjv_defaultOption(&opt);

  mjvPerturb pert;
  mjv_defaultPerturb(&pert);

  // simulate object encapsulates the UI
  auto sim = std::make_unique<mj::Simulate>(
      std::make_unique<mj::GlfwAdapter>(),
      &cam, &opt, &pert, /* is_passive = */ false
  );


  // 读取yaml路径 (支持通过环境变量 IEIR_MUJOCO_CONFIG 切换配置文件)
  const std::string mujoco_pkg_path = ament_index_cpp::get_package_share_directory("ieir_simulation"); // 获取包路径
  const char* config_env = std::getenv("IEIR_MUJOCO_CONFIG");
  const std::string config_filename = config_env ? config_env : "simulate.yaml";
  const std::string yaml_path = mujoco_pkg_path + "/config/" + config_filename; // 拼接yaml路径
  YAML::Node config = YAML::LoadFile(yaml_path)["mujoco_simulator"];

  // 读取mjcf路径
  std::string mjcf_path = config["modelFile"].as<std::string>();
  if (const char* model_env = std::getenv("IEIR_MUJOCO_MODEL_FILE")) {
    mjcf_path = model_env;
  }
  mjcf_path = ResolveModelPath(mjcf_path);
  
  // Check if file exists, if not try to resolve it (simple check)
  if (mjcf_path.empty()) {
      RCLCPP_ERROR(tempNode->get_logger(), "modelFile not specified in simulate.yaml");
      return 1;
  }


  // 设置控制回调函数
  mjcb_control = PDControllerCallBack; 

  // 启动物理仿真线程
  std::thread physicsthreadhandle(&PhysicsThread, sim.get(), mjcf_path.data());


  // start simulation UI loop (blocking call)
  sim->RenderLoop();
  physicsthreadhandle.join();

  // 释放资源
  rclcpp::shutdown();
  pthread_exit(NULL);
  return 0;
}
