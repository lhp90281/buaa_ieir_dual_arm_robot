// Isolated FK-only display: no motor commands, controller clients or simulation steps.
#include <mujoco/mujoco.h>
#include <GLFW/glfw3.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>
#include "lodepng.h"

struct View {
  mjModel* model = nullptr;
  mjData* data = nullptr;
  mjvCamera camera;
  mjvOption options;
  mjvScene* scene = nullptr;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keys;
  double mouse_x = 0, mouse_y = 0;
  bool limit_edit = false;
  bool motion_workflow = false;
  bool editing = false;
  std::string edit_text;
};

static void send_angle(View* view) {
  char* end = nullptr;
  const double angle = std::strtod(view->edit_text.c_str(), &end);
  std_msgs::msg::String msg;
  msg.data = (end == view->edit_text.c_str() || *end || !std::isfinite(angle))
    ? "invalid_angle" : "angle_deg:" + view->edit_text;
  view->keys->publish(msg);
}

static void key(GLFWwindow* window, int code, int, int action, int mods) {
  if (action != GLFW_PRESS && action != GLFW_REPEAT) return;
  auto* view = static_cast<View*>(glfwGetWindowUserPointer(window));
  if (view->limit_edit && view->editing) {
    if (code == GLFW_KEY_BACKSPACE && !view->edit_text.empty()) {
      view->edit_text.pop_back();
      send_angle(view);
    } else if (code == GLFW_KEY_ENTER || code == GLFW_KEY_KP_ENTER) {
      send_angle(view);
      view->editing = false;
    } else if (code == GLFW_KEY_ESCAPE) {
      std_msgs::msg::String msg;
      msg.data = "escape";
      view->keys->publish(msg);
      glfwSetWindowShouldClose(window, true);
    }
    return;
  }
  if (view->limit_edit && code == GLFW_KEY_E && action == GLFW_PRESS) {
    view->editing = true;
    view->edit_text.clear();
    return;
  }
  const bool arrow = code == GLFW_KEY_LEFT || code == GLFW_KEY_RIGHT;
  if (action == GLFW_REPEAT && !(view->limit_edit && arrow)) return;
  std_msgs::msg::String msg;
  if (view->limit_edit && arrow) {
    msg.data = code == GLFW_KEY_LEFT ? "left" : "right";
    if (mods & GLFW_MOD_SHIFT) msg.data += "_fine";
  } else if (code == GLFW_KEY_SPACE) msg.data = "space";
  else if (code == GLFW_KEY_ENTER || code == GLFW_KEY_KP_ENTER) msg.data = "enter";
  else if (code == GLFW_KEY_R) msg.data = "r";
  else if (code == GLFW_KEY_S && view->motion_workflow) msg.data = "s";
  else if (code == GLFW_KEY_N && view->motion_workflow) msg.data = "n";
  else if (code == GLFW_KEY_BACKSPACE) msg.data = "backspace";
  else if (code == GLFW_KEY_ESCAPE) {
    msg.data = "escape";
    glfwSetWindowShouldClose(window, true);
  } else return;
  view->keys->publish(msg);
}

static void mouse(GLFWwindow* window, double x, double y) {
  auto* v = static_cast<View*>(glfwGetWindowUserPointer(window));
  const double dx = x - v->mouse_x, dy = y - v->mouse_y;
  v->mouse_x = x;
  v->mouse_y = y;
  const bool left = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS;
  const bool right = glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS;
  if (!left && !right) return;
  int width, height;
  glfwGetWindowSize(window, &width, &height);
  if (height <= 0) return;
  mjv_moveCamera(v->model, right ? mjMOUSE_MOVE_V : mjMOUSE_ROTATE_V,
                 dx / height, dy / height, v->scene, &v->camera);
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("calibration_viewer");
  const auto prefix = node->declare_parameter<std::string>("topic_prefix", "/joint_direction_preview");
  const auto path = node->declare_parameter<std::string>("model_path",
    ament_index_cpp::get_package_share_directory("dual_arm_support") + "/mjcf/dual_arm_robot.xml");
  const auto screenshot = node->declare_parameter<std::string>("screenshot_path", "");
  const auto frame_limit = node->declare_parameter<int>("exit_after_frames", 0);
  char error[1024] = {};
  View v;
  v.limit_edit = node->declare_parameter<bool>("limit_edit", false);
  v.motion_workflow = node->declare_parameter<bool>("motion_workflow", false);
  v.model = mj_loadXML(path.c_str(), nullptr, error, sizeof(error));
  if (!v.model) {std::cerr << error << '\n'; rclcpp::shutdown(); return 1;}
  v.data = mj_makeData(v.model);
  mjv_defaultCamera(&v.camera);
  mjv_defaultOption(&v.options);
  v.camera.lookat[0] = 0;
  v.camera.lookat[1] = 0;
  v.camera.lookat[2] = 0.6;
  v.camera.distance = 2.6;
  v.camera.azimuth = 120;
  v.camera.elevation = -20;
  v.options.geomgroup[3] = 0;
  // The calibration setup has no grippers. Hide their entire visual subtree.
  for (int g = 0; g < v.model->ngeom; ++g) {
    for (int body = v.model->geom_bodyid[g]; body > 0; body = v.model->body_parentid[body]) {
      const char* name = mj_id2name(v.model, mjOBJ_BODY, body);
      if (name && std::string(name).find("gripper") != std::string::npos) {
        v.model->geom_group[g] = 3;
        break;
      }
    }
  }
  if (!glfwInit()) {mj_deleteData(v.data); mj_deleteModel(v.model); rclcpp::shutdown(); return 2;}
  auto* window = glfwCreateWindow(1100, 820, "Joint calibration - display only", nullptr, nullptr);
  if (!window) {glfwTerminate(); mj_deleteData(v.data); mj_deleteModel(v.model); rclcpp::shutdown(); return 2;}
  glfwMakeContextCurrent(window);
  glfwSwapInterval(1);
  mjvScene scene;
  mjrContext context;
  mjv_defaultScene(&scene);
  mjr_defaultContext(&context);
  mjv_makeScene(v.model, &scene, 2000);
  v.scene = &scene;
  mjr_makeContext(v.model, &context, mjFONTSCALE_150);
  v.keys = node->create_publisher<std_msgs::msg::String>(prefix + "/key", 10);
  glfwSetWindowUserPointer(window, &v);
  glfwSetKeyCallback(window, key);
  glfwSetCharCallback(window, [](GLFWwindow* w, unsigned int code) {
    auto* view = static_cast<View*>(glfwGetWindowUserPointer(w));
    if (!view->editing || view->edit_text.size() >= 16) return;
    if ((code >= '0' && code <= '9') || code == '-' || code == '+' || code == '.') {
      view->edit_text += static_cast<char>(code);
      send_angle(view);
    }
  });
  // Camera operations need the rendered scene; the pointer lives until shutdown.
  glfwSetCursorPosCallback(window, mouse);
  glfwSetScrollCallback(window, [](GLFWwindow* w, double, double y) {
    auto* view = static_cast<View*>(glfwGetWindowUserPointer(w));
    view->camera.distance = std::clamp(view->camera.distance * std::exp(-0.1 * y), 0.2, 10.0);
  });
  std::string status = "Waiting for calibration. Robot is at model zero; no physics.";
  auto last = std::chrono::steady_clock::now();
  std::vector<float> colors(v.model->geom_rgba, v.model->geom_rgba + 4 * v.model->ngeom);
  auto poses = node->create_subscription<sensor_msgs::msg::JointState>(prefix + "/pose", 10,
    [&](sensor_msgs::msg::JointState::ConstSharedPtr msg) {
      if (msg->name.size() != msg->position.size() || msg->name.size() > 1) return;
      for (double q : msg->position)
        if (!std::isfinite(q) || std::abs(q) > ((v.limit_edit || v.motion_workflow) ? 2 * mjPI : 1.01)) return;
      mju_zero(v.data->qpos, v.model->nq);
      std::copy(colors.begin(), colors.end(), v.model->geom_rgba);
      for (size_t i = 0; i < msg->name.size(); ++i) {
        const auto& name = msg->name[i];
        if (name.rfind("left_joint_", 0) != 0 && name.rfind("right_joint_", 0) != 0) continue;
        const int j = mj_name2id(v.model, mjOBJ_JOINT, name.c_str());
        if (j >= 0 && v.model->jnt_type[j] == mjJNT_HINGE) {
          v.data->qpos[v.model->jnt_qposadr[j]] = msg->position[i];
          for (int g = 0; g < v.model->ngeom; ++g) {
            if (v.model->geom_bodyid[g] == v.model->jnt_bodyid[j]) {
              v.model->geom_rgba[4*g] = 1.0f;
              v.model->geom_rgba[4*g+1] = 0.65f;
              v.model->geom_rgba[4*g+2] = 0.1f;
            }
          }
        }
      }
      last = std::chrono::steady_clock::now();
    });
  auto statuses = node->create_subscription<std_msgs::msg::String>(prefix + "/status", 10,
    [&](std_msgs::msg::String::ConstSharedPtr msg) {status = msg->data;});
  mju_zero(v.data->qpos, v.model->nq);
  int frames = 0;
  while (rclcpp::ok() && !glfwWindowShouldClose(window)) {
    rclcpp::spin_some(node);
    mj_forward(v.model, v.data);
    int width, height;
    glfwGetFramebufferSize(window, &width, &height);
    if (width > 0 && height > 0) {
      mjrRect viewport{0, 0, width, height};
      mjv_updateScene(v.model, v.data, &v.options, nullptr, &v.camera, mjCAT_ALL, &scene);
      mjrRect model_viewport = viewport;
      if ((v.limit_edit || v.motion_workflow) && height > 400) {
        // Reserve space for reference values and editing without hiding joints.
        mjr_rectangle(viewport, 0.12f, 0.18f, 0.23f, 1.0f);
        model_viewport.bottom = 70;
        model_viewport.height = height - 350;
      }
      mjr_render(model_viewport, &scene, &context);
      const bool stale = std::chrono::steady_clock::now() - last > std::chrono::seconds(1);
      mjr_overlay(mjFONT_NORMAL, mjGRID_TOPLEFT, viewport,
        (status + (stale ? "\nPREVIEW STREAM IDLE" : "")).c_str(), nullptr, &context);
      if (v.editing)
        mjr_overlay(mjFONT_NORMAL, mjGRID_BOTTOMLEFT, viewport,
          ("Reference angle (deg): " + v.edit_text + "_\nENTER ends editing; ENTER again confirms.").c_str(),
          nullptr, &context);
      ++frames;
      if (!screenshot.empty() && frames == (frame_limit > 0 ? frame_limit : 30)) {
        std::vector<unsigned char> rgb(width * height * 3), flipped(rgb.size());
        mjr_readPixels(rgb.data(), nullptr, viewport, &context);
        for (int y = 0; y < height; ++y)
          std::copy_n(rgb.data() + (height - y - 1) * width * 3, width * 3,
                      flipped.data() + y * width * 3);
        const auto code = lodepng::encode(screenshot, flipped, width, height, LCT_RGB);
        if (code) std::cerr << "Screenshot failed: " << lodepng_error_text(code) << '\n';
      }
    }
    glfwSwapBuffers(window);
    glfwPollEvents();
    if (frame_limit > 0 && frames >= frame_limit) break;
  }
  mjr_freeContext(&context);
  mjv_freeScene(&scene);
  glfwDestroyWindow(window);
  glfwTerminate();
  mj_deleteData(v.data);
  mj_deleteModel(v.model);
  rclcpp::shutdown();
  return 0;
}
