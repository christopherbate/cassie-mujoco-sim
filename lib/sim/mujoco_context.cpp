#include "cassiemujoco/cassiemujoco_cpp.h"

#include <mujoco/mujoco.h>

#include "mujoco/simulate/glfw_dispatch.h"

using namespace cassie;
using namespace cassie::sim;

sim::MujocoContext::MujocoContext() {}

std::unique_ptr<MujocoContext> sim::MujocoContext::create() {
  // init GLFW
  if (!mujoco::Glfw().glfwInit()) {
    mju_error("could not initialize GLFW");
    return nullptr;
  }

  return std::make_unique<MujocoContext>();
}