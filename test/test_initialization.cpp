#include "agilitycassie/cassie_user_in_t.h"
#include "cassiemujoco/cassiemujoco_cpp.h"
#include <iostream>

using namespace cassie;

int main(int argc, char **argv) {
  std::unique_ptr<sim::MujocoContext> context = sim::MujocoContext::create();
  if (!context) {
    std::cerr << "Failed to create MujocoContext" << std::endl;
    return 1;
  }

  // Create a new simulation.
  auto simulation = sim::DefaultSimulation::create<sim::DefaultSimulation>(
      "./model/cassie.xml");
  if (!simulation) {
    std::cerr << "Failed to create simulation" << std::endl;
    return 1;
  }

  // Simulation is copyable.
  cassie_user_in_t in{};
  sim::DefaultSimulation copy(*simulation);
  simulation->step(in);
  copy.step(in);

  return 0;
}