#include "agilitycassie/cassie_user_in_t.h"
#include "cassiemujoco/cassiemujoco_cpp.h"
#include <iostream>

#include "cassiemujoco/udp.h"

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

  std::unique_ptr<sim::Renderer> renderer =
      sim::Renderer::create(simulation->getMujocoSim()->model);
  if (!renderer) {
    std::cerr << "Failed to create sim renderer" << std::endl;
    return 1;
  }

  cassie_user_in_t userInput{};

  while (!renderer->shouldClose()) {

    simulation->step(userInput);

    renderer->renderOnce(simulation->getMujocoSim()->model,
                         simulation->getMujocoSim()->data);
  }

  return 0;
}