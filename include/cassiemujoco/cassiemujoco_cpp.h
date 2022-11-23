#ifndef CASSIEMUJOCO_CASSIEMUJOCO_CPP
#define CASSIEMUJOCO_CASSIEMUJOCO_CPP

#include <GLFW/glfw3.h>
#include <functional>
#include <memory>

#include <mujoco/mujoco.h>

#include "agilitycassie/cassie_core_sim.h"
#include "agilitycassie/cassie_out_t.h"
#include "agilitycassie/pd_input.h"
#include "agilitycassie/state_output.h"

namespace cassie::sim {

template <typename T>
using deleted_unique_ptr = std::unique_ptr<T, std::function<void(T *)>>;

/// Represents a global program context. This should only be created once by the
/// client per process.
class MujocoContext {
public:
  MujocoContext();

  static std::unique_ptr<MujocoContext> create();
};

struct RuntimeModelInfo {
  int left_foot_body_id;
  int right_foot_body_id;
  int left_heel_id;
  int right_heel_id;
  int left_toe_id;
  int right_toe_id;
};

/// Bundles the data structures required for a mujoco simulation.
struct MujocoSim {
  mjModel *model = nullptr;
  mjData *data = nullptr;
  RuntimeModelInfo runtimeInfo;
};

/// Create a new mujoco model and data from an XML model description.
deleted_unique_ptr<MujocoSim>
getMujocoSimFromXmlPath(const std::string &xml_path);

/// Copy a mujoco sim model and data.
deleted_unique_ptr<MujocoSim> copyMujocoSim(const MujocoSim &other);

// *******************************************
// Constants
// *******************************************

constexpr int64_t DRIVE_FILTER_NB = 9;
constexpr int64_t JOINT_FILTER_NB = 4;
constexpr int64_t JOINT_FILTER_NA = 3;
constexpr int64_t NUM_DRIVES = 10;
constexpr int64_t NUM_JOINTS = 6;
constexpr int64_t TORQUE_DELAY_CYCLES = 6;

// *******************************************
// Sensors Filter
// *******************************************

struct drive_filter_t {
  std::array<double, DRIVE_FILTER_NB> x;
};

struct joint_filter_t {
  std::array<double, JOINT_FILTER_NB> x;
  std::array<double, JOINT_FILTER_NA> y;
};

/// Encapsulates the cassie robot simulation state.
struct CassieState {
  deleted_unique_ptr<state_output_t> estimator;
  deleted_unique_ptr<pd_input_t> pd;
  cassie_out_t cassie_out;
  std::array<drive_filter_t, NUM_DRIVES> drive_filter;
  std::array<joint_filter_t, NUM_JOINTS> joint_filters;
  std::array<std::array<double, TORQUE_DELAY_CYCLES>, NUM_DRIVES> torque_delay;
};

/// Base representation for a simulation. It contains Mujoco "model" and
/// "data" handles. The simulation can be copied.
class Simulation {
public:
  Simulation(const std::string &xml_path);

  Simulation(const Simulation &other);

  virtual ~Simulation();

  /// Initialize a simulation
  static bool initializeSimulation(Simulation &sim);

  /// This method provides a convenient interface for creating and initializing
  /// derived rewrite patterns of the given type `T`.
  template <typename T, typename... Args>
  static std::unique_ptr<T> create(Args &&...args) {
    std::unique_ptr<T> sim = std::make_unique<T>(std::forward<Args>(args)...);
    if (!sim) {
      return nullptr;
    }
    if (!initializeSimulation(*sim)) {
      return nullptr;
    }
    return sim;
  }

  deleted_unique_ptr<MujocoSim> &getMujocoSim() { return sim; }
  const deleted_unique_ptr<MujocoSim> &getMujocoSim() const { return sim; }

  deleted_unique_ptr<cassie_core_sim_t> &getCore() { return core; }
  const deleted_unique_ptr<cassie_core_sim_t> &getCore() const { return core; }

public:
  /// Step the simulation.
  virtual cassie_out_t step(const cassie_user_in_t &userControlInputs) {
    return cassie_out_t{};
  };

protected:
  deleted_unique_ptr<MujocoSim> sim;
  deleted_unique_ptr<cassie_core_sim_t> core;
  CassieState cassieState;
};
/// The simulation with the default step type.
class DefaultSimulation : public Simulation {
public:
  using Simulation::Simulation;

  cassie_out_t step(const cassie_user_in_t &userControlInputs) override;
};

/// Encapsulates the GLFW data structures used by the renderer.
struct GlfwInfo {
  GLFWwindow *window = nullptr;
  GLFWvidmode vidMode{};

  GlfwInfo &setWindow(GLFWwindow *w) {
    this->window = w;
    return *this;
  }
  GlfwInfo &setVideoMode(GLFWvidmode mode) {
    this->vidMode = mode;
    return *this;
  }
};

/// Encapsulates the Mujoco user interface.
struct MujocoInterface {
  mjuiState uiState{};
  mjUI ui0;
  mjUI ui1;
  mjvFigure groundReactionForceFigure;
};

/// Bundles the data structures required for a mujoco visualization
struct MujocoVis {
  mjvCamera camera;
  mjvOption option;
  mjvScene scene;
  mjvPerturb perturb;
  mjrContext context;
  int fontScale;
  // Must retain a reference to the model.
  mjModel *model;
};

struct MouseState {
  bool buttonLeftDown = false;
  bool buttonMiddleDown = false;
  bool buttonRightDown = false;
  std::array<float, 2> pos;
};

struct InteractionState {
  MouseState mouse;
};

// *******************************************
// Renderer
// *******************************************

/// Class that represents the GLFW-based renderer.
class Renderer {
public:
  Renderer() = default;
  Renderer(const Renderer &) = delete;

  ~Renderer();

  static std::unique_ptr<Renderer> create(mjModel *model);

  bool shouldClose() const;

  void renderOnce(mjModel *model, mjData *data);

  InteractionState &getInteractionState() { return state; }

  deleted_unique_ptr<MujocoVis> &getMujocoVis() { return info; }

protected:
  GlfwInfo glfwData;
  deleted_unique_ptr<MujocoVis> info;
  MujocoInterface ui;
  InteractionState state;
};

inline long long getMicroseconds() {
  struct timespec now;
  clock_gettime(CLOCK_MONOTONIC, &now);
  return now.tv_sec * 1000000 + now.tv_nsec / 1000;
}

class CassieSim {
public:
private:
};

class CassieVis {
public:
  CassieVis(CassieSim &sim, bool offscreen);

private:
};

} // namespace cassie::sim

#endif // CASSIEMUJOCO_CASSIEMUJOCO_CPP
