#include "cassiemujoco/cassiemujoco_cpp.h"
#include "mujoco/simulate/glfw_dispatch.h"
#include <GLFW/glfw3.h>
#include <iostream>
#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjui.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>
#include <string_view>

using namespace cassie;
using namespace cassie::sim;

using mujoco::Glfw;

namespace {} // namespace

static void onMouseMove(GLFWwindow *w, double xpos, double ypos) {
  auto *v = reinterpret_cast<Renderer *>(glfwGetWindowUserPointer(w));
  MouseState &m = v->getInteractionState().mouse;
  if (!m.buttonLeftDown && !m.buttonMiddleDown && !m.buttonRightDown)
    return;

  // compute mouse displacement, save
  double dx = xpos - m.pos[0];
  double dy = ypos - m.pos[1];
  m.pos =
      std::array<float, 2>{static_cast<float>(xpos), static_cast<float>(ypos)};

  std::array<int, 2> windowSize;
  Glfw().glfwGetWindowSize(w, &windowSize[0], &windowSize[1]);

  int mod_shift = Glfw().glfwGetKey(w, GLFW_KEY_LEFT_SHIFT) ||
                  Glfw().glfwGetKey(w, GLFW_KEY_RIGHT_SHIFT);

  // determine action based on mouse button
  int action = mjMOUSE_ZOOM;
  if (m.buttonRightDown) {
    action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
  } else if (m.buttonLeftDown) {
    action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
  }

  // move perturb or camera
  mjtNum xchange = dx / windowSize[0];
  mjtNum ychange = dy / windowSize[1];
  mjv_moveCamera(v->getMujocoVis()->model, action, xchange, ychange,
                 &v->getMujocoVis()->scene, &v->getMujocoVis()->camera);
}

static void onMouseButton(GLFWwindow *w, int button, int act, int mods) {
  auto *v = reinterpret_cast<Renderer *>(glfwGetWindowUserPointer(w));
  MouseState &m = v->getInteractionState().mouse;
  m.buttonLeftDown = Glfw().glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_LEFT);
  m.buttonRightDown = Glfw().glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_RIGHT);
  m.buttonMiddleDown = Glfw().glfwGetMouseButton(w, GLFW_MOUSE_BUTTON_MIDDLE);
}

static mjvFigure getGroundReactionForceFigure() {
  mjvFigure f;
  mjv_defaultFigure(&f);
  f.figurergba[3] = 0.5f;

  // Set flags
  // f.flg_extend = 1;
  // f.flg_barplot = 1;
  // std::string_view title = "Ground Reaction Forces";
  // std::copy(title.begin(), title.end(), std::begin(f.title));

  // y-tick number format
  // std::string_view yFormat = "%.2f";
  // std::copy(yFormat.begin(), yFormat.end(), std::begin(f.yformat));
  // grid size
  f.gridsize[0] = 5;
  f.gridsize[1] = 5;
  // minimum range
  f.range[0][0] = -200;
  f.range[0][1] = 0;
  f.range[1][0] = 0;
  f.range[1][1] = 20;

  // legends
  // std::array<std::string_view, 2> legend = {"left foot", "right foot"};
  // for (unsigned i = 0; i < legend.size(); i++)
  //   std::copy(legend[i].begin(), legend[i].end(), std::begin(f.linename[i]));

  // init x axis (don't show yet)
  for (int n = 0; n < 2; n++) {
    for (int i = 0; i < mjMAXLINEPNT; i++) {
      f.linedata[n][2 * i] = (float)-i;
    }
  }
  // int min_range[2][2] = { {0, 1}, {-1, 1} };
  // memcpy(min_range, v->figsensor.range, sizeof(min_range));
  return f;
}

static mjUI initializeMjUI(int id) {
  mjUI ui;
  ui.spacing = mjui_themeSpacing(0);
  ui.color = mjui_themeColor(0);
  ui.predicate = nullptr;
  ui.rectid = id + 1;
  ui.auxid = id;
  return ui;
}

/// Get a good default font scale.
static int getFontScale(GLFWwindow *window) {
  int width_win, width_buf, height;
  Glfw().glfwGetWindowSize(window, &width_win, &height);
  Glfw().glfwGetFramebufferSize(window, &width_buf, &height);
  double b2w = (double)width_buf / (double)width_win;

  // compute PPI
  int width_MM, height_MM;
  Glfw().glfwGetMonitorPhysicalSize(Glfw().glfwGetPrimaryMonitor(), &width_MM,
                                    &height_MM);
  int width_vmode =
      Glfw().glfwGetVideoMode(Glfw().glfwGetPrimaryMonitor())->width;
  double PPI = 25.4 * b2w * (double)width_vmode / (double)width_MM;

  // estimate font scaling, guard against unrealistic PPI
  int fs;
  if (width_buf > width_win) {
    fs = mju_round(b2w * 100);
  } else if (PPI > 50 && PPI < 350) {
    fs = mju_round(PPI);
  } else {
    fs = 150;
  }
  fs = mju_round(fs * 0.02) * 50;
  fs = mjMIN(300, mjMAX(100, fs));
  return fs;
}

/// Initialize mujoco rendering/viewing data structures to default states.
static deleted_unique_ptr<MujocoVis> getMujocoInfo(GLFWwindow *window,
                                                   mjModel *model) {
  MujocoVis *info = new MujocoVis;
  mjv_defaultCamera(&info->camera);
  mjv_defaultOption(&info->option);
  mjv_defaultScene(&info->scene);
  mjv_defaultPerturb(&info->perturb);

  info->fontScale = getFontScale(window);

  // Initialize the Mujoco OpenGL context to empty.
  mjr_defaultContext(&info->context);

  // Create the scene and context.
  mjv_makeScene(model, &info->scene, 1000);
  mjr_makeContext(model, &info->context, mjFONTSCALE_100);
  info->model = model;
  return deleted_unique_ptr<MujocoVis>(info, [](MujocoVis *info) {
    mjr_freeContext(&info->context);
    mjv_freeScene(&info->scene);
  });
}

static MujocoInterface getMujocoInterface() {
  MujocoInterface ui;
  // Initialize the Mujoco UIs.
  ui.ui0 = initializeMjUI(0);
  ui.ui1 = initializeMjUI(1);
  ui.groundReactionForceFigure = getGroundReactionForceFigure();
  return ui;
}

/// Create a renderer and return the initialized pointer. This will setup the
/// GFLW window and initial Mujoco rendering objects.
std::unique_ptr<Renderer> sim::Renderer::create(mjModel *model) {
  // Set multi-sampling.
  Glfw().glfwWindowHint(GLFW_SAMPLES, 4);
  Glfw().glfwWindowHint(GLFW_VISIBLE, 1);

  auto glfwData = GlfwInfo().setVideoMode(
      *Glfw().glfwGetVideoMode(Glfw().glfwGetPrimaryMonitor()));

  // Create the GLFW window.
  GLFWwindow *window = Glfw().glfwCreateWindow(
      (2 * glfwData.vidMode.width) / 3, (2 * glfwData.vidMode.height) / 3,
      "CassieMujocoSim", nullptr, nullptr);
  if (!window) {
    Glfw().glfwTerminate();
    mju_error("could not create window");
    return nullptr;
  }
  glfwData.setWindow(window);
  std::cout << "Created window." << std::endl;

  // make context current, set v-sync
  Glfw().glfwMakeContextCurrent(glfwData.window);
  Glfw().glfwSwapInterval(1);

  // Create the renderer.
  auto renderer = std::make_unique<Renderer>();
  renderer->glfwData = glfwData;
  renderer->info = getMujocoInfo(glfwData.window, model);
  renderer->ui = getMujocoInterface();

  // Set GLFW window callbacks.
  Glfw().glfwSetWindowUserPointer(glfwData.window, renderer.get());
  Glfw().glfwSetCursorPosCallback(glfwData.window, onMouseMove);
  Glfw().glfwSetMouseButtonCallback(glfwData.window, onMouseButton);
  // Glfw().glfwSetScrollCallback(glfwData.window, scroll);
  // Glfw().glfwSetKeyCallback(glfwData.window, key_callback);

  return renderer;
}

sim::Renderer::~Renderer() { Glfw().glfwTerminate(); }

static void fillRect(mjrRect rect) { mjr_rectangle(rect, 0.2f, 0.3f, 0.4f, 1); }

static mjrRect getSmallRect(mjrRect rect) {
  return mjrRect{rect.left + rect.width / 2, rect.bottom + rect.height / 3,
                 rect.width / 2, rect.height / 2};
}

static void renderGroundReactionForceFigure(mjrRect figureRect,
                                            deleted_unique_ptr<MujocoVis> &info,
                                            MujocoInterface &ui) {
  mjr_figure(figureRect, &ui.groundReactionForceFigure, &info->context);
}

/// Perform the rendering step.
static void defaultRenderingStep(mjrRect viewport, mjModel *model, mjData *data,
                                 deleted_unique_ptr<MujocoVis> &info,
                                 MujocoInterface &ui) {
  // Update the scene and render.
  mjv_updateScene(model, data, &info->option, &info->perturb, &info->camera,
                  mjCAT_ALL, &info->scene);
  mjr_render(viewport, &info->scene, &info->context);

  // Render the UIs.
  mjui_render(&ui.ui0, &ui.uiState, &info->context);
  mjui_render(&ui.ui1, &ui.uiState, &info->context);

  // Render GRF figure.
  // renderGroundReactionForceFigure(getSmallRect(viewport), info, ui);
}

/// Render in a loop.
void Renderer::renderOnce(mjModel *model, mjData *data) {
  // while (!Glfw().glfwWindowShouldClose(glfwData.window)) {
  // const std::lock_guard<std::mutex> lock(this->mtx);
  // handle events (calls all callbacks)

  // Get the viewport rectangle.
  mjrRect viewport{0, 0, 0, 0};
  Glfw().glfwGetFramebufferSize(glfwData.window, &viewport.width,
                                &viewport.height);
  // fillRect(viewport);
  mjrRect figuresRect = viewport;

  // Perform rendering activities.
  defaultRenderingStep(viewport, model, data, info, ui);

  Glfw().glfwSwapBuffers(glfwData.window);

  Glfw().glfwPollEvents();
}

bool sim::Renderer::shouldClose() const {
  return Glfw().glfwWindowShouldClose(glfwData.window);
}
