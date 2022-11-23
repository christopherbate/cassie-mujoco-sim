#include "agilitycassie/cassie_core_sim.h"
#include "agilitycassie/cassie_out_t.h"
#include "agilitycassie/pd_input.h"
#include "agilitycassie/state_output.h"
#include "cassiemujoco/cassiemujoco_cpp.h"

#include <cassert>
#include <iostream>

#include "cassie.h"
#include <mujoco/mujoco.h>

using namespace cassie;
using namespace cassie::sim;

#define ID_NAME_LOOKUP(model, idvar, objtype, name)                            \
  do {                                                                         \
    idvar = mj_name2id(model, objtype, name);                                  \
    if (-1 == idvar) {                                                         \
      fprintf(stderr, "Could not find body named " name "\n");                 \
      return false;                                                            \
    }                                                                          \
  } while (0)

/// The number of position variables in the model. This should be slightly
/// larger than the number of velocity variables (+1 for each quaternion).
constexpr unsigned kQPosSize = 35;

/// Positions for "reset" config where Cassie is standing on the ground.
static constexpr std::array<double, kQPosSize> qpos_init = {
    0,       0, 1.01,   1,      0,       0,        0,
    0.0045,  0, 0.4973, 0.9785, -0.0164, 0.01787,  -0.2049,
    -1.1997, 0, 1.4267, 0,      -1.5244, 1.5244,   -1.5968,
    -0.0045, 0, 0.4973, 0.9786, 0.00386, -0.01524, -0.2051,
    -1.1997, 0, 1.4267, 0,      -1.5244, 1.5244,   -1.5968};

static bool initializeModel(mjModel *model, RuntimeModelInfo &info) {
  // Populate sensor object Ids.
  // TODO: what are these numbers?
  std::vector<int> sens_objid = {0, 1, 2, 3,  4,  9,  10, 14, 5, 6,
                                 7, 8, 9, 20, 21, 25, 0,  0,  0, 0};
  for (unsigned i = 0; i < sens_objid.size(); i++)
    model->sensor_objid[i] = sens_objid[i];

  // Look up relevant IDs based on names
  ID_NAME_LOOKUP(model, info.left_foot_body_id, mjOBJ_BODY, "left-foot");
  ID_NAME_LOOKUP(model, info.right_foot_body_id, mjOBJ_BODY, "right-foot");
  ID_NAME_LOOKUP(model, info.left_heel_id, mjOBJ_SITE, "left-heel");
  ID_NAME_LOOKUP(model, info.left_toe_id, mjOBJ_SITE, "left-toe");
  ID_NAME_LOOKUP(model, info.right_heel_id, mjOBJ_SITE, "right-heel");
  ID_NAME_LOOKUP(model, info.right_toe_id, mjOBJ_SITE, "right-toe");
  return true;
}

/// Reset the "constant" data of the model/data. We also call `mj_forward` once
/// to propogate the values through the simulation.
static void resetCassieSim(mjModel *model, mjData *data, CassieState &c) {
  c.cassie_out = getCassieOutDefault();

  mj_setConst(model, data);

  double qvel_zero[model->nv];
  double qacc_zero[model->nv];
  for (int i = 0; i < model->nv; i++) {
    qvel_zero[i] = 0.0f;
    qacc_zero[i] = 0.0f;
  }
  mju_copy(data->qpos, qpos_init.data(), qpos_init.size());
  mju_copy(data->qvel, qvel_zero, model->nv);
  mju_copy(data->qacc, qacc_zero, model->nv);
  data->time = 0.0;
  mj_forward(model, data);
}

deleted_unique_ptr<MujocoSim>
sim::getMujocoSimFromXmlPath(const std::string &xml_path) {
  MujocoSim *sim = new MujocoSim{};
  std::string error;
  error.resize(1024);
  sim->model =
      mj_loadXML(xml_path.c_str(), nullptr, error.data(), error.size());
  if (sim->model == nullptr) {
    mju_error(error.c_str());
    return nullptr;
  }

  if (!initializeModel(sim->model, sim->runtimeInfo))
    return nullptr;

  sim->data = mj_makeData(sim->model);
  if (sim->data == nullptr) {
    mju_error("could not make sim data");
    return nullptr;
  }

  return deleted_unique_ptr<MujocoSim>(sim, [](MujocoSim *s) {
    if (s->model)
      mj_deleteModel(s->model);
    if (s->data)
      mj_deleteData(s->data);
  });
}

/// Copy a mujoco sim model and data.
deleted_unique_ptr<MujocoSim> sim::copyMujocoSim(const MujocoSim &other) {
  deleted_unique_ptr<MujocoSim> sim =
      deleted_unique_ptr<MujocoSim>(new MujocoSim{}, [](MujocoSim *s) {
        if (s->model)
          mj_deleteModel(s->model);
        if (s->data)
          mj_deleteData(s->data);
      });

  sim->model = mj_copyModel(nullptr, other.model);
  sim->data = mj_copyData(nullptr, sim->model, other.data);

  return sim;
}

Simulation::~Simulation() {}

sim::Simulation::Simulation(const std::string &xml_path) {
  sim = getMujocoSimFromXmlPath(xml_path);

  // Reset internal mujoco variables such as model position.
  resetCassieSim(sim->model, sim->data, cassieState);

  // Allocate a new core simulation and copy.
  cassie_core_sim_t *core_sim = cassie_core_sim_alloc();
  cassie_core_sim_setup(core_sim);
  core = deleted_unique_ptr<cassie_core_sim_t>(
      core_sim, [](cassie_core_sim_t *c) { cassie_core_sim_free(c); });

  cassieState.estimator = deleted_unique_ptr<state_output_t>(
      state_output_alloc(), [](state_output_t *o) { state_output_free(o); });
  state_output_setup(cassieState.estimator.get());
  cassieState.pd = deleted_unique_ptr<pd_input_t>(
      pd_input_alloc(), [](pd_input_t *p) { pd_input_free(p); });
  pd_input_setup(cassieState.pd.get());
}

sim::Simulation::Simulation(const Simulation &other) {
  sim = copyMujocoSim(*other.getMujocoSim());
  assert(sim != nullptr);

  // Allocate a new core simulation and copy.
  cassie_core_sim_t *core_sim = cassie_core_sim_alloc();
  cassie_core_sim_copy(core_sim, &*other.getCore());
  core = deleted_unique_ptr<cassie_core_sim_t>(
      core_sim, [](cassie_core_sim_t *c) { cassie_core_sim_free(c); });

  cassieState.estimator = deleted_unique_ptr<state_output_t>(
      state_output_alloc(), [](state_output_t *o) { state_output_free(o); });
  state_output_setup(cassieState.estimator.get());
  cassieState.pd = deleted_unique_ptr<pd_input_t>(
      pd_input_alloc(), [](pd_input_t *p) { pd_input_free(p); });
  pd_input_setup(cassieState.pd.get());
}

bool sim::Simulation::initializeSimulation(Simulation &sim) {
  // Check anything that might have failed in the constructor.
  if (!sim.getMujocoSim())
    return false;
  if (!sim.getCore())
    return false;

  resetCassieSim(sim.getMujocoSim()->model, sim.getMujocoSim()->data,
                 sim.cassieState);

  return true;
}

/*******************************************************************************
 * Control Simulation
 ******************************************************************************/

#define DRIVE_LIST                                                             \
  X(leftLeg.hipRollDrive)                                                      \
  X(leftLeg.hipYawDrive)                                                       \
  X(leftLeg.hipPitchDrive)                                                     \
  X(leftLeg.kneeDrive)                                                         \
  X(leftLeg.footDrive)                                                         \
  X(rightLeg.hipRollDrive)                                                     \
  X(rightLeg.hipYawDrive)                                                      \
  X(rightLeg.hipPitchDrive)                                                    \
  X(rightLeg.kneeDrive)                                                        \
  X(rightLeg.footDrive)

#define JOINT_LIST                                                             \
  X(leftLeg.shinJoint)                                                         \
  X(leftLeg.tarsusJoint)                                                       \
  X(leftLeg.footJoint)                                                         \
  X(rightLeg.shinJoint)                                                        \
  X(rightLeg.tarsusJoint)                                                      \
  X(rightLeg.footJoint)

/// Return the motor torque ofr the given motor, using the torque delay state
/// and the torque command.
static double
getCurrentMotorTorque(const mjModel *m, mjData *d, int i, double u,
                      std::array<double, TORQUE_DELAY_CYCLES> &torque_delay,
                      bool sto) {
  double ratio = m->actuator_gear[6 * i];
  double tmax = m->actuator_ctrlrange[2 * i + 1];
  double w = d->actuator_velocity[i];
  double wmax = m->actuator_user[m->nuser_actuator * i] * 2 * M_PI / 60;

  // Calculate torque limit based on motor speed
  double tlim = 2 * tmax * (1 - fabs(w) / wmax);
  tlim = fmax(fmin(tlim, tmax), 0);

  // Apply STO
  if (sto)
    u = 0;

  // Compute motor-side torque
  double tau = copysign(fmin(fabs(u / ratio), tlim), u);

  // Torque delay line
  d->ctrl[i] = torque_delay[TORQUE_DELAY_CYCLES - 1];
  for (int i = TORQUE_DELAY_CYCLES - 1; i > 0; --i)
    torque_delay[i] = torque_delay[i - 1];
  torque_delay[0] = tau;
  // Return the current value of the output-side torque
  return d->ctrl[i] * ratio;
}

static void setCassieOutMotorTorques(const mjModel *m, mjData *d,
                                     const cassie_in_t *cassie_in,
                                     CassieState *state) {
  // STO
  bool sto = state->cassie_out.pelvis.radio.channel[8] < 1;

  // Get the ordered list of torque command inputs from the `cassie_in_t`
  // struct.
  std::array<double, NUM_DRIVES> torqueCommandInputs = {
#define X(drive) cassie_in->drive.torque,
      DRIVE_LIST
#undef X
  };

  // Create an ordered list of output references.
  std::array<elmo_out_t *, NUM_DRIVES> outputs = {
#define X(drive) &state->cassie_out.drive,
      DRIVE_LIST
#undef X
  };

  // Get the motor torque according to the torque command input, the torque
  // delay, and limitations based on current motor velocity, etc.
  for (int i = 0; i < NUM_DRIVES; ++i)
    outputs[i]->torque = getCurrentMotorTorque(m, d, i, torqueCommandInputs[i],
                                               state->torque_delay[i], sto);
}

/// Modifies `c->cassie_out` information to include the joint and drive encoder
/// information.
static void setCassieSensorData(mjModel *model, mjData *data, CassieState *c) {
  // Ordered list of drive_out_t addresses
  elmo_out_t *drives[NUM_DRIVES] = {
#define X(drive) &c->cassie_out.drive,
      DRIVE_LIST
#undef X
  };

  // Ordered list of cassie_joint_out_t addresses
  cassie_joint_out_t *joints[NUM_JOINTS] = {
#define X(joint) &c->cassie_out.joint,
      JOINT_LIST
#undef X
  };

  // Sensor ID for each encoder
  static const int drive_sensor_ids[10] = {0, 1, 2, 3, 4, 8, 9, 10, 11, 12};
  static const int joint_sensor_ids[6] = {5, 6, 7, 13, 14, 15};

  // Populate drive and joint encoders with filtered data based on the mujoco
  // simulation.
  for (int i = 0; i < NUM_DRIVES; ++i)
    set_drive_encoder(model, drives[i], data->sensordata, &c->drive_filter[i],
                      drive_sensor_ids[i]);
  for (int i = 0; i < NUM_JOINTS; ++i)
    set_joint_encoder(model, joints[i], data->sensordata, &c->joint_filters[i],
                      joint_sensor_ids[i]);

  // Copy IMU sensor data from mujoco to the `cassie_out` struct.
  mju_copy(c->cassie_out.pelvis.vectorNav.orientation, &data->sensordata[16],
           4);
  mju_copy(c->cassie_out.pelvis.vectorNav.angularVelocity,
           &data->sensordata[20], 3);
  mju_copy(c->cassie_out.pelvis.vectorNav.linearAcceleration,
           &data->sensordata[23], 3);
  mju_copy(c->cassie_out.pelvis.vectorNav.magneticField, &data->sensordata[26],
           3);
}

static cassie_out_t simStepEthercat(MujocoSim &sim,
                                    const cassie_in_t *controlInputs,
                                    CassieState *state) {
  // This will modify cassie `state` drive torques in `cassie_out` according to
  // the actuator model. `state->cassie_out` is modified, which is later used to
  // directly populate Mujoco control inputs.
  setCassieOutMotorTorques(sim.model, sim.data, controlInputs, state);

  // Get measurement data before control is actually stepped.
  setCassieSensorData(sim.model, sim.data, state);

  // Step the simulation forward.
  const int numMujocoSteps = round(5e-4 / sim.model->opt.timestep);
  for (unsigned i = 0; i < numMujocoSteps; i++) {
    mj_step1(sim.model, sim.data);
    mj_step2(sim.model, sim.data);
  }
  return state->cassie_out;
}

cassie_out_t
sim::DefaultSimulation::step(const cassie_user_in_t &userControlInputs) {
  // Step the core simulation, which translates the (simplified?) user control
  // inputs into some other structs. The user is providing 10 torque commands,
  // and this result breaks those commands out into more detailed struct in
  // `cassieControlInputs`.
  cassie_in_t cassieControlInputs;
  cassie_core_sim_step(core.get(), &userControlInputs, &cassieState.cassie_out,
                       &cassieControlInputs);

  // Run the low-level simulator, which emulates the delay between controller
  // and motor on the actual robot.
  return simStepEthercat(*sim, &cassieControlInputs, &cassieState);
}