#include "cassie.h"

#include <array>

#include "agilitycassie/cassie_in_t.h"
#include "agilitycassie/cassie_out_t.h"

#include "mujoco/mujoco.h"

using namespace cassie;
using namespace cassie::sim;

constexpr std::array<int, DRIVE_FILTER_NB> drive_filter_b = {
    2727, 534, -2658, -795, 72, 110, 19, -6, -3};

constexpr std::array<double, JOINT_FILTER_NB> joint_filter_b = {
    12.348, 12.348, -12.348, -12.348};

constexpr std::array<double, JOINT_FILTER_NA> joint_filter_a = {1.0, -1.7658,
                                                                0.79045};

static void elmo_out_init(elmo_out_t *o, double torqueLimit, double gearRatio) {
  o->statusWord = 0x0637;
  o->dcLinkVoltage = 48;
  o->driveTemperature = 30;
  o->torqueLimit = torqueLimit;
  o->gearRatio = gearRatio;
}

/// Initialize the leg controls to nominal values.
static void cassie_leg_out_init(cassie_leg_out_t *o) {
  o->medullaCounter = 1;
  o->medullaCpuLoad = 94;
  elmo_out_init(&o->hipRollDrive, 140.63, 25);
  elmo_out_init(&o->hipYawDrive, 140.63, 25);
  elmo_out_init(&o->hipPitchDrive, 216.16, 16);
  elmo_out_init(&o->kneeDrive, 216.16, 16);
  elmo_out_init(&o->footDrive, 45.14, 50);
}

/// Initialize the cassie controls to nominal values.
cassie_out_t sim::getCassieOutDefault() {
  // The struct is zero-initialized when created
  cassie_out_t out;
  cassie_out_t *o = &out;

  // Calibrated
  o->isCalibrated = true;

  // Pelvis
  o->pelvis.medullaCounter = 1;
  o->pelvis.medullaCpuLoad = 159;
  o->pelvis.vtmTemperature = 40;

  // Target PC
  o->pelvis.targetPc.etherCatStatus[1] = 8;
  o->pelvis.targetPc.etherCatStatus[4] = 1;
  o->pelvis.targetPc.taskExecutionTime = 2e-4;
  o->pelvis.targetPc.cpuTemperature = 60;

  // Battery
  o->pelvis.battery.dataGood = true;
  o->pelvis.battery.stateOfCharge = 1;
  for (int i = 0; i < 4; ++i)
    o->pelvis.battery.temperature[i] = 30;
  for (int i = 0; i < 12; ++i)
    o->pelvis.battery.voltage[i] = 4.2;

  // Radio
  o->pelvis.radio.radioReceiverSignalGood = true;
  o->pelvis.radio.receiverMedullaSignalGood = true;
  o->pelvis.radio.channel[8] = 1;

  // VectorNav
  o->pelvis.vectorNav.dataGood = true;
  o->pelvis.vectorNav.pressure = 101.325;
  o->pelvis.vectorNav.temperature = 25;

  // Legs
  cassie_leg_out_init(&o->leftLeg);
  cassie_leg_out_init(&o->rightLeg);
  return out;
}

void sim::set_drive_encoder(const mjModel *m, elmo_out_t *drive,
                            const mjtNum *sensordata, drive_filter_t *filter,
                            int isensor) {
  // Position
  // Get digital encoder value
  int bits = m->sensor_user[m->nuser_sensor * isensor];
  int encoder_value = sensordata[isensor] / (2 * M_PI) * (1 << bits);
  double ratio = m->actuator_gear[6 * m->sensor_objid[isensor]];
  double scale = (2 * M_PI) / (1 << bits) / ratio;
  drive->position = encoder_value * scale;

  // Velocity
  // Initialize unfiltered signal array to prevent bad transients
  bool allzero = true;
  for (int i = 0; i < DRIVE_FILTER_NB; ++i)
    allzero &= filter->x[i] == 0;
  if (allzero) {
    // If all filter values are zero, initialize the signal array
    // with the current encoder value
    for (int i = 0; i < DRIVE_FILTER_NB; ++i)
      filter->x[i] = encoder_value;
  }

  // Shift and update unfiltered signal array
  for (int i = DRIVE_FILTER_NB - 1; i > 0; --i)
    filter->x[i] = filter->x[i - 1];
  filter->x[0] = encoder_value;
  // Compute filter value
  int y = 0;
  for (int i = 0; i < DRIVE_FILTER_NB; ++i)
    y += filter->x[i] * drive_filter_b[i];
  drive->velocity = y * scale / M_PI;
}

void sim::set_joint_encoder(const mjModel *m, cassie_joint_out_t *joint,
                            const mjtNum *sensordata, joint_filter_t *filter,
                            int isensor) {
  // Position
  // Get digital encoder value
  int bits = m->sensor_user[m->nuser_sensor * isensor];
  int encoder_value = sensordata[isensor] / (2 * M_PI) * (1 << bits);
  double scale = (2 * M_PI) / (1 << bits);
  joint->position = encoder_value * scale;

  // Velocity
  // Initialize unfiltered signal array to prevent bad transients
  bool allzero = true;
  for (int i = 0; i < JOINT_FILTER_NB; ++i)
    allzero &= filter->x[i] == 0;
  if (allzero) {
    // If all filter values are zero, initialize the signal array
    // with the current encoder value
    for (int i = 0; i < JOINT_FILTER_NB; ++i)
      filter->x[i] = joint->position;
  }

  // Shift and update signal arrays
  for (int i = JOINT_FILTER_NB - 1; i > 0; --i)
    filter->x[i] = filter->x[i - 1];
  filter->x[0] = joint->position;
  for (int i = JOINT_FILTER_NA - 1; i > 0; --i)
    filter->y[i] = filter->y[i - 1];

  // Compute filter value
  filter->y[0] = 0;
  for (int i = 0; i < JOINT_FILTER_NB; ++i)
    filter->y[0] += filter->x[i] * joint_filter_b[i];
  for (int i = 1; i < JOINT_FILTER_NA; ++i)
    filter->y[0] -= filter->y[i] * joint_filter_a[i];
  joint->velocity = filter->y[0];
}

// double sim::set_motor(const mjModel *m, mjData *d, int motor_index,
//                                    double u, double *torque_delay, bool sto)
//                                    {
//   double ratio = m->actuator_gear[6 * motor_index];
//   double tmax = m->actuator_ctrlrange[2 * motor_index + 1];
//   double w = d->actuator_velocity[motor_index];
//   double wmax =
//       m->actuator_user[m->nuser_actuator * motor_index] * 2 * M_PI / 60;

//   // Calculate torque limit based on motor speed
//   double tlim = 2 * tmax * (1 - fabs(w) / wmax);
//   tlim = fmax(fmin(tlim, tmax), 0);

//   // Apply STO
//   if (sto)
//     u = 0;

//   // Compute motor-side torque
//   double tau = copysign(fmin(fabs(u / ratio), tlim), u);

//   // Torque delay line
//   d->ctrl[motor_index] = torque_delay[TORQUE_DELAY_CYCLES - 1];
//   for (int i = TORQUE_DELAY_CYCLES - 1; i > 0; --i)
//     torque_delay[i] = torque_delay[i - 1];
//   torque_delay[0] = tau;
//   // Return the current value of the output-side torque
//   return d->ctrl[motor_index] * ratio;
// }
