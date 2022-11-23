#ifndef SIM_CASSIE_SENSORS
#define SIM_CASSIE_SENSORS

#include "agilitycassie/cassie_out_t.h"
#include "cassiemujoco/cassiemujoco_cpp.h"

/*******************************************************************************
 * Utilities for the agilitycassie data structures
 ******************************************************************************/

namespace cassie::sim {

cassie_out_t getCassieOutDefault();

void set_drive_encoder(const mjModel *m, elmo_out_t *drive,
                       const mjtNum *sensordata, drive_filter_t *filter,
                       int isensor);

void set_joint_encoder(const mjModel *m, cassie_joint_out_t *joint,
                       const mjtNum *sensordata, joint_filter_t *filter,
                       int isensor);

// void set_drive_encoder(const mjModel *m, elmo_out_t *drive,
//                        const mjtNum *sensordata, drive_filter_t *filter,
//                        int isensor);

// double set_motor(const mjModel *m, mjData *d, int motor_index, double u,
//                  double *torque_delay, bool sto);

} // namespace cassie::sim

#endif // SIM_CASSIE_SENSORS
