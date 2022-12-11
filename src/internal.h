#ifndef INCLUDE_INTERNAL
#define INCLUDE_INTERNAL

#include <stdio.h>

// Debug using one string literal. x cannot contain other printf qualifiers.
// TODO: replace this with a version that allows substitution qualifiers.
#define DBGF(x) fprintf(stderr, "%s:%d " x "\n", __FILE__, __LINE__)

/*******************************************************************************
 * Sensor filtering
 ******************************************************************************/

#define DRIVE_FILTER_NB 9
#define JOINT_FILTER_NB 4
#define JOINT_FILTER_NA 3

#define NUM_DRIVES 10
#define NUM_JOINTS 6
#define TORQUE_DELAY_CYCLES 6

typedef struct drive_filter {
  int x[DRIVE_FILTER_NB];
} drive_filter_t;

typedef struct joint_filter {
  double x[JOINT_FILTER_NB];
  double y[JOINT_FILTER_NA];
} joint_filter_t;

#endif /* INCLUDE_INTERNAL */
