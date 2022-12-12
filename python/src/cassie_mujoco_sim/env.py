"""env.py

Provides the implementation for the Cassie Mujoco environment. Note that this is not necessarily
an "OpenAI Gym" environment, rather the CassieEnv implements a simple reset/step contract.

The source is based on the environment from github.com/osudrl/RSS-2020-learning-memory-based-control.
"""
from collections import namedtuple
import logging
from math import floor
from pathlib import Path
import random

from .cassiemujoco import pd_in_t, state_out_t, CassieSim, CassieVis

import numpy as np

from .config import CassieEnvParameters

# Global Constants
_STATE_EST_SIZE = 46
_CLOCK_SIZE = 2
_SPEED_SIZE = 1

log = logging.getLogger(__name__)


class Trajectory:
    """ A utility class that loads a specific trajectory.
    """

    def __init__(self, path: Path):
        # TODO: eliminate magic numbers: time, qp rank, qv rank, torque, mpos rank, mvel rank
        COL_WIDTH = 1 + 35 + 32 + 10 + 10 + 10
        data = np.fromfile(str(path), dtype=np.double).reshape((-1, COL_WIDTH))

        # states
        self.time = data[:, 0]
        self.qpos = data[:, 1:36]
        self.qvel = data[:, 36:68]

        # actions
        self.torque = data[:, 68:78]
        self.mpos = data[:, 78:88]
        self.mvel = data[:, 88:98]

    def __len__(self):
        return len(self.time)


StepResult = namedtuple("StepResult", ["state", "reward", "done", "unused"])


class CassieEnvironment:
    """ The Cassie environment as implemented in: github.com/osudrl/RSS-2020-learning-memory-based-control.

    Note that minor modifications were made:
        - the class now accepts a configuration schema
        - type hints added
        - cleanup: logging, dead code, etc.
        - dynamics randomization was removed as this is not used (yet). If required,
          it can be added back in.
    
    Code still requires a lot of cleanup, but we leave most as-is to avoid breaking
    anything right now.
    
    TODO: Once results are stabilized, revisit this code.
    """
    def __init__(self, params: CassieEnvParameters):
        log.info("Loading Cassie MJCF model from: %s", params.xml_model_path)
        self.sim = CassieSim(modelfile=params.xml_model_path)
        self.trajectory = Trajectory(Path(params.trajectory_data_path))
        self.vis = None

        self.params = params

        self.observation_space = np.zeros(
            _STATE_EST_SIZE + _CLOCK_SIZE + _SPEED_SIZE)
        self.action_space = np.zeros(10)

        # Mutable state variables
        self.P = np.array([100,  100,  88,  96,  50])
        self.D = np.array([10.0, 10.0, 8.0, 9.6, 5.0])

        u = pd_in_t()
        for i in range(5):
            u.leftLeg.motorPd.torque[i] = 0
            u.rightLeg.motorPd.torque[i] = 0
            u.leftLeg.motorPd.dTarget[i] = 0
            u.rightLeg.motorPd.dTarget[i] = 0
            u.leftLeg.motorPd.pGain[i] = 0
            u.rightLeg.motorPd.pGain[i] = 0
            u.leftLeg.motorPd.dGain[i] = 0
            u.rightLeg.motorPd.dGain[i] = 0

            u.leftLeg.motorPd.pTarget[i] = 0
            u.rightLeg.motorPd.pTarget[i] = 0
        self.u = u

        self.cassie_state = state_out_t()

        self.sim_rate = 60  # simulate X mujoco steps with same pd target
        # 60 brings simulation from 2000Hz to roughly 30Hz
        self.time = 0  # number of time steps in current episode
        self.phase = 0  # portion of the phase the robot is in
        self.counter = 0  # number of phase cycles completed in episode

        # NOTE: a reference trajectory represents ONE phase cycle
        self.phase_len = floor(len(self.trajectory) / self.sim_rate) - 1
        log.info("Trajectory phase length: %d", self.phase_len)

        # see include/cassiemujoco.h for meaning of these indices
        self.pos_idx = [7, 8, 9, 14, 20, 21, 22, 23, 28, 34]
        self.vel_idx = [6, 7, 8, 12, 18, 19, 20, 21, 25, 31]
        self.offset = np.array(
            [0.0045, 0.0, 0.4973, -1.1997, -1.5968, 0.0045, 0.0, 0.4973, -1.1997, -1.5968])
        self.speed = 0
        self.phase_add = 1

        # Record default dynamics parameters
        self.default_damping = self.sim.get_dof_damping()
        self.default_mass = self.sim.get_body_mass()
        self.default_ipos = self.sim.get_body_ipos()

    def get_observation_space_rank(self):
        return self.observation_space.shape[-1]

    def get_action_space_rank(self):
        return self.action_space.shape[0]

    def step_simulation(self, action):

        target = action + self.offset

        self.u = pd_in_t()
        for i in range(5):
            # TODO: move setting gains out of the loop?
            # maybe write a wrapper for pd_in_t ?
            self.u.leftLeg.motorPd.pGain[i] = self.P[i]
            self.u.rightLeg.motorPd.pGain[i] = self.P[i]

            self.u.leftLeg.motorPd.dGain[i] = self.D[i]
            self.u.rightLeg.motorPd.dGain[i] = self.D[i]

            self.u.leftLeg.motorPd.torque[i] = 0  # Feedforward torque
            self.u.rightLeg.motorPd.torque[i] = 0

            self.u.leftLeg.motorPd.pTarget[i] = target[i]
            self.u.rightLeg.motorPd.pTarget[i] = target[i + 5]

            self.u.leftLeg.motorPd.dTarget[i] = 0
            self.u.rightLeg.motorPd.dTarget[i] = 0

        self.cassie_state = self.sim.step_pd(self.u)

    def step(self, action) -> StepResult:
        """ Each step with run the mujoco sim `self.simrate` times 
        with the same action input. Then reward will be calculated.
        """
        for _ in range(self.sim_rate):
            self.step_simulation(action)

        height = self.sim.qpos()[2]

        self.time += 1
        self.phase += self.phase_add

        if self.phase > self.phase_len:
            self.phase = 0
            self.counter += 1

        # Early termination
        done = not (height > 0.4 and height < 3.0)

        reward = self.compute_reward()

        if reward < 0.3:
            done = True

        return StepResult(self.get_full_state(), reward, done, {})

    def reset(self):
        """ Resets the environment. The returned state is placed at a random phase
        point along the walking cycle trajectory.

        Returns the full state.
        """
        self.phase = random.randint(0, self.phase_len)
        self.time = 0
        self.counter = 0

        qpos, qvel = self.get_ref_state(self.phase)

        self.sim.set_qpos(qpos)
        self.sim.set_qvel(qvel)
        self.sim.set_dof_damping(self.default_damping)
        self.sim.set_body_mass(self.default_mass)
        self.sim.set_body_ipos(self.default_ipos)
        self.sim.set_const()
        self.cassie_state = self.sim.step_pd(self.u)
        self.speed = np.random.uniform(-0.15, 0.8)
        state = self.get_full_state()
        return state

    def compute_reward(self):
        """TODO: organize these calculations better and outline 
        some portion of them to a file.
        """
        qpos = np.asarray(self.sim.qpos())
        qvel = np.asarray(self.sim.qvel())

        ref_pos, _ = self.get_ref_state(self.phase)

        joint_error = 0
        com_error = 0
        orientation_error = 0
        spring_error = 0

        # each joint pos
        weight = [0.15, 0.15, 0.1, 0.05, 0.05, 0.15, 0.15, 0.1, 0.05, 0.05]
        for i, j in enumerate(self.pos_idx):
            target = ref_pos[j]
            actual = qpos[j]

            joint_error += 30 * weight[i] * (target - actual) ** 2

        forward_diff = np.abs(qvel[0] - self.speed)
        if forward_diff < 0.05:
            forward_diff = 0

        y_vel = np.abs(qvel[1])
        if y_vel < 0.03:
            y_vel = 0

        straight_diff = np.abs(qpos[1])
        if straight_diff < 0.05:
            straight_diff = 0

        actual_q = qpos[3:7]
        target_q = [1, 0, 0, 0]
        orientation_error = 5 * (1 - np.inner(actual_q, target_q) ** 2)

        # left and right shin springs
        for i in [15, 29]:
            target = ref_pos[i]
            actual = qpos[i]

            spring_error += 1000 * (target - actual) ** 2

        reward = 0.000 + \
            0.300 * np.exp(-orientation_error) + \
            0.200 * np.exp(-joint_error) +       \
            0.200 * np.exp(-forward_diff) +      \
            0.200 * np.exp(-y_vel) +             \
            0.050 * np.exp(-straight_diff) +     \
            0.050 * np.exp(-spring_error)

        return reward

    def get_damping(self):
        return np.array(self.sim.get_dof_damping())

    def get_mass(self):
        return np.array(self.sim.get_body_mass())

    def get_ipos(self):
        return np.array(self.sim.get_body_ipos()[3:6])

    def get_ref_state(self, phase=None):
        """ Get the corresponding state from the reference
        trajectory for the current phase
        """
        if phase is None:
            phase = self.phase

        if phase > self.phase_len:
            phase = 0

        pos = np.copy(self.trajectory.qpos[phase * self.sim_rate])

        ###### Setting variable speed  #########
        pos[0] *= self.speed
        pos[0] += (self.trajectory.qpos[-1, 0] -
                   self.trajectory.qpos[0, 0]) * self.counter * self.speed
        ######                          ########

        # setting lateral distance target to 0
        # regardless of reference trajectory
        pos[1] = 0

        vel = np.copy(self.trajectory.qvel[phase * self.sim_rate])
        vel[0] *= self.speed

        return pos, vel

    def get_full_state(self) -> np.ndarray:
        clock = [np.sin(2 * np.pi * self.phase / self.phase_len),
                 np.cos(2 * np.pi * self.phase / self.phase_len)]

        ext_state = np.concatenate((clock, [self.speed]))

        # Use state estimator
        robot_state = np.concatenate([
            [self.cassie_state.pelvis.position[2] -
                self.cassie_state.terrain.height],  # pelvis height
            # pelvis orientation
            self.cassie_state.pelvis.orientation[:],
            # actuated joint positions
            self.cassie_state.motor.position[:],
            # pelvis translational velocity
            self.cassie_state.pelvis.translationalVelocity[:],
            # pelvis rotational velocity
            self.cassie_state.pelvis.rotationalVelocity[:],
            # actuated joint velocities
            self.cassie_state.motor.velocity[:],
            # pelvis translational acceleration
            self.cassie_state.pelvis.translationalAcceleration[:],
            # unactuated joint positions
            self.cassie_state.joint.position[:],
            # unactuated joint velocities
            self.cassie_state.joint.velocity[:]
        ])
        full_state = np.concatenate([robot_state, ext_state])
        return full_state

    def render(self):
        if self.vis is None:
            self.vis = CassieVis(self.sim)
        self.vis.draw(self.sim)
