#include <pybind11/pybind11.h>

#include "agilitycassie/cassie_in_t.h"
#include "agilitycassie/cassie_user_in_t.h"
#include "agilitycassie/include/agilitycassie/cassie_out_t.h"
#include "agilitycassie/pd_in_t.h"
#include "agilitycassie/state_out_t.h"

#include "cassiemujoco/cassiemujoco_cpp.h"
#include "cassiemujoco/udp_cpp.h"

// #define STRINGIFY(x) #x
// #define MACRO_STRINGIFY(x) STRINGIFY(x)

namespace py = pybind11;

PYBIND11_MODULE(cassie_mujoco_sim_py, m) {
  m.doc() = R"pbdoc(        
    )pbdoc";

  py::class_<cassie::sim::CassieUdp>(m, "CassieUdp")
      .def(py::init<const std::string &, const std::string &,
                    const std::string &, const std::string &>(),
           py::arg("remote_addr") = "127.0.0.1",
           py::arg("remote_port") = "25000", py::arg("local_addr") = "0.0.0.0",
           py::arg("local_port") = "25001")
      .def("send", &cassie::sim::CassieUdp::send)
      .def("send_pd", &cassie::sim::CassieUdp::send_pd)
      .def("recv_wait_pd", &cassie::sim::CassieUdp::recv_wait_pd)
      .def("recv_newest_pd", &cassie::sim::CassieUdp::recv_newest_pd)
      .def("recv_wait", &cassie::sim::CassieUdp::recv_wait)
      .def("recv_newest", &cassie::sim::CassieUdp::recv_newest)
      .def("delay", &cassie::sim::CassieUdp::delay)
      .def("seq_num_in_diff", &cassie::sim::CassieUdp::seq_num_in_diff);

  py::class_<cassie::sim::DefaultSimulation>(m, "DefaultSimulation")
      .def(py::init<const std::string &>())
      .def("step", &cassie::sim::DefaultSimulation::step);

  // Input structs
  py::class_<elmo_in_t>(m, "elmo_in_t")
      .def_readwrite("controlWord", &elmo_in_t::controlWord)
      .def_readwrite("torque", &elmo_in_t::torque);

  py::class_<cassie_leg_in_t>(m, "cassie_leg_in_t")
      .def_readwrite("hipRollDrive", &cassie_leg_in_t::hipRollDrive)
      .def_readwrite("hipYawDrive", &cassie_leg_in_t::hipYawDrive)
      .def_readwrite("hipPitchDrive", &cassie_leg_in_t::hipPitchDrive)
      .def_readwrite("kneeDrive", &cassie_leg_in_t::kneeDrive)
      .def_readwrite("footDrive", &cassie_leg_in_t::footDrive);

  py::class_<cassie_pelvis_in_t>(m, "cassie_pelvis_in_t")
      .def_readwrite("sto", &cassie_pelvis_in_t::sto);

  py::class_<cassie_in_t>(m, "cassie_in_t")
      .def_readwrite("leftLeg", &cassie_in_t::leftLeg)
      .def_readwrite("rightLeg", &cassie_in_t::rightLeg);

  py::class_<cassie_user_in_t>(m, "cassie_user_in_t")
      .def_readonly("torque", &cassie_user_in_t::torque)
      .def_readonly("telemetry", &cassie_user_in_t::telemetry);

  // typedef struct {
  //   double torque[5];
  //   double pTarget[5];
  //   double dTarget[5];
  //   double pGain[5];
  //   double dGain[5];
  // } pd_motor_in_t;

  py::class_<pd_motor_in_t>(m, "pd_motor_in_t")
      .def_readonly("torque", &pd_motor_in_t::torque);

  // typedef struct {
  //   double torque[6];
  //   double pTarget[6];
  //   double dTarget[6];
  //   double pGain[6];
  //   double dGain[6];
  // } pd_task_in_t;

  py::class_<pd_task_in_t>(m, "pd_task_in_t")
      .def_readonly("torque", &pd_task_in_t::torque);

  py::class_<pd_leg_in_t>(m, "pd_leg_in_t")
      .def_readonly("taskPd", &pd_leg_in_t::taskPd)
      .def_readonly("motorPd", &pd_leg_in_t::motorPd);

  py::class_<pd_in_t>(m, "pd_in_t")
      .def(py::init([]() { return pd_in_t{}; }))
      .def_readwrite("leftLeg", &pd_in_t::leftLeg)
      .def_readwrite("rightLeg", &pd_in_t::rightLeg)
      .def_readonly("telemetry", &pd_in_t::telemetry);

  // Output structs

  py::class_<battery_out_t>(m, "battery_out_t")
      .def_readwrite("dataGood", &battery_out_t::dataGood)
      .def_readwrite("stateOfCharge", &battery_out_t::stateOfCharge)
      .def_readonly("voltage", &battery_out_t::voltage)
      .def_readonly("current", &battery_out_t::current)
      .def_readonly("temperature", &battery_out_t::temperature);

  py::class_<cassie_joint_out_t>(m, "cassie_joint_out_t")
      .def_readwrite("position", &cassie_joint_out_t::position)
      .def_readwrite("velocity", &cassie_joint_out_t::velocity);

  py::class_<elmo_out_t>(m, "elmo_out_t")
      .def_readwrite("position", &elmo_out_t::position)
      .def_readwrite("velocity", &elmo_out_t::velocity)
      .def_readwrite("torque", &elmo_out_t::torque)
      .def_readwrite("driveTemperature", &elmo_out_t::driveTemperature)
      .def_readwrite("torqueLimit", &elmo_out_t::torqueLimit)
      .def_readwrite("gearRatio", &elmo_out_t::gearRatio);

  py::class_<cassie_leg_out_t>(m, "cassie_leg_out_t")
      .def_readonly("hipRollDrive", &cassie_leg_out_t::hipRollDrive)
      .def_readonly("hipYawDrive", &cassie_leg_out_t::hipYawDrive)
      .def_readonly("hipPitchDrive", &cassie_leg_out_t::hipPitchDrive)
      .def_readonly("kneeDrive", &cassie_leg_out_t::kneeDrive)
      .def_readonly("footDrive", &cassie_leg_out_t::footDrive)
      .def_readonly("shinJoint", &cassie_leg_out_t::shinJoint)
      .def_readonly("tarsusJoint", &cassie_leg_out_t::tarsusJoint)
      .def_readonly("footJoint", &cassie_leg_out_t::footJoint);

  py::class_<radio_out_t>(m, "radio_out_t")
      .def_readwrite("radioReceiverSignalGood",
                     &radio_out_t::radioReceiverSignalGood)
      .def_readwrite("receiverMedullaSignalGood",
                     &radio_out_t::receiverMedullaSignalGood)
      .def_readonly("channel", &radio_out_t::channel);

  py::class_<target_pc_out_t>(m, "target_pc_out_t")
      .def_readonly("etherCatStatus", &target_pc_out_t::etherCatStatus)
      .def_readonly("etherCatNotifications",
                    &target_pc_out_t::etherCatNotifications)
      .def_readonly("taskExecutionTime", &target_pc_out_t::taskExecutionTime);

  py::class_<vectornav_out_t>(m, "vectornav_out_t")
      .def_readonly("pressure", &vectornav_out_t::pressure)
      .def_readonly("temperature", &vectornav_out_t::temperature)
      .def_readonly("magneticField", &vectornav_out_t::magneticField)
      .def_readonly("angularVelocity", &vectornav_out_t::angularVelocity)
      .def_readonly("linearAcceleration", &vectornav_out_t::linearAcceleration)
      .def_readonly("orientation", &vectornav_out_t::orientation);

  py::class_<cassie_pelvis_out_t>(m, "cassie_pelvis_out_t")
      .def_readonly("targetPc", &cassie_pelvis_out_t::targetPc)
      .def_readonly("batttery", &cassie_pelvis_out_t::battery)
      .def_readonly("radio", &cassie_pelvis_out_t::radio)
      .def_readonly("vectorNav", &cassie_pelvis_out_t::vectorNav);

  py::class_<cassie_out_t>(m, "cassie_out_t")
      .def_readwrite("pelvis", &cassie_out_t::pelvis)
      .def_readonly("leftLeg", &cassie_out_t::leftLeg)
      .def_readonly("rightLeg", &cassie_out_t::rightLeg)
      .def_readwrite("isCalibrated", &cassie_out_t::isCalibrated);

  m.def("pack_cassie_out_t", &pack_cassie_out_t,
        "serialize cassie_out_t struct into a packet");
  m.def("unpack_cassie_out_t", &unpack_cassie_out_t,
        "deserialize cassie_out_t struct from a packet");

  // state out

  // typedef struct {
  //   state_pelvis_out_t pelvis;
  //   state_foot_out_t leftFoot;
  //   state_foot_out_t rightFoot;
  //   state_terrain_out_t terrain;
  //   state_motor_out_t motor;
  //   state_joint_out_t joint;
  //   state_radio_out_t radio;
  //   state_battery_out_t battery;
  // } state_out_t;
  py::class_<state_out_t>(m, "state_out_t").def(py::init([]() {
    return state_out_t{};
  }));

  m.def("pack_state_out_t", &pack_state_out_t, "serialize state_out_t");
  m.def("unpack_state_out_t", &unpack_state_out_t,
        "deserialize state_out_t struct");
}