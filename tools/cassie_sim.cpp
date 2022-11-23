#include "agilitycassie/cassie_user_in_t.h"
#include "cassiemujoco/cassiemujoco_cpp.h"
#include <iostream>
#include <vector>

#include "cassiemujoco/udp.h"

using namespace cassie;

static const uint8_t *getRecvDataPtr(const std::vector<uint8_t> &recvBuffer) {
  const uint8_t *data_in = &recvBuffer[PACKET_HEADER_LEN];
  return data_in;
}

static uint8_t *getMutableSendDataPtr(std::vector<uint8_t> &sendBuffer) {
  return &sendBuffer[PACKET_HEADER_LEN];
}

static packet_header_info_t
processInputPacket(const std::vector<uint8_t> &recv_buffer,
                   std::vector<uint8_t> sendBuffer) {
  // Create header information struct
  packet_header_info_t header_info = {0};
  // Process incoming header and write outgoing header
  process_packet_header(&header_info, recv_buffer.data(), sendBuffer.data());
  return header_info;
}

int main(int argc, char **argv) {

  // Bind to network interface
  const char *iface_addr_str = "0.0.0.0";
  const char *iface_port_str = "25000";
  int sock = udp_init_host(iface_addr_str, iface_port_str);
  if (-1 == sock)
    exit(EXIT_FAILURE);

  // Create packet input/output buffers
  const size_t dinlen = CASSIE_USER_IN_T_PACKED_LEN;
  const size_t doutlen = CASSIE_OUT_T_PACKED_LEN;
  const size_t recvlen = PACKET_HEADER_LEN + dinlen;
  const size_t sendlen = PACKET_HEADER_LEN + doutlen;
  std::vector<uint8_t> recvbuf(recvlen);
  std::vector<uint8_t> sendbuf(sendlen);

  // Address to send sensor data packets to
  struct sockaddr_storage src_addr = {0};
  socklen_t addrlen = sizeof src_addr;

  std::cout << "Listening on " << iface_addr_str << ":" << iface_port_str
            << "\n";

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
  int64_t lastPacketTime = cassie::sim::getMicroseconds();
  bool sendResponse = false;
  while (!renderer->shouldClose()) {

    // Get newest packet, or return -1 if no new packets are available
    ssize_t nbytes = get_newest_packet(sock, recvbuf.data(), recvlen,
                                       (struct sockaddr *)&src_addr, &addrlen);

    // If a new packet was received, process and unpack it
    if (recvlen == nbytes) {
      (void)processInputPacket(recvbuf, sendbuf);

      // printf("\033[F\033[Jdelay: %d, diff: %d\n",
      //        header_info.delay, header_info.seq_num_in_diff);

      // Unpack received data into cassie user input struct
      unpack_cassie_user_in_t(getRecvDataPtr(recvbuf), &userInput);

      // Update packet received timestamp
      lastPacketTime = cassie::sim::getMicroseconds();
      sendResponse = true;
    }

    cassie_out_t cassieOut = simulation->step(userInput);
    pack_cassie_out_t(&cassieOut, getMutableSendDataPtr(sendbuf));

    // Send response
    if (sendResponse) {
      send_packet(sock, sendbuf.data(), sendlen, (struct sockaddr *)&src_addr,
                  addrlen);
      sendResponse = false;
    }

    renderer->renderOnce(simulation->getMujocoSim()->model,
                         simulation->getMujocoSim()->data);
  }

  return 0;
}