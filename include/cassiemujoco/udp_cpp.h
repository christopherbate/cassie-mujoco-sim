#ifndef INCLUDE_CASSIEMUJOCO_UDP_CPP
#define INCLUDE_CASSIEMUJOCO_UDP_CPP

#include "agilitycassie/include/agilitycassie/cassie_out_t.h"
#include "agilitycassie/include/agilitycassie/cassie_user_in_t.h"
#include "agilitycassie/include/agilitycassie/pd_in_t.h"
#include "agilitycassie/include/agilitycassie/state_out_t.h"
#include "cassiemujoco/udp.h"
#include <string>
#include <vector>
namespace cassie {
namespace sim {

class CassieUdp {
public:
  CassieUdp(const std::string &remote_addr, const std::string &remote_port,
            const std::string &local_addr, const std::string &local_port);
  ~CassieUdp();

  void send(const cassie_user_in_t &u);
  void send_pd(const pd_in_t &u);

  cassie_out_t recv_wait();
  cassie_out_t recv_newest();

  state_out_t recv_wait_pd();
  state_out_t recv_newest_pd();

  int delay() const;
  int seq_num_in_diff() const;

private:
  int sock;
  packet_header_info_t packet_header_info{};
  const size_t recvlen = 2 + 697;
  const size_t sendlen = 2 + 58;
  const size_t recvlen_pd = 2 + 493;
  const size_t sendlen_pd = 2 + 476;
  std::vector<uint8_t> recvbuf;
  std::vector<uint8_t> sendbuf;
  uint8_t *inbuf;
  uint8_t *outbuf;
};

} // namespace sim
} // namespace cassie

#endif /* INCLUDE_CASSIEMUJOCO_UDP_CPP */
