#include <volasim_msgs/DroneState.pb.h>
#include <zmq.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

int main(int argc, char* argv[]) {
  if (argc != 4) {
    std::cerr << "usage: send_position <x> <y> <z>" << std::endl;
    return 1;
  }

  double x = std::stod(argv[1]);
  double y = std::stod(argv[2]);
  double z = std::stod(argv[3]);

  zmq::context_t ctx(1);
  zmq::socket_t  push(ctx, zmq::socket_type::push);
  push.connect("ipc:///tmp/volasim_cmd_pos");

  volasim_msgs::DroneState msg;
  auto*                    pos = msg.mutable_odom()->mutable_position();
  pos->set_x(static_cast<float>(x));
  pos->set_y(static_cast<float>(y));
  pos->set_z(static_cast<float>(z));

  std::string bytes;
  (void)msg.SerializeToString(&bytes);
  push.send(zmq::buffer(bytes), zmq::send_flags::none);

  std::cout << "[send_position] sent: " << x << " " << y << " " << z
            << std::endl;
  return 0;
}
