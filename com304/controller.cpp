#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unistd.h>
#include <pybind11/pybind11.h>
namespace py = pybind11;

class Controller {
public: 
  Controller(const char* network_interface) {
    init(network_interface);
  }

  void init(const char* network_interface)
  {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);
    sport_client = new unitree::robot::go2::SportClient();
    sport_client->SetTimeout(10.0f);
    sport_client->Init();
  }

  void standup()
  {
    sport_client->StandUp();
  }

  void sitdown()
  {
    sport_client->Sit();
  }

  ~Controller()
  {
    delete sport_client;
  }

  unitree::robot::go2::SportClient *sport_client;
};

PYBIND11_MODULE(controller, m) {
    py::class_<Controller>(m, "Controller")
        .def(py::init<const char*>())
        .def("standup", &Controller::standup)
        .def("sitdown", &Controller::sitdown);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  //argv[1] is network interface of the robot
  Controller c(argv[1]);
  sleep(1);
  return 0;
}