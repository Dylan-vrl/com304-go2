#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unistd.h>
// #include <pybind11/pybind11.h>
// namespace py = pybind11;

#define TOPIC_HIGHSTATE "rt/sportmodestate"

using namespace unitree::common;

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

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&Controller::HighStateHandler, this, std::placeholders::_1), 1);
  }

  void write() {
    ct += dt;
    double px_local, py_local, yaw_local;
    double vx_local, vy_local, vyaw_local;
    double px_err, py_err, yaw_err;
    double time_seg, time_temp;

    unitree::robot::go2::PathPoint path_point_tmp;
    std::vector<unitree::robot::go2::PathPoint> path;


    time_seg = 0.2;
    time_temp = ct - time_seg;
    for (int i = 0; i < 30; i++)
    {
      time_temp += time_seg;

      px_local = 0.2 * time_temp;
      vx_local = 0.5;

      path_point_tmp.timeFromStart = i * time_seg;
      path_point_tmp.x = px_local + px0;
      path_point_tmp.y = py0;
      path_point_tmp.yaw = yaw0;
      path_point_tmp.vx = vx_local;
      path_point_tmp.vy = 0;
      path_point_tmp.vyaw = 0;
      path.push_back(path_point_tmp);
    }
    sport_client->TrajectoryFollow(path);
  }


  void read() {

  }

  void GetInitState()
  {
    px0 = state.position()[0];
    py0 = state.position()[1];
    yaw0 = state.imu_state().rpy()[2];
    std::cout << "initial position: x0: " << px0 << ", y0: " << py0 << ", yaw0: " << yaw0 << std::endl;

    sport_client->StopMove();
    writeThread = CreateRecurrentThreadEx("write", UT_CPU_ID_NONE, 40000, &Controller::write, this);
    readThread = CreateRecurrentThreadEx("read", UT_CPU_ID_NONE, 40000, &Controller::read, this);
  };

  void HighStateHandler(const void *message)
  {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;

    // std::cout << "Position: " << state.position()[0] << ", " << state.position()[1] << ", " << state.position()[2] << std::endl;
    // std::cout << "IMU rpy: " << state.imu_state().rpy()[0] << ", " << state.imu_state().rpy()[1] << ", " << state.imu_state().rpy()[2] << std::endl;
  };

  ~Controller()
  {
    delete sport_client;
  };

  double px0, py0, yaw0;
  double ct = 0;
  float dt = 0.005;

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient *sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  ThreadPtr writeThread;
  ThreadPtr readThread;
};

// PYBIND11_MODULE(controller, m) {
//     py::class_<Controller>(m, "Controller")
//         .def(py::init<const char*>())
//         .def("standup", &Controller::standup)
//         .def("sitdown", &Controller::sitdown)
//         .def("vel_move", &Controller::vel_move)
//         .def("start_vel_move", &Controller::start_vel_move);
// }

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
  c.GetInitState();
  
  while(1) {
    sleep(2);
  }

  return 0;
}