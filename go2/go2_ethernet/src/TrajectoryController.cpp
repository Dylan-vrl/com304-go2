#include <unitree/robot/go2/sport/sport_client.hpp>
#include <unitree/robot/channel/channel_subscriber.hpp>
#include <unitree/idl/go2/SportModeState_.hpp>
#include <unistd.h>
#include <cmath>

#include <pybind11/pybind11.h>
namespace py = pybind11;

#define TOPIC_HIGHSTATE "rt/sportmodestate"
#define DELTA_TIME 0.002 // Threads period in seconds, 0.001~0.01

#define GOAL_EPSILON 0.002 // Distance to be considered as reaching the goal

#define LOW_DIST_VEL 0.2
#define HIGH_DIST_VEL 0.4

#define LOW_ANG_VEL 0.4
#define HIGH_ANG_VEL 0.8 

#define LOW_VEL_DIST_THRESHOLD 0.3 // Distance from the goal from which we slow down
#define LOW_VEL_ANG_THRESHOLD M_PI_4 // Distance from the goal from which we slow down

using namespace unitree::common;

// Controls the robot using trajectory follow commands
class TrajectoryController {
public: 
  enum Axis {
    X = 0,
    Y = 1,
    YAW = 2,
    AXIS_COUNT = 3
  };

  VelocityController(const char* network_interface) {
    Init(network_interface);
  }

  /// @brief Rotate the robot by deltaYaw radians counter-clockwise (clockwise if deltaY is negative)
  /// @param deltaYaw angle in radians between -PI and PI
  void Rotate(double deltaYaw) {
    Stop(); // Stop any ongoing movement before starting rotation

    // Normalize goal to be between -PI and PI
    double goalYaw = pose[YAW] + deltaYaw;
    while(goalYaw > M_PI)
      goalYaw -= 2 * M_PI;
    while(goalYaw < -M_PI) 
      goalYaw += 2 * M_PI;

    this->goal[YAW] = goalYaw;
    this->shouldMove[YAW] = abs(deltaYaw) > 0;

    std::cout << "Start rotating" << std::endl;
  }

  void Move(double deltaX, double deltaY) {
    Stop(); // Stop any ongoing movement before starting translation

    this->goal[X] = pose[X] + deltaX;
    this->goal[Y] = pose[Y] + deltaY;

    this->shouldMove[X] = abs(deltaX) > 0;
    this->shouldMove[Y] = abs(deltaY) > 0;

    std::cout << "Start moving" << std::endl;
  }

  void Stop() {
    sport_client->StopMove();
    for (size_t i = 0; i < AXIS_COUNT; i++) {
      shouldMove[i] = false;
    }
  }

  void Stop(Axis axis) {
    shouldMove[axis] = false;
    for (size_t i = 0; i < AXIS_COUNT; i++) {
      if (shouldMove[i])
        return;
    }
    sport_client->StopMove();
  }

  ~VelocityController() {
    delete sport_client;
  }

private:
  const double LOW_VEL[AXIS_COUNT] = { LOW_DIST_VEL, LOW_DIST_VEL, LOW_ANG_VEL };
  const double HIGH_VEL[AXIS_COUNT] = { HIGH_DIST_VEL, HIGH_DIST_VEL, HIGH_ANG_VEL };
  const double LOW_VEL_THRESHOLD[AXIS_COUNT] = { LOW_VEL_DIST_THRESHOLD, LOW_VEL_DIST_THRESHOLD, LOW_VEL_ANG_THRESHOLD };

  double pose[AXIS_COUNT] = { }; // 0: x, 1: y, 2: yaw

  double goal[AXIS_COUNT] = { };
  bool shouldMove[AXIS_COUNT] = { };

  unitree_go::msg::dds_::SportModeState_ state;
  unitree::robot::go2::SportClient *sport_client;
  unitree::robot::ChannelSubscriberPtr<unitree_go::msg::dds_::SportModeState_> suber;

  ThreadPtr writeThread;
  ThreadPtr readThread;

  void Init(const char* network_interface) {
    unitree::robot::ChannelFactory::Instance()->Init(0, network_interface);

    sport_client = new unitree::robot::go2::SportClient();
    sport_client->SetTimeout(10.0f);
    sport_client->Init();

    suber.reset(new unitree::robot::ChannelSubscriber<unitree_go::msg::dds_::SportModeState_>(TOPIC_HIGHSTATE));
    suber->InitChannel(std::bind(&VelocityController::HighStateHandler, this, std::placeholders::_1), 1);

    sport_client->StopMove();
    Read();

    writeThread = CreateRecurrentThreadEx("write", UT_CPU_ID_NONE, DELTA_TIME * 1000000, &VelocityController::Write, this);
    readThread = CreateRecurrentThreadEx("read", UT_CPU_ID_NONE, DELTA_TIME * 1000000, &VelocityController::Read, this);
  }

  void Write() {
    bool moveThisFrame = false;
    float v[AXIS_COUNT] = { 0 };

    for (size_t axis = 0; axis < AXIS_COUNT; axis++) {
      if (shouldMove[axis])
        moveThisFrame = true;
      else
        continue;
      v[axis] = (DistToGoal((Axis)axis) < LOW_VEL_THRESHOLD[axis] ? LOW_VEL : HIGH_VEL)[axis];
      v[axis] *= DirToGoal((Axis)axis);

      if (GoalReached((Axis)axis)) {
        std::cout << "Goal reached for axis " << axis << std::endl;
        Stop((Axis)axis);
        v[axis] = 0;
      }
    }

    if (!moveThisFrame)
      return;

    sport_client->Move(v[0], v[1], v[2]);
  }


  void Read() {
    this->pose[X] = state.position()[0];
    this->pose[Y] = state.position()[1];
    this->pose[YAW] = state.imu_state().rpy()[2];
  }

  double DistToGoal(Axis axis) {
    double current = pose[axis];
    return abs(goal[axis] - current);
  }

  double DirToGoal(Axis axis) {
    double current = pose[axis];
    return goal[axis] - current > 0 ? 1 : -1;
  }

  bool GoalReached(Axis axis) {
    return DistToGoal(axis) < GOAL_EPSILON;
  }

  void HighStateHandler(const void *message) {
    state = *(unitree_go::msg::dds_::SportModeState_ *)message;
  };
};

PYBIND11_MODULE(VelocityController, m) {
  py::class_<VelocityController> VelocityController(m, "VelocityController");

  VelocityController.def(py::init<const char*>())
    .def("move", &VelocityController::Move)
    .def("rotate", &VelocityController::Rotate)
    .def("stop", static_cast<void (VelocityController::*)()>(&VelocityController::Stop))
    .def("stop", static_cast<void (VelocityController::*)(VelocityController::Axis)>(&VelocityController::Stop));

  py::enum_<VelocityController::Axis>(VelocityController, "Axis")
    .value("X", VelocityController::Axis::X)
    .value("Y", VelocityController::Axis::Y)
    .value("YAW", VelocityController::Axis::YAW);
}

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }
  //argv[1] is network interface of the robot
  VelocityController c(argv[1]);
  sleep(1);
  c.Rotate(M_PI);
  while(1) {
    sleep(2);
  }

  return 0;
}