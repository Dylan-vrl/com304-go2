// Minimalist script that makes the dog sit down then stand up.

#include <unitree/robot/go2/sport/sport_client.hpp>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    std::cout << "Usage: " << argv[0] << " networkInterface" << std::endl;
    exit(-1);
  }

  unitree::robot::ChannelFactory::Instance()->Init(0, argv[1]);  
  unitree::robot::go2::SportClient sport_client;
  sport_client.SetTimeout(10.0f);
  sport_client.Init();

  sport_client.StandUp();
  sleep(1);
  sport_client.Sit();
  sleep(1);
  sport_client.StandUp();
  sleep(1);
  
  return 0;
}