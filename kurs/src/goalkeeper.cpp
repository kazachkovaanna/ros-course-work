#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "robot.h"
#include <random>

 
using namespace std;
 
int main(int argc, char** argv) {
 
    ros::init(argc, argv, "keeper");
    GazeboService::getInstance();
    ros::service::waitForService("gazebo/spawn_sdf_model"); 
    ros::Rate rate(0.25);

    string name;
    int team, number, total;
    double x,y;
    ros::param::get("~team",team);
    ros::param::get("~number",number);
    ros::param::get("~name", name);
	ros::param::get("~x", x);
    ros::param::get("~y", y);

    std::cout<<"name "<<name<<" team "<<team<<" x "<<x<<" y "<<y<<std::endl;
    Robot* player;
    player = new Robot(x, y, team, number, total, "goalkeeper"); 
  
    rate.sleep();
    random_device rd;
    uniform_real_distribution<double> interval(-2.0, 2.0);

    while(ros::ok()){
        if(team==1)
        player->moveTo(player->getTransofrms("ball").getBall().position.x, 10.0);
        else 
        player->moveTo(player->getTransofrms("ball").getBall().position.x, -10.0);
    }
    ros::spinOnce();
    return 0;
}