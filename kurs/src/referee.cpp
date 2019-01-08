#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "robot.h"
#include <random>
 
using namespace std;
 
int main(int argc, char** argv) {
    ROS_INFO("-1");
    ros::init(argc, argv, "player");
    ROS_INFO("0");
    GazeboService::getInstance();

    ROS_INFO("1");

    ros::service::waitForService("gazebo/spawn_sdf_model"); 

   
    while(ros::ok()){
        //play
    }
    ros::spinOnce();
    return 0;
}