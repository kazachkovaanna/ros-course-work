#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "robot.h"
#include <random>

 
using namespace std;
 
int main(int argc, char** argv) {
    ros::init(argc, argv, "player");
    GazeboService::getInstance();
    ros::service::waitForService("gazebo/spawn_sdf_model"); 

    // initializing with parameters
    ros::Rate rate(0.25);
    string name;
    int team, number, total;
    double x,y;
    ros::param::get("~team",team);
    ros::param::get("~number",number);
    ros::param::get("~total",total);
	ros::param::get("~x", x);
    ros::param::get("~y", y);

    name = "player"+team+number;

    //spawn model to gazebo
    Robot* player;
    player = new Robot(x, y, team, number,total, "player"); 
    random_device rd;
    uniform_real_distribution<double> interval(-10.0, 10.0);
    rate.sleep();
    std::cout<<"Created robot!"<<endl;

    while(ros::ok()){
        //play
        player->play();
    }
    ros::spinOnce();
    return 0;
}