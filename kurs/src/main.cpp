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
    ros::init(argc, argv, "gaztest");
    ROS_INFO("0");
    GazeboService::getInstance();
    ros::Rate rate(0.5);

    ROS_INFO("1");

    ros::service::waitForService("gazebo/spawn_sdf_model"); 

    ROS_INFO("2");

    Robot* robot = new Robot(0, 0, 1, 1, 2, "player"); 
    Robot* enemyRobot = new Robot(2, 2, 2, 1, 2, "player");
    ROS_INFO("3");
    robot->moveTo(2.0, 2.0);
    ROS_INFO("4");

    random_device rd;
    uniform_real_distribution<double> interval(-10.0, 10.0);
    int i  = 0;

    while(ros::ok()){
        // robot->moveTo(interval(rd), interval(rd));
        // enemyRobot->moveTo(interval(rd), interval(rd));
        cout<<"["<<i<<"] robot.play"<<endl;
        robot->play();
        cout<<"["<<i<<"] enemy.play"<<endl;
        enemyRobot->play();
        rate.sleep();
    }
    ros::spinOnce();
    
    ROS_INFO_STREAM("9. After spinOnce, about to return");

    delete robot;
    return 0;
}