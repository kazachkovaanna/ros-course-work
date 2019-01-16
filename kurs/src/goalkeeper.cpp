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
    ros::Rate rate(0.5);

    string name;
    int team, number, total;
    double x,y;
    ros::param::get("~team",team);
    ros::param::get("~number",number);
    ros::param::get("~name", name);
	ros::param::get("~x", x);
    ros::param::get("~y", y);

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform stampedTransform;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    for(int i = 0; i < 10; i++){
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
        rate.sleep();
    }
    while(true){
        try
            {
                tf_listener.waitForTransform(name, "ball", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform(name, "ball", ros::Time(0), stampedTransform);
                break;
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;   
            }
    }
    

    // ros::Publisher publisher_found_topic = n.advertise<std_msgs::String>("found_topic", 10);
    
    std::cout<<"name "<<name<<" team "<<team<<" x "<<x<<" y "<<y<<std::endl;
    Robot* player;
    player = new Robot(x, y, team, number, total, "goalkeeper"); 
  
    random_device rd;
    uniform_real_distribution<double> interval(-2.0, 2.0);

    while(ros::ok()){
        tf::StampedTransform ballToRobotTransform = player->getTransofrms("ball").getBall();
        // player->moveTo(ballToRobotTransform.getOrigin().getX(), ballToRobotTransform.getOrigin().getY());
        if(team==1)
        player->moveTo(ballToRobotTransform.getOrigin().getX(), 10.0);
        else 
        player->moveTo(ballToRobotTransform.getOrigin().getX(), -10.0);
    }
    ros::spinOnce();
    return 0;
}