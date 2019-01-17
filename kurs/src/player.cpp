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
    ros::Rate rate(0.5);
    string name;
    int team, number, total;
    double x,y;
    ros::param::get("~team",team);
    ros::param::get("~number",number);
    ros::param::get("~total",total);
	ros::param::get("~x", x);
    ros::param::get("~y", y);

    name = "player"+team+number;

    //creating transform to send this player's pose

    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform rescuer_transform;
    tf::Transform transform;

    //make sure that all nodes now have my transform
    transform.setOrigin(tf::Vector3(x, y, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    for(int i = 0; i < 10; i++){
    
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", name));
        rate.sleep();
    }

    //make sure that player now gets a position of a ball
    while(true){
        try
            {
                tf_listener.waitForTransform(name, "ball", ros::Time(0), ros::Duration(0.5));
                tf_listener.lookupTransform(name, "ball", ros::Time(0), rescuer_transform);
                break;
            }
            catch (tf::TransformException &ex)
            {
                rate.sleep();
                continue;   
            }
    }


    //spawn model to gazebo
    Robot* player;
    player = new Robot(x, y, team, number,total, "player"); 
    random_device rd;
    uniform_real_distribution<double> interval(-10.0, 10.0);
    rate.sleep();
    cout<<"Created robot!"<<endl;

    while(ros::ok()){
        //play
        // tf::StampedTransform ballToRobotTransform = player->getTransofrms();
        // player->moveTo(ballToRobotTransform.getOrigin().getX(), ballToRobotTransform.getOrigin().getY());
        player->play();
    }
    ros::spinOnce();
    return 0;
}