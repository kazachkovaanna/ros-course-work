#include "ros/ros.h"
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "string.h"
#include "ball.h"
#include <random>

int main(int argc, char** argv) {
 
    ros::init(argc, argv, "ball");
    GazeboService::getInstance();
    ros::service::waitForService("gazebo/spawn_sdf_model"); 
    string name;
    int team;
    double x,y;
    ros::Rate rate(0.5);


    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;
    tf::StampedTransform rescuer_transform;
    tf::Transform transform;

    transform.setOrigin(tf::Vector3(0, 0, 0.0));
    transform.setRotation(tf::Quaternion(0, 0, 0, 1));
    tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/ball"));

    for(int i = 0; i<11; i++){
        tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/ball"));
        rate.sleep();
    }
    // ros::param::get("~name", name);
    // ros::param::get("~team",team);
	// ros::param::get("~x", x);
    // ros::param::get("~y", y);
    
    // ros::Publisher publisher_found_topic = n.advertise<std_msgs::String>("found_topic", 10);
    
    std::cout<<"name "<<name<<" team "<<team<<" x "<<x<<" y "<<y<<std::endl;
    Ball *ball = new Ball();

    // Robot* ball = new Robot(x, y, "/home/user/Projects/ros/catkin_ws/src/kurs/models/ball/model.sdf", name); 
  
    random_device rd;
    uniform_real_distribution<double> interval(-10.0, 10.0);
    tf::TransformListener tfListener;
    tf::TransformBroadcaster tfBroadcaster;
    tf::StampedTransform ballStampedTransform;
    // tf::Transform transform;
        // cout<<msg.model_name<<endl;
        
            transform.setOrigin(tf::Vector3(0, 0, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/ball"));
        

    while(ros::ok()){
        //be a ball

        for(int i = 0; i<10; i++){
            // if(i==0)tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/ball"));
            double nx = interval(rd);
            double ny = interval(rd);
            ball->moveTo(nx, ny);
            transform.setOrigin(tf::Vector3(nx, ny, 0.0));
            tf_broadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "/ball"));
            rate.sleep();
        }
    }
    ros::spinOnce();
    return 0;

    
}
