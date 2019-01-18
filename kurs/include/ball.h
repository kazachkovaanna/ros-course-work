#pragma once

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "GazeboService.h"
#include "tf/LinearMath/Quaternion.h"
#include "tf/transform_datatypes.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include <cmath>

using namespace std;

class Ball{
protected:
    ros::Publisher gazeboModelStatePublisher;
    ros::Subscriber ballStateSubscriber;
    ros::ServiceClient ballPoseClient;
    
    string pathToModel;
    string modelDescription;
    string modelName;
    geometry_msgs::Pose pose;
    static const int fps = 15;
    // orientation robotsOrientation; 
    double angle;
    ros::Rate* rate;
    ros::Rate* halfRate;
    tf::TransformListener* tfListener;
    tf::TransformBroadcaster* tfBroadcaster;
    tf::StampedTransform* robotStampedTransform;
    tf::Transform* robotTransform;

    ros::Subscriber gazeboBallStateSubscriber;
protected:


    void spawnModel(){
        gazebo_msgs::SpawnModel srv;
        srv.request.model_xml = modelDescription;
        srv.request.model_name = modelName;
        srv.request.initial_pose = pose;
        GazeboService::getInstance().submitSpawnMessage(srv);
        std::cout<<"Spawned robot "<<modelName<<std::endl;
    }

public:
    Ball(){
        // double x, double y, const string& pathToModel, const string& modelName
        gazeboBallStateSubscriber = GazeboService::getInstance().getModelStateSubscriber(listenState);
        

        this->modelName = "ball";
        this->pathToModel = "/home/user/Projects/ros/catkin_ws/src/ros-course-work/kurs/models/ball/model.sdf";
        pose.position.x=1.0;
        pose.position.y=-2.0;
        tf::TransformListener tfListener;
        tf::TransformBroadcaster tfBroadcaster;
        tf::StampedTransform ballStampedTransform;
        tf::Transform transform;
        // geometry_msgs::Pose ballPose(msg.pose);
        transform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        transform.setRotation(tf::Quaternion(0, 0, 0, 1));
        cout<<"Time is "<<ros::Time::now().nsec<<endl;
        tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/ball"));
        cout<<"I have sent a transform!"<<endl;
        // robotsOrientation = FORWARD;
        rate = new ros::Rate(fps);
        halfRate = new ros::Rate(fps/2);
        ballPoseClient = GazeboService::getInstance().getGazeboModelPoseClient();
        gazeboModelStatePublisher = GazeboService::getInstance().getModelStatePublisher();
        ifstream fin(this->pathToModel);
        string lineBuffer;
        while(getline(fin, lineBuffer)) {
            modelDescription += lineBuffer+ '\n';
        }
        fin.close();
        std::cout<<"Created ball --> "<<modelName<<std::endl;
        spawnModel();
    }

    ~Ball(){
        delete tfListener;
        delete tfBroadcaster;
        delete robotStampedTransform;
        delete robotTransform;
        delete rate;
        delete halfRate;
    }

    static void listenState(const gazebo_msgs::ModelState& msg){
        static ros::Publisher ballPositionPublisher = GazeboService::getInstance().getBallPosePublisher();
        static tf::TransformListener tfListener;
        static tf::TransformBroadcaster tfBroadcaster;
        static tf::StampedTransform ballStampedTransform;
        static tf::Transform transform;
        cout<<msg.model_name<<endl;
        
        if(msg.model_name == "ball"){
            geometry_msgs::Pose ballPose(msg.pose);
            cout<<ballPose.position.x<<","<<ballPose.position.y<<endl;
            transform.setOrigin(tf::Vector3(ballPose.position.x, ballPose.position.y, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            tfBroadcaster.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/world", "/ball"));
        }
    }

    void beABall(){
        static ros::Publisher ballPositionPublisher = GazeboService::getInstance().getBallPosePublisher();
        static tf::TransformListener tfListener;
        static tf::TransformBroadcaster tfBroadcaster;
        static tf::StampedTransform ballStampedTransform;
        static tf::Transform transform;
        gazebo_msgs::GetModelState modelState;
        modelState.request.model_name = this->modelName;
        if(ballPoseClient.call(modelState)){
            geometry_msgs::Pose ballPose(modelState.response.pose);
            if(ballPose.position.x > 10 || ballPose.position.x < -10 ||
                ballPose.position.y > 10 || ballPose.position.y < -10
            ){
                // gazebo_msgs::ModelState modelState;
                // modelState.model_name = this->modelName;
                // ballPose.position.x = 0;
                // ballPose.position.y = 0;
                // modelState.pose = ballPose;
                // gazeboModelStatePublisher.publish(modelState);
                // spawnModel();
            }
            cout<<ballPose.position.x<<","<<ballPose.position.y<<endl;
            transform.setOrigin(tf::Vector3(ballPose.position.x, ballPose.position.y, 0.0));
            transform.setRotation(tf::Quaternion(0, 0, 0, 1));
            ros::Time now = ros::Time::now();
            tfBroadcaster.sendTransform(tf::StampedTransform(transform, now, "/world", "/ball"));
            cout<<"Sent time "<<now<<endl;
        }
    }

    void moveTo(double x, double y){
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        std::cout<<"robot's pos " << pose.position<<std::endl;
        std::cout<<"before turn robot's or " << pose.orientation<<std::endl;
        std::cout<<"required pos:"<<x<<";"<<y<<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
        double dx, dy;
        int signX, signY;   
        if(x > pose.position.x) {
            std::cout<<"my x "<< pose.position.x<<" required x "<<x<<std::endl;
            dx = (x - pose.position.x);
            signX = 1;
        }
        else {
            dx = (pose.position.x - x);
            signX = -1;
        }
        if(y > pose.position.y) {
            dy = (y - pose.position.y);
            signY = 1;
        }
        else {
            dy = (pose.position.y - y);
            signY = -1;
        }
        dx /= fps;
        // std::cout<<"robot's dx = " << dx <<std::endl;
        dy /= fps;
        // std::cout<<"robot's dy = " << dy <<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"after turn before move robot's or " << pose.orientation<<std::endl;
        
        pose.position.x = x;
        pose.position.y=y;
        // for (int i = 0; i < fps+1; i++){
        //     robotTransform->setOrigin(tf::Vector3(x, y, 0.0));
        //     robotTransform->setRotation(tf::Quaternion(0, 0, 0, 1));
        //     tfBroadcaster->sendTransform(tf::StampedTransform(
        //         *robotStampedTransform, ros::Time::now(), "/world", "/ball"));
        //     pose.position.x += signX * dx;
        //     pose.position.y += signY * dy;
        //     // std::cout<<"Moving to "<<pose.position<<std::endl;
        //     modelState.pose =pose;
        //     gazeboModelStatePublisher.publish(modelState);
        //     rate->sleep();
        // }
        modelState.pose =pose;
        gazeboModelStatePublisher.publish(modelState);
        // spawnModel();
        robotTransform->setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        robotTransform->setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster->sendTransform(tf::StampedTransform(*robotTransform, ros::Time::now(), "/world", "/"+modelName));
    }


};


// s