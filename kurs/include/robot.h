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
#include <map>
#include <random>

using namespace std;

enum orientation { FORWARD, BACK, RIGHT, LEFT };

class Robot{
protected:
    ros::Publisher gazeboModelStatePublisher;
    // ros::Subscriber ballPoseSubscriber;
    string pathToModel;
    string modelDescription;
    string modelName;
    geometry_msgs::Pose pose;
    int fps = 15;
    orientation robotsOrientation; 
    double angle;
    ros::Rate* rate;
    ros::Rate* halfRate;
    tf::TransformListener* tfListener;
    tf::TransformBroadcaster* tfBroadcaster;
    tf::StampedTransform* robotStampedTransform;
    tf::Transform* robotTransform;
    map<string, tf::StampedTransform> robotsPoses;

    void spawnModel(){
        gazebo_msgs::SpawnModel srv;
        srv.request.model_xml = modelDescription;
        srv.request.model_name = modelName;
        srv.request.initial_pose = pose;
        GazeboService::getInstance().submitSpawnMessage(srv);
        std::cout<<"Spawned robot "<<modelName<<std::endl;
    }

    void updateSpeed(){
        random_device rd;
        uniform_real_distribution<double> interval(10, 20);
        fps = interval(rd);
        delete rate;
        delete halfRate;
        rate = new ros::Rate(fps);
        halfRate = new ros::Rate(fps/2);
    }

public:
    Robot(double x, double y, const string& pathToModel, const string& modelName){
        this->modelName = modelName;
        this->pathToModel = pathToModel;
        pose.position.x=x;
        pose.position.y=y;
        tfListener = new tf::TransformListener();
        tfBroadcaster = new tf::TransformBroadcaster();
        robotStampedTransform = new tf::StampedTransform();
        robotTransform = new tf::Transform();         
        robotsOrientation = FORWARD;

        rate = new ros::Rate(fps);
        halfRate = new ros::Rate(fps/2);
        updateSpeed();
        cout<<"published to tf my pose x=" <<pose.position.x<<" y="<<pose.position.y<<endl;
        robotTransform->setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        robotTransform->setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster->sendTransform(tf::StampedTransform(*robotTransform, ros::Time::now(), "/world", "/"+modelName));
        gazeboModelStatePublisher = GazeboService::getInstance().getModelStatePublisher();
        // ballPoseSubscriber = GazeboService::getInstance().getBallPoseSubscriver();
        ifstream fin(this->pathToModel);
        string lineBuffer;
        while(getline(fin, lineBuffer)) {
            modelDescription += lineBuffer+ '\n';
        }
        fin.close();
        std::cout<<"Created robot "<<modelName<<std::endl;
        spawnModel();
    }


    ~Robot(){
        delete tfListener;
        delete tfBroadcaster;
        delete robotStampedTransform;
        delete robotTransform;
        delete rate;
        delete halfRate;
    }

    tf::StampedTransform getTransofrms(const string& objectName){
        // robotsPoses.
        tf::StampedTransform ballRobotTransform;
        cout<<"In get transform, model name is "<<modelName<<endl;

        // cout<<"ballTr "<<endl;
        while(true){
            try{
                cout<<"trying wait"<<endl;
                // tfListener->waitForTransform("/"+modelName, "/ball", ros::Time(0), ros::Duration(1));
                tfListener->waitForTransform("/ball", objectName, ros::Time(0), ros::Duration(1));
                cout<<"trying lookup"<<endl;
                // tfListener->lookupTransform("/"+modelName, "/ball", ros::Time(0), ballRobotTransform);
                tfListener->lookupTransform("/ball", objectName, ros::Time(0), ballRobotTransform);
                cout<<"Done!"<<endl;
                break;
            }
            catch (tf::TransformException &ex)
                {
                    ROS_ERROR("%s",ex.what());
                    cout<<"Not ready yet..."<<endl;
                    rate->sleep();
                    sleep(1);
                    continue;
                }
        }
        return ballRobotTransform;
    }

    void play(){
        cout<<"playing!"<<endl;
        random_device rd;
        uniform_real_distribution<double> interval(0, 3);
        uniform_real_distribution<double> intervalx(-10.0, 10.0);
        uniform_real_distribution<double> intervaly(-9.0, 9.0);
        if(interval(rd)>1.5){
            tf::StampedTransform balltf= getTransofrms("ball");
            double bx = balltf.getOrigin().getX();
            double by = balltf.getOrigin().getY();
            
            for(int i = 0; i <3; i++){
                
            }
            moveTo(bx, by);
        }
        else moveTo(intervalx(rd), intervaly(rd));
    }

    void moveTo(double x, double y){
        updateSpeed();
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        std::cout<<"robot's pos " << pose.position<<std::endl;
        std::cout<<"before turn robot's or " << pose.orientation<<std::endl;
        std::cout<<"required pos:"<<x<<";"<<y<<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
        double dx, dy;
        int signX, signY;   
        bool turned = false;  
        std::cout<<"robots orientation: "<<robotsOrientation<<std::endl;   
        if(x > pose.position.x) {
            std::cout<<"my x "<< pose.position.x<<" required x "<<x<<std::endl;
            dx = (x - pose.position.x);
            signX = 1;
            turned = _turn(FORWARD);
        }
        else {
            dx = (pose.position.x - x);
            signX = -1;
            turned = _turn(BACK);
        }
        if(y > pose.position.y) {
            dy = (y - pose.position.y);
            signY = 1;
            turned = _turn(LEFT);
        }
        else {
            dy = (pose.position.y - y);
            signY = -1;
            turned = _turn(RIGHT);
        }
        dx /= fps;
        // std::cout<<"robot's dx = " << dx <<std::endl;
        dy /= fps;
        // std::cout<<"robot's dy = " << dy <<std::endl;
        std::cout<<"-----------------------------------"<<std::endl;
        std::cout<<"after turn before move robot's or " << pose.orientation<<std::endl;
        
        for (int i = 0; i < fps+1; i++){
            robotTransform->setOrigin(tf::Vector3(x, y, 0.0));
            robotTransform->setRotation(tf::Quaternion(0, 0, 0, 1));
            tfBroadcaster->sendTransform(tf::StampedTransform(
                *robotStampedTransform, ros::Time::now(), "/world", "/"+modelName));
            pose.position.x += signX * dx;
            pose.position.y += signY * dy;
            // std::cout<<"Moving to "<<pose.position<<std::endl;
            modelState.pose =pose;
            gazeboModelStatePublisher.publish(modelState);
            if(turned) rate->sleep();
            else halfRate->sleep();
        }
        robotTransform->setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        robotTransform->setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster->sendTransform(tf::StampedTransform(*robotTransform, ros::Time::now(), "/world", "/"+modelName));
    }

protected:
    bool _turn(orientation to)
    {
        if (to == robotsOrientation)
        {
            return false;
        }

        switch(to)
        {
        case FORWARD:
            switch(robotsOrientation)
            {
            case BACK:
                _turnAround();
                break;
            case RIGHT:
                _turnConterclockwise();
                break;
            case LEFT:
                _turnClockwise();
                break;
            }
            break;
        case BACK:
            switch(robotsOrientation)
            {
            case FORWARD:
                _turnAround();
                break;
            case RIGHT:
                _turnClockwise();
                break;
            case LEFT:
                _turnConterclockwise();
                break;
            }
            break;
        case RIGHT:
            switch(robotsOrientation)
            {
            case FORWARD:
                _turnClockwise();
                break;
            case BACK:
                _turnConterclockwise();
                break;
            case LEFT:
                _turnAround();
                break;
            }
            break;
        case LEFT:
            switch(robotsOrientation)
            {
            case FORWARD:
                _turnConterclockwise();
                break;
            case BACK:
                _turnClockwise();
                break;
            case RIGHT:
                _turnAround();
                break;
            }
            break;
        }

        robotsOrientation = to;
        return true;
    }

    void _turnClockwise()
    {
        _turn(M_PI, true);
    }

    void _turnConterclockwise()
    {
        _turn(M_PI, false);
    }

    void _turnAround()
    {
        _turn(2 * M_PI, false);
    }

    void _turn(double angle, bool clockwise)
    {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        tf::Quaternion tfq;
        geometry_msgs::Quaternion q;
        
        std::cout<<angle<<"and my angle is "<<this->angle<<std::endl;

        double dangle = angle / fps;
        std::cout<<"dangle "<<dangle<<std::endl;

        for (int i = 0; i < fps; i++)
        {
            if (clockwise)
                this->angle -= dangle;
            else
                this->angle += dangle;
            std::cout<<"now angle is "<<this->angle<<std::endl;
            tfq.setRPY(0, 0, this->angle);
            tf::quaternionTFToMsg(tfq, q);
            std::cout<<"computed or"<<q<<std::endl;
            pose.orientation = q;
            modelState.pose =pose;
            gazeboModelStatePublisher.publish(modelState);
            rate->sleep();
        }
        std::cout<<"final or"<<q<<std::endl;
}
};