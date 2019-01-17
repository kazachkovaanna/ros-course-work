#pragma once

#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include <fstream>
#include "GazeboService.h"
#include "transforms.h"
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
    // tf::Transform* robotTransform;
    map<string, tf::StampedTransform> robotsPoses;
    ros::ServiceClient modelsPoseClient;

    int team;
    int total;

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
    Robot(double x, double y, int team, int number, int total, string type){
        this->modelName = type+to_string(team)+to_string(number);
        this->pathToModel = "/home/user/Projects/ros/catkin_ws/src/ros-course-work/kurs/models/player"+to_string(team)+"/model.sdf";
        this->total = total;
        pose.position.x=x;
        pose.position.y=y;
        tfListener = new tf::TransformListener(ros::Duration(170.0));
        tfBroadcaster = new tf::TransformBroadcaster();
        robotStampedTransform = new tf::StampedTransform();
        tf::Transform robotTransform;
        robotsOrientation = FORWARD;

        modelsPoseClient = GazeboService::getInstance().getGazeboModelPoseClient();

        rate = new ros::Rate(fps);
        halfRate = new ros::Rate(fps/2);
        updateSpeed();
        cout<<"published to tf my pose x=" <<pose.position.x<<" y="<<pose.position.y<<endl;
        robotTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        robotTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster->sendTransform(tf::StampedTransform(robotTransform, ros::Time::now(), "/world", "/"+modelName));
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
        // delete robotTransform;
        delete rate;
        delete halfRate;
    }


    /**
     * gets transforms to all robots and ball
    */
    Transforms getTransofrms(const string& objectName){
        Transforms transforms;

        // for(int i  = 1; i<= total/2; i++){
        //     tf::StampedTransform transform;
        //     while(true){
        //     try{
        //         cout<<"trying wait for player1"<<i<<endl;
        //         tfListener->waitForTransform("player1"+to_string(i), modelName,ros::Time(0), ros::Duration(1));
        //         cout<<"trying lookup"<<endl;
        //         tfListener->lookupTransform("player1"+to_string(i), modelName,ros::Time(0), transform);
        //         cout<<"Done!"<<endl;
        //         break;
        //     }
        //     catch (tf::TransformException &ex)
        //         {
        //             ROS_ERROR("%s",ex.what());
        //             cout<<"Not ready yet..."<<endl;
        //             rate->sleep();
        //             sleep(1);
        //             continue;
        //         }
        //     }
        //     transforms.addToTeam1(transform);
        // }
        // for(int i  = 1; i<= total/2; i++){
        //     tf::StampedTransform transform;
        //     while(true){
        //     try{
        //         cout<<"trying wait for player2"<<i<<endl;
        //         tfListener->waitForTransform("player2"+to_string(i), modelName,ros::Time(0), ros::Duration(1));
        //         cout<<"trying lookup"<<endl;
        //         tfListener->lookupTransform("player2"+to_string(i), modelName,ros::Time(0), transform);
        //         cout<<"Done!"<<endl;
        //         break;
        //     }
        //     catch (tf::TransformException &ex)
        //         {
        //             ROS_ERROR("%s",ex.what());
        //             cout<<"Not ready yet..."<<endl;
        //             rate->sleep();
        //             sleep(1);
        //             continue;
        //         }
        //     }
        //     transforms.addToTeam2(transform);
        // }
        // for(int i  = 1; i<= 2; i++){
        //     tf::StampedTransform transform;
        //     while(true){
        //     try{
        //         cout<<"trying wait for player1"<<i<<endl;
        //         tfListener->waitForTransform("goalkeeper"+to_string(i),modelName, ros::Time(0), ros::Duration(1));
        //         cout<<"trying lookup"<<endl;
        //         tfListener->lookupTransform("goalkeeper"+to_string(i),modelName, ros::Time(0), transform);
        //         cout<<"Done!"<<endl;
        //         break;
        //     }
        //     catch (tf::TransformException &ex)
        //         {
        //             ROS_ERROR("%s",ex.what());
        //             cout<<"Not ready yet..."<<endl;
        //             rate->sleep();
        //             sleep(1);
        //             continue;
        //         }
        //     }
        //     transforms.addToGoalkeepers(transform);
        // }

        // robotsPoses.
        tf::StampedTransform ballRobotTransform;
        cout<<"In get transform, model name is "<<modelName<<endl;

        // cout<<"ballTr "<<endl;
        while(true){
            try{
                cout<<"trying wait"<<endl;
                tfListener->waitForTransform("/"+modelName, "/ball", ros::Time::now(), ros::Duration(40));
                // tfListener->waitForTransform("/ball", modelName, ros::Time(0), ros::Duration(1));
                cout<<"trying lookup"<<endl;
                tfListener->lookupTransform("/"+modelName, "/ball", ros::Time(0), ballRobotTransform);
                // tfListener->lookupTransform("/ball", modelName, ros::Time(0), ballRobotTransform);
                cout<<"Done! Ball is at:"<<endl;
                cout<<ballRobotTransform.getOrigin().getX()<<";"<<ballRobotTransform.getOrigin().getY()<<endl;
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
        transforms.addBall(ballRobotTransform);
        return transforms;
    }

    void play(){
        cout<<"playing!"<<endl;
        Transforms transforms = getTransofrms("ball");

        //если робот далеко от мяча

        //если робот ведет мяч


        random_device rd;
        uniform_real_distribution<double> interval(0, 3);
        uniform_real_distribution<double> intervalx(-10.0, 10.0);
        uniform_real_distribution<double> intervaly(-9.0, 9.0);
        if(interval(rd)>1.0){
            double bx = transforms.getBall().getOrigin().getX();
            double by = transforms.getBall().getOrigin().getY();
            cout<<"Ball's position is "<<bx<<" "<<by<<endl;
            

            moveTo(bx, by);
        }
        else {
            cout<<"moving to random place!"<<endl;
            moveTo(intervalx(rd), intervaly(rd));
        }
    }

    void moveTo(double x, double y){
        updateSpeed();
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        // std::cout<<"robot's pos " << pose.position<<std::endl;
        // std::cout<<"before turn robot's or " << pose.orientation<<std::endl;
        // std::cout<<"required pos:"<<x<<";"<<y<<std::endl;
        // std::cout<<"-----------------------------------"<<std::endl;
        double dx, dy;
        int signX, signY;   
        bool turned = false;  
        // std::cout<<"robots orientation: "<<robotsOrientation<<std::endl;   
        if(x > pose.position.x) {
            // std::cout<<"my x "<< pose.position.x<<" required x "<<x<<std::endl;
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
        // std::cout<<"-----------------------------------"<<std::endl;
        // std::cout<<"after turn before move robot's or " << pose.orientation<<std::endl;
        
        for (int i = 0; i < fps+1; i++){
            tf::Transform robotTransform;
            robotTransform.setOrigin(tf::Vector3(x, y, 0.0));
            robotTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
            cout<<"trying to send Transform"<<endl;
            tfBroadcaster->sendTransform(tf::StampedTransform(
                robotTransform, ros::Time::now(), "/world", "/"+modelName));
            pose.position.x += signX * dx;
            pose.position.y += signY * dy;
            // std::cout<<"Moving to "<<pose.position<<std::endl;
            modelState.pose =pose;
            gazeboModelStatePublisher.publish(modelState);
            if(turned) rate->sleep();
            else halfRate->sleep();
        }
        tf::Transform robotTransform;
        robotTransform.setOrigin(tf::Vector3(pose.position.x, pose.position.y, 0.0));
        robotTransform.setRotation(tf::Quaternion(0, 0, 0, 1));
        tfBroadcaster->sendTransform(tf::StampedTransform(robotTransform, ros::Time::now(), "/world", "/"+modelName));
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
        
        // std::cout<<angle<<"and my angle is "<<this->angle<<std::endl;

        double dangle = angle / fps;
        // std::cout<<"dangle "<<dangle<<std::endl;

        for (int i = 0; i < fps; i++)
        {
            if (clockwise)
                this->angle -= dangle;
            else
                this->angle += dangle;
            // std::cout<<"now angle is "<<this->angle<<std::endl;
            tfq.setRPY(0, 0, this->angle);
            tf::quaternionTFToMsg(tfq, q);
            // std::cout<<"computed or"<<q<<std::endl;
            pose.orientation = q;
            modelState.pose =pose;
            gazeboModelStatePublisher.publish(modelState);
            rate->sleep();
        }
        // std::cout<<"final or"<<q<<std::endl;
}
};