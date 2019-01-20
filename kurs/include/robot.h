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
#include <cmath>
#include <map>
#include <random>

using namespace std;

enum orientation { FORWARD, BACK, RIGHT, LEFT };

class Robot{
protected:
    ros::Publisher gazeboModelStatePublisher;
    string pathToModel;
    string modelDescription;
    string modelName;
    geometry_msgs::Pose pose;
    int fps = 15;
    orientation robotsOrientation; 
    double angle;
    ros::Rate* rate;
    ros::Rate* halfRate;
    ros::ServiceClient modelsPoseClient;

    int team;
    int total;
    int number;
    string type;
    double goalx, goaly;

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
        uniform_real_distribution<double> interval(20, 60);
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
        this->type = type;
        this->number = number;
        pose.position.x=x;
        pose.position.y=y;
        robotsOrientation = FORWARD;
        if(team==1){
            goalx = 0;
            goaly = -10;
        } else{
            goalx = 0;
            goaly = 10;
        }

        modelsPoseClient = GazeboService::getInstance().getGazeboModelPoseClient();

        rate = new ros::Rate(fps);
        halfRate = new ros::Rate(fps/2);
        updateSpeed();
        gazeboModelStatePublisher = GazeboService::getInstance().getModelStatePublisher();
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
        delete rate;
        delete halfRate;
    }


    /**
     * gets transforms to all robots and ball
    */
    Transforms getTransofrms(const string& objectName){
        Transforms transforms;
        cout<<"In get transform, model name is "<<modelName<<endl;

        gazebo_msgs::GetModelState modelState;
        
        //получить позицию мяча
        modelState.request.model_name = "ball";
        if(modelsPoseClient.call(modelState)){
            transforms.addBall(modelState.response.pose);
        }
        //своей команды
        for (int i = 1; i <= total/2; i++){
            modelState.request.model_name = "player1"+to_string(i);
            if(modelsPoseClient.call(modelState)){
                transforms.addToTeam1(modelState.response.pose);
            }
        }
        //противника
        for (int i = 1; i <= total/2; i++){
            modelState.request.model_name = "player2"+to_string(i);
            if(modelsPoseClient.call(modelState)){
                transforms.addToTeam2(modelState.response.pose);
            }
        }
        //своего вратаря
        modelState.request.model_name = "goalkeeper11";
        if(modelsPoseClient.call(modelState)){
            transforms.addToGoalkeepers(modelState.response.pose);
        }
        //чужого вратаря
        modelState.request.model_name = "goalkeeper21";
        if(modelsPoseClient.call(modelState)){
            transforms.addToGoalkeepers(modelState.response.pose);
        }
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
            double bx = transforms.getBall().position.x;
            double by = transforms.getBall().position.y;
            cout<<"Ball's position is "<<bx<<" "<<by<<endl;
            if(isnan(bx) || isnan(by)) return;
            //едем пинать мяч
            //считаем уравнение прямой между мячом и воротами
            
            double a, b;
            if(bx == 0) a = 1;
            else a = (by - goaly)/(bx - goalx);
            b = goaly - a*goalx;
            double dx, dy;

            if(team == 1) dx = bx + 0.1;
            else dx = bx - 0.1;            
            dy = a*dx + b;

            cout<<"bx "<<bx <<" by "<<by<<endl;
            cout<<"goalx "<<goalx<<" goaly "<<goaly<<endl;
            cout<<"a "<<a<<" b "<<b<<endl;
            cout<<"dx "<<dx <<" dy "<<dy<<endl;

            moveTo(dx, dy);

            if(team == 1) dx = bx - 0.1;
            else dx = bx + 0.1;
            dy = a*dx + b;
            moveTo(dx, dy);
        }
        else {
            cout<<"moving to random place!"<<endl;
            moveTo(intervalx(rd), intervaly(rd));
        }
    }

    void moveTo(double x, double y){
        random_device rd;
        uniform_real_distribution<double> intervalx(-10.0, 10.0);
        uniform_real_distribution<double> intervaly(-9.0, 9.0);

        cout<<this->number<<endl;
        if(this->type=="player"){
        if(this->number==1){
            if(x > -3) x = -3;            
        } else if(this->number ==2){
            if(x > 2) x = 2;
            if(x < -3) x = -2.8;
        }
        else{
            if(x<2) x = 2.2;
        }}
        if(this->type=="player" &&  abs(abs(pose.position.x) - abs(x)) <0.5 && abs(abs(pose.position.y) - abs(y)) <0.5 ){
            // moveTo(intervalx(rd), intervaly(rd));
            return;
        }
        updateSpeed();
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        // std::cout<<"robot's pos " << pose.position<<std::endl;
        // std::cout<<"before turn robot's or " << pose.orientation<<std::endl;
        std::cout<<"required pos:"<<x<<";"<<y<<std::endl;
        // std::cout<<"-----------------------------------"<<std::endl;
        double dx, dy;
        int signX, signY;   
        bool turned = false;  
        // std::cout<<"robots orientation: "<<robotsOrientation<<std::endl;   
        if(x > pose.position.x) {
            // std::cout<<"my x "<< pose.position.x<<" required x "<<x<<std::endl;
            dx = (x - pose.position.x);
            signX = 1;
            // turned = _turn(FORWARD);
        }
        else {
            dx = (pose.position.x - x);
            signX = -1;
            // turned = _turn(BACK);
        }
        if(y > pose.position.y) {
            dy = (y - pose.position.y);
            signY = 1;
            // turned = _turn(LEFT);
        }
        else {
            dy = (pose.position.y - y);
            signY = -1;
            // turned = _turn(RIGHT);
        }
        dx /= fps;
        // std::cout<<"robot's dx = " << dx <<std::endl;
        dy /= fps;
        // std::cout<<"robot's dy = " << dy <<std::endl;
        // std::cout<<"-----------------------------------"<<std::endl;
        // std::cout<<"after turn before move robot's or " << pose.orientation<<std::endl;
        
        turnRobot(x, y);
        for (int i = 0; i < fps+1; i++){
            pose.position.x += signX * dx;
            pose.position.y += signY * dy;
            // std::cout<<"Moving to "<<pose.position<<std::endl;
            modelState.pose =pose;
            gazeboModelStatePublisher.publish(modelState);
            if(turned) rate->sleep();
            else halfRate->sleep();
        }
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

    void turnRobot(double moveToX, double moveToY) {
        double width = moveToX - pose.position.x;
        double height = moveToY - pose.position.y;
        double lenght = sqrt(pow(width, 2) + pow(height, 2));
        double angle;

        static double lastAngle = 0;

        if(abs(width) < 1e-5) angle = asin(height/lenght);
        else angle = acos(width/lenght);
        angle -= lastAngle;
        lastAngle = angle;
        _turn(angle, true);
    }

    void _turn(double angle, bool clockwise)
    {
        gazebo_msgs::ModelState modelState;
        modelState.model_name = this->modelName;
        tf::Quaternion tfq;
        geometry_msgs::Quaternion q;
        
        std::cout<<angle<<"and my angle is "<<this->angle<<std::endl;
        ros::Rate turnRate(60);

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
            // turnRate.sleep();
        }
        // std::cout<<"final or"<<q<<std::endl;
}
};