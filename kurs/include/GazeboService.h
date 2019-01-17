#include <ros/ros.h>
#include "gazebo_msgs/SpawnModel.h"
#include "gazebo_msgs/DeleteModel.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/GetModelState.h"
#include <fstream>

class GazeboService {
private:
    ros::NodeHandle nodeHandle;
    ros::ServiceClient addRobotService;

    GazeboService()
    {           
        addRobotService = nodeHandle.serviceClient<gazebo_msgs::SpawnModel>("gazebo/spawn_sdf_model");
    }

public:
    using cameraCallback = void(*)(const std::string &);

    static GazeboService& getInstance(){
        static GazeboService service;
        return service;
    }

    void submitSpawnMessage(gazebo_msgs::SpawnModel spawnMessage){
        addRobotService.call(spawnMessage);
    }
    
    ros::Publisher getModelStatePublisher(){
        return nodeHandle.advertise<gazebo_msgs::ModelState>("gazebo/set_model_state", 10);
    }

    ros::Subscriber getModelStateSubscriber(void(*callback)(const gazebo_msgs::ModelState& msg)){
        return nodeHandle.subscribe("gazebo/get_model_state", 100, callback);
    }

    ros::Publisher getBallPosePublisher(){
        return nodeHandle.advertise<geometry_msgs::Pose>("ball_position", 100);
    }

    ros::ServiceClient getGazeboModelPoseClient(){
        return nodeHandle.serviceClient<gazebo_msgs::GetModelState>("gazebo/get_model_state");
    }

    ros::Subscriber getBallPoseSubscriver(void(*callback)(const geometry_msgs::Pose& msg)){
        return nodeHandle.subscribe("ball_position", 10, callback);
    }

};