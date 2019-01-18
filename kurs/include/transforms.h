#include <vector>

using namespace std;
class Transforms{
protected:
    vector<geometry_msgs::Pose> team1;
    vector<geometry_msgs::Pose> team2;
    geometry_msgs::Pose ball;
    vector<geometry_msgs::Pose> goalkeepers;

    public:
    void addToTeam1(geometry_msgs::Pose transform){
        team1.push_back(transform);
    }

    void addToTeam2(geometry_msgs::Pose transform){
        team2.push_back(transform);
    }

    void addToGoalkeepers(geometry_msgs::Pose transform){
        goalkeepers.push_back(transform);
    }

    void addBall(geometry_msgs::Pose transform){
        ball = transform;
    }

    vector<geometry_msgs::Pose> getTeam1(){
        return team1;
    }

    vector<geometry_msgs::Pose> getTeam2(){
        return team2;
    }

    vector<geometry_msgs::Pose> getGoalkeepers(){
        return goalkeepers;
    }

    geometry_msgs::Pose getBall(){
        return ball;
    }

};