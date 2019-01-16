#include <vector>
#include "tf/transform_datatypes.h"

using namespace std;
class Transforms{
protected:
    vector<tf::StampedTransform> team1;
    vector<tf::StampedTransform> team2;
    tf::StampedTransform ball;
    vector<tf::StampedTransform> goalkeepers;

    public:
    void addToTeam1(tf::StampedTransform transform){
        team1.push_back(transform);
    }

    void addToTeam2(tf::StampedTransform transform){
        team2.push_back(transform);
    }

    void addToGoalkeepers(tf::StampedTransform transform){
        goalkeepers.push_back(transform);
    }

    void addBall(tf::StampedTransform transform){
        ball = transform;
    }

    vector<tf::StampedTransform> getTeam1(){
        return team1;
    }

    vector<tf::StampedTransform> getTeam2(){
        return team2;
    }

    vector<tf::StampedTransform> getGoalkeepers(){
        return goalkeepers;
    }

    tf::StampedTransform getBall(){
        return ball;
    }

};