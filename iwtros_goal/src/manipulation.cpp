#include <iwtros_goal/iiwa_manipulation.h>

iwtros::iiwaMove::iiwaMove(ros::NodeHandle nh) : schunkGripper(nh), _nh(nh){
        // Initialize the move_group
        // joint model group
        // visual markers
}

iwtros::iiwaMove::~iiwaMove(){}