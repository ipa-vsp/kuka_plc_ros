#include <iwtros_goal/iiwa_manipulation.h>
#include <thread.h>

iwtros::iiwaMove::iiwaMove(ros::NodeHandle nh) : schunkGripper(nh), _nh(nh){
        // Initialize the move_group
        // joint model group
        // visual markers
        init(_nh);
}

iwtros::iiwaMove::~iiwaMove(){}

void iwtros::iiwaMove::_loadParam(){
        PLANNER_ID = "PTP";
        PLANNING_GROUP = "iiwa_arm";
        REFERENCE_FRAME = "iiwa_link_0";
        EE_FRAME = "iiwa_link_ee";
        velocityScalling = 0.5;
        accelerationScalling = 0.5;
}

void iwtros::iiwaMove::init(ros::NodeHandle nh){
        _loadParam();
        _initialized = true;
}

geometry_msgs::PoseStamped iwtros::iiwaMove::generatePose(double x, double y, double z,
                                                double roll, double pitch, double yaw,
                                                std::string base_link){
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = base_link.c_str();
        pose.header.stamp = ros::Time::now() + ros::Duration(2.1);
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = z;
        tf2::Quaternion q;
        q.setRPY(roll, pitch, yaw);
        pose.pose.orientation.x = q.x();
        pose.pose.orientation.y = q.y();
        pose.pose.orientation.z = q.z();
        pose.pose.orientation.w = q.w();
        return pose;
}

void iwtros::iiwaMove::run(){
        if(!_initialized){
                ROS_ERROR("IIWA Motion initialization is failed");
                return;
        }
        moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
        move_group.setPlannerId(PLANNER_ID);
        move_group.setMaxVelocityScalingFactor(velocityScalling);
        move_group.setMaxAccelerationScalingFactor(accelerationScalling);
        move_group.setPoseReferenceFrame(REFERENCE_FRAME);
        move_group.setEndEffectorLink(EE_FRAME);

        std::thread t1(&iiwaMove::_ctrl_loop, this, move_group);
        t1.join();
}

void iwtros::iiwaMove::_ctrl_loop(moveit::planning_interface::MoveGroupInterface &move_group){
        pick_pose = generatePose(1, 1, 1 , 1, 1 , 1, "iiwa_link_0");
}