#ifndef _IIWA_MANIPULATION_H
#define _IIWA_MANIPULATION_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

#include <moveit/planning_interface/planning_interface.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <iwtros_goal/schunk_gripper.h> 
#include <iwtros_msgs/kukaControl.h>
#include <iwtros_msgs/plcControl.h>

namespace rvt = rviz_visual_tools;

namespace iwtros{
    class iiwaMove : public schunkGripper
    {
    private:
        ros::NodeHandle _nh;
        const robot_state::JointModelGroup * joint_model_group;
        ros::Publisher _plcPub;
        ros::Subscriber _plcSub;
        bool _initialized = false;
        // moveit parameters
        /** ToDo:
         * Currently parameters are hard coded.
         * change this in the future
         */
        std::string PLANNER_ID;
        std::string PLANNING_GROUP;
        std::string REFERENCE_FRAME;
        std::string EE_FRAME;
        double velocityScalling;
        double accelerationScalling;
        geometry_msgs::PoseStamped conveyor_pose, DHBW_pose, home_pose;
        iwtros_msgs::plcControl _plcSubscriberControl; 
        iwtros_msgs::kukaControl _plcKUKA; 

    public:
        iiwaMove(ros::NodeHandle nh);
        ~iiwaMove();
        void _loadParam();
        void init(ros::NodeHandle nh);

        /** PLC Control Callback*/ 
        void plcCallback(const iwtros_msgs::plcControl::ConstPtr& data);

        /** Return geometry pose from given poisition values*/
        geometry_msgs::PoseStamped generatePose(double x, double y, double z,
                                                double roll, double pitch, double yaw,
                                                std::string base_link);

        /** Genarate Motion contraints for Pilz industrial motion*/
        void motionContraints(const geometry_msgs::PoseStamped pose, 
                                moveit::planning_interface::MoveGroupInterface &move_group);
        
        /** Motion execution pipe line */
        void motionExecution(const geometry_msgs::PoseStamped pose, 
                                moveit::planning_interface::MoveGroupInterface &move_group);
        
        /** Pick and Place Pipeline */
        void pnpPipeLine(geometry_msgs::PoseStamped pick,
                        geometry_msgs::PoseStamped place,
                        const double offset, 
                        moveit::planning_interface::MoveGroupInterface &move_group);

        /** Rviz visual marker*/
        void visualMarkers(const geometry_msgs::PoseStamped target_pose,
                            moveit::planning_interface::MoveGroupInterface::Plan plan);
        /** Update KUKA goals */
        void poseUpdate();
        /** Main Execution */
        void run();
        void _ctrl_loop(moveit::planning_interface::MoveGroupInterface &move_group);

    };
}


#endif // !_IIWA_MANIPULATION_H
