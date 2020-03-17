#include <iwtros_goal/iiwa_manipulation.h>
#include <thread>
#include <iostream>

#include <moveit_msgs/Constraints.h>
#include <shape_msgs/SolidPrimitive.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

iwtros::iiwaMove::iiwaMove(ros::NodeHandle nh, const std::string planning_group) : schunkGripper(nh), _nh(nh), move_group(planning_group){
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
        // ToDo: input array param goals
}

void iwtros::iiwaMove::init(ros::NodeHandle nh){
        _loadParam();
        _plcSub = nh.subscribe<iwtros_msgs::plcControl>("plc_control", 10, boost::bind(&iiwaMove::plcCallback, this, _1));
        _plcPub = nh.advertise<iwtros_msgs::kukaControl>("plc_listner", 10);
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

void iwtros::iiwaMove::poseUpdate(){
        // ToDo: Check the ROS parameter server for new pose
        conveyor_pose = generatePose(0.33, -0.427, 1.222, M_PI, 0 , M_PI/4, "iiwa_link_0");
        DHBW_pose = generatePose(0.45, 0.62, 1.02, M_PI, 0, M_PI/4, "iiwa_link_0");
        home_pose = generatePose(0.5, 0, 1, M_PI, 0, 0, "iiwa_link_0");
}

void iwtros::iiwaMove::plcCallback(const iwtros_msgs::plcControl::ConstPtr& data){
        _plcSubscriberControl.MoveHome = data->MoveHome;
        _plcSubscriberControl.ConveyorPickPose = data->ConveyorPickPose;
        _plcSubscriberControl.DHBWPickPose = data->DHBWPickPose;
}

void iwtros::iiwaMove::run(){
        if(!_initialized){
                ROS_ERROR("IIWA Motion initialization is failed");
                return;
        }

        move_group.setPlannerId(PLANNER_ID);
        move_group.setMaxVelocityScalingFactor(velocityScalling);
        move_group.setMaxAccelerationScalingFactor(accelerationScalling);
        move_group.setPoseReferenceFrame(REFERENCE_FRAME);
        move_group.setEndEffectorLink(EE_FRAME);

        joint_model_group = move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);

        std::thread t1(&iiwaMove::_ctrl_loop, this);
        t1.join();
        ros::shutdown();
}

void iwtros::iiwaMove::_ctrl_loop(){
        static ros::Rate r(1);
        // ToDo: Check asynchronous spinner is required
        while(ros::ok()){
                poseUpdate();

                _plcKUKA.ConveyorPlaced = false;
                _plcKUKA.DHBWPlaced = false;
                _plcKUKA.ReachedHome = false;

                if(_plcSubscriberControl.MoveHome){
                        _plcSubscriberControl.MoveHome = false;
                        // ToDo Pose verification
                        motionExecution(home_pose);
                        _plcKUKA.ReachedHome = true;
                        _plcPub.publish(_plcKUKA);
                }
                if (_plcSubscriberControl.ConveyorPickPose){
                        _plcSubscriberControl.ConveyorPickPose = false;
                        pnpPipeLine(conveyor_pose, DHBW_pose, 0.12);
                        _plcKUKA.DHBWPlaced = true;
                        _plcPub.publish(_plcKUKA);
                }
                if(_plcSubscriberControl.DHBWPickPose){
                        _plcSubscriberControl.DHBWPickPose = false;
                        pnpPipeLine(DHBW_pose, conveyor_pose, 0.12);
                        _plcKUKA.ConveyorPlaced = true;
                        _plcPub.publish(_plcKUKA);
                }else{
                        ROS_INFO("No motion input");
                        _plcPub.publish(_plcKUKA);
                }

                r.sleep();
        }
}

void iwtros::iiwaMove::pnpPipeLine(geometry_msgs::PoseStamped pick,
                        geometry_msgs::PoseStamped place,
                        const double offset){
        // Go to Pick prepose (PTP)
        pick.pose.position.z += offset;
        motionExecution(pick);
        // ToDo: Open finger
        // Go to Pick pose, ToDo: Set LIN motion
        pick.pose.position.z -= offset;
        motionExecution(pick);
        // ToDo: Close finger
        // Go to Pick Postpose, ToDo: Set LIN motion
        pick.pose.position.z += offset;
        motionExecution(pick);
        // Go to Place Prepose (PTP)
        place.pose.position.z += offset;
        motionExecution(place);
        // Go to Place pose, ToDo: Set LIN motion
        place.pose.position.z -= offset;
        motionExecution(place);
        // ToDo: Open Finger
        // Go to Place Postpose, ToDo: Set LIN motion
        place.pose.position.z += offset;
        motionExecution(place);
}

void iwtros::iiwaMove::motionExecution(const geometry_msgs::PoseStamped pose){
        motionContraints(pose);
        move_group.setPoseTarget(pose);
        
        // ToDo: Valide the IK solution

        moveit::planning_interface::MoveGroupInterface::Plan mPlan;
        moveit::planning_interface::MoveItErrorCode eCode = move_group.plan(mPlan);
        ROS_INFO("Motion planning is: %s", eCode?"Success":"Failed");
        if(eCode) move_group.execute(mPlan);
        move_group.clearTrajectoryConstraints();
        move_group.clearPoseTarget();
}

void iwtros::iiwaMove::motionContraints(const geometry_msgs::PoseStamped pose){
        // Orientation contraints
        moveit_msgs::OrientationConstraint oCon;
        oCon.header.frame_id = REFERENCE_FRAME;
        oCon.link_name = EE_FRAME;
        oCon.orientation = pose.pose.orientation;
        oCon.absolute_x_axis_tolerance = 0.01;
        oCon.absolute_y_axis_tolerance = 0.01;
        oCon.absolute_z_axis_tolerance = 0.01;
        oCon.weight = 1.0;
        // ToDO: planning Contraints
        // Trajectory Contraints
        moveit_msgs::TrajectoryConstraints tCon;
        tCon.constraints.resize(1);
        tCon.constraints[0].orientation_constraints.push_back(oCon);
        tCon.constraints[0].position_constraints.resize(1);
        tCon.constraints[0].position_constraints[0].header.frame_id = REFERENCE_FRAME;
        tCon.constraints[0].position_constraints[0].link_name = EE_FRAME;
        tCon.constraints[0].position_constraints[0].constraint_region.primitive_poses.resize(1);
        tCon.constraints[0].position_constraints[0].constraint_region.primitive_poses[0] = pose.pose;
        tCon.constraints[0].position_constraints[0].constraint_region.primitives.resize(1);
        tCon.constraints[0].position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
        tCon.constraints[0].position_constraints[0].constraint_region.primitives[0].dimensions.push_back(2e-3);
        move_group.setTrajectoryConstraints(tCon);
        // ToDo: Goal Constraints
}

void iwtros::iiwaMove::visualMarkers(const geometry_msgs::PoseStamped target_pose,
                                        moveit::planning_interface::MoveGroupInterface::Plan plan){
        moveit_visual_tools::MoveItVisualTools visual_tool(REFERENCE_FRAME);
        visual_tool.deleteAllMarkers();
        visual_tool.loadRemoteControl();
        Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
        text_pose.translation().z() = 0.0;
        visual_tool.publishText(text_pose, "PnP Execution", rviz_visual_tools::WHITE, rviz_visual_tools::XLARGE);
        // visual_tool.trigger();
        // Visualize Trajectory
        visual_tool.publishAxisLabeled(target_pose.pose, "PnP");
        visual_tool.publishTrajectoryLine(plan.trajectory_, joint_model_group);
        visual_tool.trigger();
}