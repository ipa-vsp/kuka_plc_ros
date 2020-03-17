#include <iwtros_goal/iiwa_manipulation.h>

int main(int argc, char ** argv){
    ros::init(argc, argv, "pnp_node");
    ros::NodeHandle nh;

    iwtros::iiwaMove mover(nh, "iiwa_arm");
    ros::AsyncSpinner spinner(1);
    mover.run();
    spinner.stop();

    return 0;
}