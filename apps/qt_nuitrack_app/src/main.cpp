#include <sstream>
#include "ros/ros.h"
#include "qt_nuitrack_app/qt_nuitrack_app.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "qt_nuitrack_app");
    ros::NodeHandle node;

    size_t nCallbacks = 2;

    qt_nuitrack_app::QTNuitrackApp qt_idle(node);

    ROS_INFO("qt_nuitrack_app is ready with %d callbacks", (int)nCallbacks);
    ros::AsyncSpinner spinner(nCallbacks);
    spinner.start();
    ros::waitForShutdown();
    ROS_INFO("qt_nuitrack_app is shutting down!");
    return 0;
}
