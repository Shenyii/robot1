#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <string>

using namespace std;

int main(int argc,char** argv)
{
    ros::init(argc, argv, "navigation_goal");
    ros::NodeHandle n;
    ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

    char end_flag;
    //while(!ac.waitForServer(ros::Duration(5.0)))
    while(ros::ok())
    {
        geometry_msgs::PoseStamped goal;
        goal.header.frame_id = "map";
        goal.header.stamp = ros::Time::now();
        goal.pose.position.x = 5.904;
        goal.pose.position.y = -4.337;
        goal.pose.orientation.z = 1.000;

        ROS_INFO("Sending goal");
        pub.publish(goal);

        cout << "Are you continue ? y is continue." << endl;
        cin >> end_flag;
        if(end_flag != 'y')
        {
            break;
        }
    }
    return 0;
}