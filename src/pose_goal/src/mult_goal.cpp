#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <iostream>
#include <string>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
//MoveBaseClient ac("move_base",true);
ros::Publisher pub;// = nh.advertise<std_msgs::String>("topic_name", 1000);
float goal0[4] = {5.904,-4.337,1.000,0.000};
float goal1[4] = {6.052,0.723,1.000,0.000};
float goal2[4] = {-5.167,1.021,-0.668,0.744};
float goal3[4] = {5.806,-1.587,-0.832,0.550};
float goal4[4] = {6.194,-7.246,0.732,0.682};

void setHome()
{
    geometry_msgs::PoseWithCovarianceStamped msg_poseinit;
    msg_poseinit.header.frame_id = "map";
    msg_poseinit.header.stamp = ros::Time::now();
    msg_poseinit.pose.pose.position.x = 5.904;
    msg_poseinit.pose.pose.position.y = -4.337;
    msg_poseinit.pose.pose.orientation.z = 1.0;
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
    pub.publish(msg_poseinit);
    ros::Duration(1.0).sleep();
}

int setGoal(float *goal_position)
{
    MoveBaseClient ac("move_base",true);
    move_base_msgs::MoveBaseGoal goal;
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server");
    }
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = goal_position[0];
    goal.target_pose.pose.position.y = goal_position[1];
    goal.target_pose.pose.orientation.z = goal_position[2];
    goal.target_pose.pose.orientation.w = goal_position[3];

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have arrived to the goal reason");
        return 1;
    }
    else
    {
        ROS_INFO("The base failed for some reason");
        return 0;
    }
}

int main(int argc,char** argv)
{
    ros::init(argc, argv, "navigation_goal");
    ros::NodeHandle n;
    pub = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("/initialpose",1);

    char home_position;
    while(1)
    {
        ROS_INFO("If you are in the home,press the 'y' to continue,'n' is set by hand");
        cin >> home_position;
        if(home_position == 'y')
        {
            setHome();
            break;
        }
        if(home_position == 'n')
        {
            ROS_INFO("Pleas set home position by hand.");
            break;
        }
        else
            ROS_INFO("Please put robot on home please.");
    }
    
    while(ros::ok())
    {
        ROS_INFO("Where are you going?(0,1,2,3,4)");
        int position_num;
        cin >> position_num;
        switch (position_num)
        {
            case 0:
            {
                setGoal(goal0);
                break;
            }

            case 1:
            {
                setGoal(goal1);
                break;
            }

            case 2:
            {
                setGoal(goal2);
                break;
            }

            case 3:
            {
                setGoal(goal3);
                break;
            }

            case 4:
            {
                setGoal(goal4);
                break;
            }
                
            default:
            {
                ROS_INFO("The program will stop!");
                return 0;
            }
        }
    }
    return 0;
}