#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <vector>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

  move_base_msgs::MoveBaseGoal goal;

  //tf::Quaternion Orientation;
//  double theta=M_PI_2l;
    double target[4][2];
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  //Orientation.setRPY(0, 0, theta);
    target[0][0]=1.0;
    target[0][1]=1.0;

    target[1][0]=1.0;
    target[1][1]=2.0;

    target[2][0]=2.0;
    target[2][1]=2.0;

    target[3][0]=2.0;
    target[3][1]=1.0;

    for(int i=0;i<5;i++)
    {
        if(i!=4)
        {
            goal.target_pose.pose.position.x=target[i][0];
            goal.target_pose.pose.position.y=target[i][1];
            goal.target_pose.pose.orientation.w = 1.0;
        }
        else
        {
            goal.target_pose.pose.position.x=target[0][0];
            goal.target_pose.pose.position.y=target[0][1];
            goal.target_pose.pose.orientation.w = 1.0;
        }

        ROS_INFO("Sending goal");
        ac.sendGoal(goal);

        ac.waitForResult();

        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Hooray, the base moved 1 meter forward");
        else
            ROS_INFO("The base failed to move forward 1 meter for some reason");
    }


  return 0;
}
