#include <ros/ros.h>
#include "nav_msgs/Path.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <vector>

using namespace std;
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int flag=1;
void turn_left(MoveBaseClient &ac,tf::Quaternion &Orientation,move_base_msgs::MoveBaseGoal &goal)
{
    ROS_INFO("turn_left!");
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.orientation.x=Orientation.x();
    goal.target_pose.pose.orientation.y=Orientation.y();
    goal.target_pose.pose.orientation.z=Orientation.z();
    goal.target_pose.pose.orientation.w=Orientation.w();
    ac.sendGoal(goal);
    ac.waitForResult();///!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
//    sleep(1);
}
void turn_right(MoveBaseClient &ac,tf::Quaternion &Orientation,move_base_msgs::MoveBaseGoal &goal)
{
    ROS_INFO("turn_right!");
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.orientation.x=-1*Orientation.x();
    goal.target_pose.pose.orientation.y=-1*Orientation.y();
    goal.target_pose.pose.orientation.z=-1*Orientation.z();
    goal.target_pose.pose.orientation.w=Orientation.w();
    ac.sendGoal(goal);
    ac.waitForResult();
//    sleep(1);
}
void move_forward(MoveBaseClient &ac,move_base_msgs::MoveBaseGoal &goal)
{
    ROS_INFO("move!");

    goal.target_pose.pose.position.x = 0.5;
    goal.target_pose.pose.orientation.x=0.0;
    goal.target_pose.pose.orientation.y=0.0;
    goal.target_pose.pose.orientation.z=0.0;
    goal.target_pose.pose.orientation.w = 1.0;
    ac.sendGoal(goal);
}
void callback(const nav_msgs::Path::ConstPtr& msg)
{
    ROS_INFO("GO INTO CALLBACK!");
    cout<<"seq:::::::::::"<<msg->header.stamp.sec<<endl;
    if(msg->header.stamp.sec==0)
        flag=0;
}
int main(int argc, char** argv)
{
    int count=0;
    int flag2=1;
  ros::init(argc, argv, "simple_navigation_goals");
  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("/move_base/NavfnROS/plan",1,callback);

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }

    move_base_msgs::MoveBaseGoal goal1;

    //tf::Quaternion Orientation;
//  double theta=M_PI_2l;
    double target[27][2];
    //we'll send a goal to the robot to move 1 meter forward
    goal1.target_pose.header.frame_id = "map";
    goal1.target_pose.header.stamp = ros::Time::now();
    //Orientation.setRPY(0, 0, theta);
    target[0][0]=0.5;target[0][1]=7.4;
    target[1][0]=-3.9;target[1][1]=8.1;
    target[2][0]=-5.4;target[2][1]=6.7;
    target[3][0]=-4.8;target[3][1]=5.5;
    target[4][0]=-0.6;target[4][1]=4.9;
    target[5][0]=-0.6;target[5][1]=2.3;
    target[6][0]=-4.4;target[6][1]=2.1;
    target[7][0]=-5.0;target[7][1]=-1.6;
    target[8][0]=-2.6;target[8][1]=-2.7;
    target[9][0]=-5.3;target[9][1]=-3.6;
    target[10][0]=-5.4;target[10][1]=-4.3;
    target[11][0]=-0.6;target[11][1]=-4.9;
    target[12][0]=0;target[12][1]=0;
    target[13][0]=-0.3;target[13][1]=-0.1;
    target[14][0]=0;target[14][1]=6.9;
    target[15][0]=-4.3;target[15][1]=6.8;
    target[16][0]=-3.2;target[16][1]=5.6;
    target[17][0]=-3.0;target[17][1]=6.5;
    target[18][0]=-1.7;target[18][1]=6.2;
    target[19][0]=-0.7;target[19][1]=5.9;
    target[20][0]=-0.8;target[20][1]=1.4;
    target[21][0]=-4.4;target[21][1]=-1.5;
    target[22][0]=-2.4;target[22][1]=-2.7;
    target[23][0]=-3.0;target[23][1]=-3.8;
    target[24][0]=-1.0;target[24][1]=-4.1;
    target[25][0]=-1.0;target[25][1]=-3.6;
    target[26][0]=0;target[26][1]=0;

    for(int i=0;i<27;i++)
    {
        goal1.target_pose.pose.position.x=target[i][0];
        goal1.target_pose.pose.position.y=target[i][1];
        goal1.target_pose.pose.orientation.w = 1.0;
        ROS_INFO("Sending goal");
        ac.sendGoal(goal1);
        ac.waitForResult();
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("go to next point!");
        else
            ROS_INFO("The base failed for some reason");
    }

    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

  tf::Quaternion Orientation;
    tf::Quaternion Orientation1;
  double theta=M_PI_2l;
    theta=theta/3;
    Orientation.setRPY(0, 0, theta);
    cout<<Orientation.x()<<"   "<<Orientation.y()<<"   "<<Orientation.z()<<"   "<<Orientation.w()<<endl;

    while(ros::ok())
    {
        move_forward(ac,goal);
        sleep(1);
        ros::spinOnce();
        cout<<"flag:"<<flag<<endl;
        if(flag==0)
        {
            ROS_INFO("reset flag");
            ac.cancelGoal();
            flag=1;
            flag2=0;
        }
        else
        {
            ac.waitForResult();
            sleep(1);
        }
        if (ac.getState() == actionlib::SimpleClientGoalState::ABORTED || flag2==0)
        {
            ROS_INFO("fail!");
            turn_left(ac,Orientation,goal);
            count=0;
            flag2=1;
        }
        else if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        {
            count++;
        }
         if(count==3)
         {
             Orientation1.setRPY(0, 0, theta*3);
             turn_right(ac,Orientation1,goal);
             count=0;
         }
    }

  return 0;
}
