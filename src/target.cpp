#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/tf.h>
#include <iostream>
#include "math.h"

typedef std::pair<tf::Vector3, double> Pose_Type;

struct _pid{
    double SetPosition;            //定义设定值
    float ActualPosition;        //定义实际值
    float SetAngle;            //定义设定值
    float ActualAngle;        //定义实际值
    float err;                //定义偏差值
    float err_last;            //定义上一个偏差值
    float Kp,Ki,Kd;            //定义比例、积分、微分系数
    float speed;          //定义速度
    float integral;            //定义积分值
     }pid;
void PID_init(){
    printf("PID_init begin \n");
    pid.SetPosition=0.0;
    pid.SetAngle = 0.0;
    pid.ActualPosition=0.0;
    pid.ActualAngle=0.0;
    pid.err=0.0;
    pid.err_last=0.0;
    pid.speed=0.0;
    pid.integral=0.0;
    pid.Kp=1;
    pid.Ki=0.005;
    pid.Kd=0.02;
    printf("PID_init end \n");
}

float PID_realize_l(float Position){
    //pid.SetPosition=Position;
    pid.ActualPosition=Position;
    pid.err=pid.SetPosition-pid.ActualPosition;
    //ROS_INFO("ERROR:%f",pid.err);
    pid.integral+=pid.err;
    pid.speed=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    pid.err_last=pid.err;

    return pid.speed;
}
float PID_realize_a(float Position){
    //pid.SetPosition=Position;
    pid.ActualAngle=Position;
    pid.err=pid.SetAngle-fabs(pid.ActualAngle);
    //ROS_INFO("ERROR:%f",pid.err);
    pid.integral+=pid.err;
    pid.speed=pid.Kp*pid.err+pid.Ki*pid.integral+pid.Kd*(pid.err-pid.err_last);
    pid.err_last=pid.err;

    return pid.speed;
}

void Reset_MoveCmd(geometry_msgs::Twist &Move_cmd)
{
    Move_cmd.linear.x = 0.0;
    Move_cmd.linear.y = 0.0;
    Move_cmd.linear.z = 0.0;

    Move_cmd.angular.x = 0.0;
    Move_cmd.angular.y = 0.0;
    Move_cmd.angular.z = 0.0;
}

// Get the current Position and Yaw angular of the robot
Pose_Type Get_RobotPose()
{
    tf::TransformListener listener;
    tf::StampedTransform  transform;
    tf::Quaternion Orientation;
    tf::Vector3 Position;
    bool flag = true;
    double Roll, Pitch, Yaw;

    try
    {
        if(flag)
        {
            listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(3.0));
            flag = false;
        }
        listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
    Position = transform.getOrigin();
    Orientation = transform.getRotation();
    tf::Matrix3x3(Orientation).getRPY(Roll, Pitch, Yaw);

    return std::make_pair(Position, Yaw);
}
double Normalize_angle(double angle)
{
    double Yaw = angle;
    while(Yaw>M_PIl)
        Yaw -= 2.0*M_PIl;
    while(Yaw<-M_PIl)
        Yaw += 2.0*M_PIl;
    return Yaw;
}
void move_forward(ros::Publisher &OdomOutBack_pub,float distance)
{
    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
    pid.SetPosition=fabs(distance);
    double Distance=0;
    Pose_Type Pose_tmp;
    Pose_tmp = Get_RobotPose();
    while(Distance < fabs(distance) && !ros::isShuttingDown())
    {
        Pose_Currenrt = Get_RobotPose();
        Distance = sqrt(pow(Pose_Currenrt.first.x() - Pose_tmp.first.x(), 2) +
                        pow(Pose_Currenrt.first.y() - Pose_tmp.first.y(), 2));
        ROS_INFO("distance:%f",Distance);
        Move_cmd.linear.x= PID_realize_l(Distance);
        //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
        ROS_INFO("liner SPEED:%f",Move_cmd.linear.x);
        OdomOutBack_pub.publish(Move_cmd);
    }
    Reset_MoveCmd(Move_cmd);
    OdomOutBack_pub.publish(Move_cmd);
    ros::Duration(1).sleep();
}
void move_back(ros::Publisher &OdomOutBack_pub,float distance)
{
    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
    pid.SetPosition=fabs(distance);
    double Distance=0;
    Pose_Type Pose_tmp;
    Pose_tmp = Get_RobotPose();
    while(Distance < fabs(distance) && !ros::isShuttingDown())
    {
        Pose_Currenrt = Get_RobotPose();
        Distance = sqrt(pow(Pose_Currenrt.first.x() - Pose_tmp.first.x(), 2) +
                        pow(Pose_Currenrt.first.y() - Pose_tmp.first.y(), 2));
        ROS_INFO("distance:%f",Distance);
        Move_cmd.linear.x= -1*PID_realize_l(Distance);
        //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
        ROS_INFO("liner SPEED:%f",Move_cmd.linear.x);
        OdomOutBack_pub.publish(Move_cmd);
    }
    Reset_MoveCmd(Move_cmd);
    OdomOutBack_pub.publish(Move_cmd);
    ros::Duration(1).sleep();
}
void rotate_right(ros::Publisher &OdomOutBack_pub,double angel)
{

    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
    Pose_Currenrt = Get_RobotPose();
    pid.SetAngle=angel;
    double angular_tolerance = 0.0036332;
    double last_angle = Pose_Currenrt.second;
    double turn_angle = 0.0;
    double delta_angle;
    ros::Rate loop_rate(1000);
    while(fabs(turn_angle-angular_tolerance) < angel &&  !ros::isShuttingDown())
    {
        Move_cmd.angular.z =-1*PID_realize_a(turn_angle);
        OdomOutBack_pub.publish(Move_cmd);
        //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
        ROS_INFO("angel speed:%f",Move_cmd.angular.z);
        //loop_rate.sleep();

        Pose_Currenrt = Get_RobotPose();

        //ROS_INFO("chazhi:%f,,,%f",Pose_Currenrt.second ,last_angle);
        delta_angle = Normalize_angle(Pose_Currenrt.second - last_angle);
        turn_angle += delta_angle;
        //ROS_INFO("turn_angle:%f,,,delta_angle:%f",turn_angle,delta_angle);
        last_angle = Pose_Currenrt.second;
        loop_rate.sleep();
    }
    // Stop the robot
    Reset_MoveCmd(Move_cmd);
    OdomOutBack_pub.publish(Move_cmd);
    // Sleep for one second
    ros::Duration(1).sleep();
}
void rotate_left(ros::Publisher &OdomOutBack_pub,double angel)
{

    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
    Pose_Currenrt = Get_RobotPose();
    pid.SetAngle=angel;
    double angular_tolerance = 0.0036332;
    double last_angle = Pose_Currenrt.second;
    double turn_angle = 0.0;
    double delta_angle;
    ros::Rate loop_rate(1000);
    while(fabs(turn_angle+angular_tolerance) < angel &&  !ros::isShuttingDown())
    {
        Move_cmd.angular.z =PID_realize_a(turn_angle);
        OdomOutBack_pub.publish(Move_cmd);
        //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
        ROS_INFO("angel speed:%f",Move_cmd.angular.z);
        //loop_rate.sleep();

        Pose_Currenrt = Get_RobotPose();

        //ROS_INFO("chazhi:%f,,,%f",Pose_Currenrt.second ,last_angle);
        delta_angle = Normalize_angle(Pose_Currenrt.second - last_angle);
        turn_angle += delta_angle;
        ROS_INFO("turn_angle:%f,,,delta_angle:%f",turn_angle,delta_angle);
        last_angle = Pose_Currenrt.second;
        loop_rate.sleep();
    }
    // Stop the robot
    Reset_MoveCmd(Move_cmd);
    OdomOutBack_pub.publish(Move_cmd);
    // Sleep for one second
    ros::Duration(1).sleep();
}
void angelcal(ros::Publisher &OdomOutBack_pub,double angel)
{
    if(angel>0)
        rotate_right(OdomOutBack_pub,0.01);
    if(angel<0)
        rotate_left(OdomOutBack_pub,0.01);
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdomOutBack_pub");
    ros::NodeHandle n;
    ros::Publisher OdomOutBack_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;

    float goal_distance_x = atoll(argv[1]);
    float goal_distance_y = atoll(argv[2]);

    double pai = M_PI_2l;

    int rate = 1000;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {
        int flag_left=0,flag_right=0;
        Reset_MoveCmd(Move_cmd);

        PID_init();
        Pose_Currenrt = Get_RobotPose();
        if(goal_distance_x-Pose_Currenrt.first.x()>0)
            move_forward(OdomOutBack_pub,goal_distance_x-Pose_Currenrt.first.x());
        else if(goal_distance_x-Pose_Currenrt.first.x()<0)
            move_back(OdomOutBack_pub,goal_distance_x-Pose_Currenrt.first.x());
        PID_init();
        if(goal_distance_y-Pose_Currenrt.first.y()>0)
        {
            rotate_left(OdomOutBack_pub,pai);
            flag_left=1;
        }
        if(goal_distance_y-Pose_Currenrt.first.y()<0)
        {
            rotate_right(OdomOutBack_pub,pai);
            flag_right=1;
        }
        PID_init();
        move_forward(OdomOutBack_pub,goal_distance_y-Pose_Currenrt.first.y());
        if(flag_left==1)
        {
            rotate_right(OdomOutBack_pub,pai);
            flag_left=0;
        }
        if(flag_right==1)
        {
            rotate_left(OdomOutBack_pub,pai);
            flag_right=0;
        }
        loop_rate.sleep();
        while(fabs(Get_RobotPose().second)>0.01)
            angelcal(OdomOutBack_pub,Get_RobotPose().second);
        Pose_Currenrt = Get_RobotPose();
        ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
        // Close the node
            ros::shutdown();
    }
    return 0;
}
