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
    float SetPosition;            //定义设定值
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
    pid.SetPosition=1.0;
    pid.SetAngle = M_PI_2l;
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
    pid.err=pid.SetAngle+pid.ActualAngle;
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "OdomOutBack_pub");
    ros::NodeHandle n;
    ros::Publisher OdomOutBack_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1);

    geometry_msgs::Twist Move_cmd;
    Pose_Type Pose_Currenrt;
//    float linear_speed = 0.1;
    float goal_distance = 1.0;
//    float angular_speed = -0.4;
    double goal_angle = M_PI_2l;
    double angular_tolerance = 0.0036332;

    int rate = 1000;
    ros::Rate loop_rate(rate);

    while (ros::ok())
    {

        Reset_MoveCmd(Move_cmd);

        for (int i = 0; i < 4; ++i)
        {
            PID_init();

            Pose_Type Pose_tmp;
            Pose_tmp = Get_RobotPose();
            double Distance = 0;
            // Move forward
            while(Distance < goal_distance && !ros::isShuttingDown())
            {
                //loop_rate.sleep();
                Pose_Currenrt = Get_RobotPose();
                Distance = sqrt(pow(Pose_Currenrt.first.x() - Pose_tmp.first.x(), 2) +
                                pow(Pose_Currenrt.first.y() - Pose_tmp.first.y(), 2));
                Move_cmd.linear.x= PID_realize_l(Distance);
                //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
                ROS_INFO("liner SPEED:%f",Move_cmd.linear.x);
                OdomOutBack_pub.publish(Move_cmd);
            }

            // Stop the robot
            Reset_MoveCmd(Move_cmd);
            OdomOutBack_pub.publish(Move_cmd);
            // Sleep for one second
            ros::Duration(1).sleep();

            double last_angle = Pose_Currenrt.second;
            double turn_angle = 0.0;
            double delta_angle;
            while(fabs(turn_angle-angular_tolerance) < goal_angle &&  !ros::isShuttingDown())
            {
                Move_cmd.angular.z =-1*PID_realize_a(turn_angle);
                OdomOutBack_pub.publish(Move_cmd);
                //ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
                ROS_INFO("angel speed:%f",Move_cmd.angular.z);
                //loop_rate.sleep();

                Pose_Currenrt = Get_RobotPose();
                delta_angle = Normalize_angle(Pose_Currenrt.second - last_angle);
                turn_angle += delta_angle;
                //ROS_INFO("turn_angle:%f,,,delta_angle:%f",turn_angle,delta_angle);
                last_angle = Pose_Currenrt.second;
                loop_rate.sleep();
            }
            // Stop the robot
            ROS_INFO("POSE:%f;%f;%f;%f",Pose_Currenrt.first.x(),Pose_Currenrt.first.y(),Pose_Currenrt.first.z(),Pose_Currenrt.second);
            Reset_MoveCmd(Move_cmd);
            OdomOutBack_pub.publish(Move_cmd);
            ros::Duration(1).sleep();



            //Move_cmd.linear.x = linear_speed;


            // Rotate left 180 degrees

        }
        // Close the node
        ros::shutdown();
    }
    return 0;
}
