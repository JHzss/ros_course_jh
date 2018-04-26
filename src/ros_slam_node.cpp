#include <ros/ros.h>
#include <string>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <sstream>

using namespace std;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

cv::Mat show_img;

int num=0;
void image_receive(const sensor_msgs::ImageConstPtr& msg)
{
    //cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);//RGB8
    cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

    show_img = cv_ptr->image.clone();
    num++;
    //count=count+1;
    cv::imwrite("/home/jh/output.jpg", show_img);
    char name[30];
    sprintf(name,"/home/jh/image/%d.jpg",num);
    cv::imwrite(name, show_img);
    //const cv::Mat& frame = cv_ptr->image;
    //cv::imshow("image", show_img);
    cout<<"into"<<num<<endl;


    //writer.release();
}

int main(int argc, char** argv)
{
    int flag=0;
    ros::init(argc, argv, "ros_slam");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/camera/rgb/image_raw", 1, image_receive);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.frame_id ="map";
    goal.target_pose.header.stamp = ros::Time::now();

    cout<<"Please enter a target location(X):"<<endl;
    float X=0.0;
    cin>>X;

    cout<<"Please enter a target location(Y):"<<endl;
    float Y=0.0;
    cin>>Y;

    goal.target_pose.pose.position.x = X;
    goal.target_pose.pose.position.y = Y;
    goal.target_pose.pose.orientation.w = 1.0;
    goal.target_pose.pose.orientation.z = 2.0;
    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();
    cv::Mat img;
    cv::VideoWriter writer("/home/jh/capture.avi", CV_FOURCC('M', 'J', 'P', 'G'), 30.0, cv::Size(640,480), true);
    while(1)
    {
        if(num==300)
        {

            for(int k=1;k<=300;k++)
            {
                if (!writer.isOpened())
                {
                    return 1;
                }
                char save[30];
                sprintf(save,"/home/jh/image/%d.jpg",k);
                cout<<save<<endl;
                img=cv::imread(save);
//                cv::imshow("image",img);
//                cv::waitKey(0);
                writer << img;
                cout<<"save one image"<<endl;

            }
            writer.release();
//            ros::shutdown();
            return 2;
        }
        else
            ros::spinOnce();
    }

}