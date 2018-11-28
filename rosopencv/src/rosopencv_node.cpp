#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
//imports from victim mapper
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"
#include <tf/transform_listener.h>


static const std::string TOPIC_NAME = "camera/rgb/image_raw";
static const std::string DEPTH_TOPIC_NAME = "camera/depth/image_raw";

ros::Publisher marker_pub;
ros::Subscriber odom_sub;
tf::Point Odom_pos;	//odometry position (x, y, z)
double xg = 0.5;
double yg = 1.5;
double Odom_yaw;	//odometry orientation (yaw)
double Odom_v, Odom_w;	//odometry linear and angular speeds

class Points{
public:
    double x;
    double y;
};

int victim_number = 0;

using namespace cv;
using namespace std;

static bool detected=false;

vector<Points> victimLocations;
Points initial;



double eucdistance(Points p1, Points p2) {
    double x = 0;
    x = sqrt((p1.x-p2.x) * (p1.x-p2.x) + (p1.y-p2.y) * (p1.y-p2.y));
    return x;
}

bool isClose(Points point) {
    bool status = false;
    for (int i = 0; i < victimLocations.size(); ++i) {
        Points org = victimLocations.at(i);
        double gap = eucdistance(org, point);
        if(gap>3) {
            status = false;
        }else{
            status = true;
            break;
        }
    }
    return status;
}



void processVictimInfo(double x, double y)
{

    //ROS_INFO("Received victim information-> x: [%f], y: [%f], z: [%f]", x, y, 0.0);

    uint32_t shape = visualization_msgs::Marker::SPHERE;
    visualization_msgs::Marker marker;
    // Set the frame ID and timestamp.  See the TF tutorials for information on these.
    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();

    // Set the namespace and id for this marker.  This serves to create a unique ID
    // Any marker sent with the same namespace and id will overwrite the old one
    marker.ns = "basic_shapes";
    victim_number++;
    marker.id = victim_number;

    // Set the marker type.  Initially this is CUBE, and cycles between that and SPHERE, ARROW, and CYLINDER
    marker.type = shape;

    // Set the marker action.  Options are ADD, DELETE, and new in ROS Indigo: 3 (DELETEALL)
    marker.action = visualization_msgs::Marker::ADD;

    // Set the pose of the marker.  This is a full 6DOF pose relative to the frame/time specified in the header
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0;
    marker.pose.orientation.y = 0;
    marker.pose.orientation.z = 0;
    marker.pose.orientation.w = 1;

    // Set the scale of the marker -- 1x1x1 here means 1m on a side
    marker.scale.x = 1.0;
    marker.scale.y = 1.0;
    marker.scale.z = 1.0;

    // Set the color -- be sure to set alpha to something non-zero!
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration();

    marker_pub.publish(marker);
    //ROS_INFO("Marker Published");


}

void odomCallback(const nav_msgs::Odometry odom_msg)
{
    /* upon "hearing" odom msg, retrieve its position and orientation (yaw) information.
     * tf utility functions can be found here: http://docs.ros.org/diamondback/api/tf/html/c++/namespacetf.html
     * odom data structure definition is here: https://mirror.umd.edu/roswiki/doc/diamondback/api/nav_msgs/html/msg/Odometry.html
     */
    //tf::pointMsgToTF(odom_msg.pose.pose.position, Odom_pos);
    xg = odom_msg.pose.pose.position.x;
    yg = odom_msg.pose.pose.position.y;
    //display on terminal screen
    //ROS_INFO("Position: (%f, %f)", xg, yg);
}


void imageCallback(const sensor_msgs::ImageConstPtr& msg ) {
    try {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::imwrite("~/Pictures/rgb.bmp", cv_ptr->image);
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        cv::waitKey(30);
        Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
        Mat outputImage;
        inRange(image, Scalar(10,10,100), Scalar(100,100,255), outputImage);
        //lower_green = np.array([65,60,60])
        //upper_green = np.array([80,255,255])
        Mat outputImage2;
        inRange(image, Scalar(65,60,60), Scalar(80,255,255), outputImage2);
        Mat outputImage3;
        inRange(image, Scalar(100,150,0), Scalar(140,255,255), outputImage3);
        int x1;
        int x2;
        int x3;
        x1 = countNonZero(outputImage);
        //lower_blue = np.array([100,150,0])
        //upper_blue = np.array([140,255,255])
        x2 = countNonZero(outputImage2);
        x3 = countNonZero(outputImage3);
        if(x1>90000){
            cout << "Victim Detected, " << "Output Value : " << x1 << ", Position : " << xg << ", " << yg << "Victim Count : " << victimLocations.size() << endl;

            Points current;
            current.x = xg;
            current.y = yg;
            bool isC = isClose(current);
            if(!isC) {
                processVictimInfo(xg, yg);
                victimLocations.push_back(current);
            }

        } else {
            cout << "Keep Searching ..." << x1 << x2 << x3 << endl;
        }

    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.",
                  msg->encoding.c_str());
    }
}

void imageDepthCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
        // Save image
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "16UC1");
        cv::Mat mat = cv_ptr->image;
        cv::Mat beConvertedMat(mat.rows, mat.cols, CV_8UC4, mat.data); // provide different view of m1 data
        cv::imwrite("~/Pictures/rgbd.bmp", beConvertedMat);
        // Show image
        cv::imshow("depthview", cv_bridge::toCvShare(msg, "16UC1")->image);
        cv::waitKey(30);
    } catch (cv_bridge::Exception& e) {
        ROS_ERROR("Could not convert from '%s' to '16UC1'.",
                  msg->encoding.c_str());
    }
}




int main(int argc, char **argv) {

    ros::init(argc, argv, "image_transport_subscriber");
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::namedWindow("depthview");
    cv::startWindowThread();

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber sub = it.subscribe(TOPIC_NAME, 1,
                                                   imageCallback);
//    image_transport::Subscriber sub_depth = it.subscribe(DEPTH_TOPIC_NAME, 1,
//                                                         imageDepthCallback);

    initial.x = -2.5;
    initial.y = 2.9;
    victimLocations.push_back(initial);
    odom_sub    = nh.subscribe("odom", 20, odomCallback);
    marker_pub = nh.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::spin();
    cv::destroyWindow("view");
    //cv::destroyWindow("depthview");
    ros::shutdown();
    return 0;
}