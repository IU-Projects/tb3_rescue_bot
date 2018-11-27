#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Pose.h"
#include "visualization_msgs/Marker.h"

ros::Publisher marker_pub;
int victim_number = 0;

void processVictimInfo(const geometry_msgs::Pose::ConstPtr& msg)
{
    ROS_INFO("Received victim information-> x: [%f], y: [%f], z: [%f]", msg->position.x, msg->position.y, msg->position.z);

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
    marker.pose.position.x = msg->position.x;
    marker.pose.position.y = msg->position.y;
    marker.pose.position.z = msg->position.z;
    marker.pose.orientation.x = msg->orientation.x;
    marker.pose.orientation.y = msg->orientation.y;
    marker.pose.orientation.z = msg->orientation.z;
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
    ROS_INFO("Marker Published");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "victim_mapper");

    ros::NodeHandle n;
    marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);
    ros::Subscriber sub = n.subscribe("victim_info_listener", 1000, processVictimInfo);

    ROS_INFO("Waiting for victim information messages!");
    ros::spin();

    return 0;
}