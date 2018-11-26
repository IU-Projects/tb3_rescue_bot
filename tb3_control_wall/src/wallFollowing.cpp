#include "node_wallFollowing.h"

#define SUBSCRIBER_BUFFER_SIZE 1  // Size of buffer for subscriber.
#define PUBLISHER_BUFFER_SIZE 1000  // Size of buffer for publisher.
#define WALL_DISTANCE 0.0
#define MAX_SPEED 0.5
#define P 10    // Proportional constant for controller
#define D 5     // Derivative constant for controller
#define ANGLE_COEF 1    // Proportional constant for angle controller
#define DIRECTION -1 // 1 for wall on the left side of the robot (-1 for the right side).
// #define PUBLISHER_TOPIC "/syros/base_cmd_vel"
#define PUBLISHER_TOPIC "/cmd_vel"
// #define SUBSCRIBER_TOPIC "/syros/laser_laser"
#define SUBSCRIBER_TOPIC "/scan"

int main(int argc, char **argv)
{
    //Initialization of node
    ros::init(argc, argv, "tb3_control_wall");
    ros::NodeHandle n;

    //Creating publisher
    ros::Publisher pubMessage = n.advertise<geometry_msgs::Twist>(PUBLISHER_TOPIC, PUBLISHER_BUFFER_SIZE);

    //Creating object, which stores data from sensors and has methods for
    //publishing and subscribing
    NodeWallFollowing *nodeWallFollowing = new NodeWallFollowing(pubMessage, WALL_DISTANCE, MAX_SPEED, DIRECTION, P, D, 1);

    //Creating subscriber and publisher
    ros::Subscriber sub = n.subscribe(SUBSCRIBER_TOPIC, SUBSCRIBER_BUFFER_SIZE, &NodeWallFollowing::messageCallback, nodeWallFollowing);
    ros::spin();

    return 0;
}
