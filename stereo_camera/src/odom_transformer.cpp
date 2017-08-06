#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"

class OdomTransformer
{
public: 
	OdomTransformer();
private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;
	ros::Publisher vision_pub;	

	void odomCallback(const nav_msgs::Odometry msg);
};

OdomTransformer::OdomTransformer()
{
	odom_sub = n.subscribe("/zed/odom", 1, &OdomTransformer::odomCallback, this);
	vision_pub = n.advertise<geometry_msgs::PoseStamped >("/mavros/vision_pose/pose", 1);
}

void OdomTransformer::odomCallback(const nav_msgs::Odometry msg)
{
	geometry_msgs::PoseStamped  pos;
	pos.header = msg.header;
	pos.pose = msg.pose.pose;
	vision_pub.publish(pos);
}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_transformer");
	OdomTransformer Odomtransformer;
	ros::spin();
}