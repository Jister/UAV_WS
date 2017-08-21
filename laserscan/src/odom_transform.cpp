#include "ros/ros.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "geometry_msgs/Pose2D.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/Range.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "tf/message_filter.h"
#include <message_filters/subscriber.h>

using namespace std;


class OdomTransform
{
public: 
	OdomTransform();
private:
	ros::NodeHandle n;
	ros::Subscriber odom_sub;
	ros::Subscriber lidar_sub;
	ros::Publisher vision_pub;	

	bool imu_init;
	// double roll;
	// double pitch;
	// double yaw;
	double distance;

	void poseCallback(const geometry_msgs::Pose2D &msg);
	void lidarCallback(const sensor_msgs::Range &msg);
	void laserCallback(const geometry_msgs::PoseWithCovarianceStamped &msg);
	//void rotate(float theta,  const Vector3f& input,  Vector3f& output);
};

OdomTransform::OdomTransform()
{
	odom_sub = n.subscribe("/amcl_pose", 1, &OdomTransform::laserCallback, this);
	lidar_sub = n.subscribe("/mavros/lidar/range", 1, &OdomTransform::lidarCallback, this);
	vision_pub = n.advertise<geometry_msgs::PoseStamped >("/mavros/vision_pose/pose", 1);
	// imu_init = false;
	// roll = 0;
	// pitch = 0;
	// yaw = 0;
	distance = 0;
}

void OdomTransform::poseCallback(const geometry_msgs::Pose2D &msg)
{
	geometry_msgs::Quaternion laser_world_q = tf::createQuaternionMsgFromYaw(msg.theta);
	geometry_msgs::PoseStamped  pos;
	pos.header.stamp = ros::Time::now();

	pos.pose.position.x = msg.x;
	pos.pose.position.y = msg.y;
	pos.pose.position.z = distance;
	pos.pose.orientation = laser_world_q;
	vision_pub.publish(pos);
}

void OdomTransform::laserCallback(const geometry_msgs::PoseWithCovarianceStamped &msg) {
	geometry_msgs::PoseStamped  pos;
	pos.header.stamp = ros::Time::now();
	pos.pose.position.x = msg.pose.pose.position.x;
	pos.pose.position.y = msg.pose.pose.position.y;
	pos.pose.position.z = distance;
	pos.pose.orientation = msg.pose.pose.orientation;
	vision_pub.publish(pos);
}

// void OdomTransform::imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
// {
// 	if(!imu_init)
// 	{
// 		tf::Quaternion q;
// 		tf::quaternionMsgToTF(imu_msg->orientation, q);
// 		tf::Matrix3x3 m(q);
// 		m.getRPY(roll, pitch, yaw);
// 		imu_init = true;
// 	}
// }

void OdomTransform::lidarCallback(const sensor_msgs::Range &msg)
{
	distance = msg.range;
}

// void OdomTransform::rotate(float theta,  const Vector3f& input,  Vector3f& output)
// {
// 	float sy = sinf(theta);
// 	float cy = cosf(theta);

// 	Matrix3f data;
// 	data(0,0) = cy;
// 	data(0,1) = -sy;
// 	data(0,2) = 0.0;
// 	data(1,0) = sy;
// 	data(1,1) = cy;
// 	data(1,2) = 0.0;
// 	data(2,0) = 0.0;
// 	data(2,1) = 0.0;
// 	data(2,2) = 1.0;

// 	output = data * input;
// }


int main(int argc, char **argv)
{
	ros::init(argc, argv, "odom_transform");
	OdomTransform Odomtransform;
	ros::spin();
}
