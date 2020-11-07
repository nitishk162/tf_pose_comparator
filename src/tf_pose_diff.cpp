#include<ros/ros.h>
#include<ros/package.h>
#include<tf/tf.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <iostream>
#include<geometry_msgs/PoseWithCovarianceStamped.h>
#include<geometry_msgs/PoseStamped.h>
#include<geometry_msgs/Pose2D.h>

#define LOG_RECORD 1
using namespace message_filters;
using namespace std;

#if LOG_RECORD
#include <fstream>
#endif

#if LOG_RECORD
ofstream delta_error_file;
std::string path;
#endif
ros::Publisher pub_error_amcl;

double normalizePrincipal( double z  )
{
	return atan2(sin(z),cos(z));
}

double angle_diff(double a, double b)
{
	double d1, d2;
	a = normalizePrincipal(a);
	b = normalizePrincipal(b);
	d1 = a-b;
	d2 = 2*M_PI - fabs(d1);
	if(d1 > 0)
		d2 *= -1.0;
	if(fabs(d1) < fabs(d2))
		return(d1);
	else
		return(d2);
}

void callback(const geometry_msgs::PoseStampedConstPtr &gt_pose, const geometry_msgs::PoseWithCovarianceStampedConstPtr &amcl)
{
	// calculating error in amcl.
	geometry_msgs::Pose2D errAmcl;
	double roll,pitch,yaw_amcl,yaw_gt;

	//get amcl yaw value
	tf::Quaternion qamcl(amcl->pose.pose.orientation.x, amcl->pose.pose.orientation.y,
			amcl->pose.pose.orientation.z, amcl->pose.pose.orientation.w );
	tf::Matrix3x3(qamcl).getRPY(roll,pitch,yaw_amcl);

	//get ground truth yaw value.
	tf::Quaternion q(gt_pose->pose.orientation.x, gt_pose->pose.orientation.y,
			gt_pose->pose.orientation.z, gt_pose->pose.orientation.w );
	tf::Matrix3x3(q).getRPY(roll,pitch,yaw_gt);

	//find error and publish it 
	errAmcl.x = fabs(gt_pose->pose.position.x - amcl->pose.pose.position.x);
	errAmcl.y = fabs(gt_pose->pose.position.y - amcl->pose.pose.position.y);
	errAmcl.theta = fabs(angle_diff(yaw_gt, yaw_amcl));
	pub_error_amcl.publish(errAmcl);

	double linearError, angularError;
	linearError = sqrt(pow(errAmcl.x,2) + pow(errAmcl.y,2));
	angularError = errAmcl.theta;

	//file recording
#if LOG_RECORD
	delta_error_file<<amcl->pose.pose.position.x<<","<<amcl->pose.pose.position.y<<","<<errAmcl.x<<","<<errAmcl.y<<","<<errAmcl.theta<<","<<linearError<<","<<angularError<<endl;
#endif

}


int main(int argc, char** argv)
{
	ros::init(argc,argv,"compare_tf_pose");
	ros::NodeHandle node;
	std::string path = ros::package::getPath("tf_pose_comparator");
		path.append("/logs/delta_error_file.dat"); 
#if LOG_RECORD
	delta_error_file.open(path.c_str());
#endif


 	message_filters::Subscriber<geometry_msgs::PoseStamped> ground_truth_sub(node, "/ground_truth_pose", 1);
	message_filters::Subscriber<geometry_msgs::PoseWithCovarianceStamped> amcl_sub(node, "/amcl_pose", 1);

	typedef sync_policies::ApproximateTime<geometry_msgs::PoseStamped, geometry_msgs::PoseWithCovarianceStamped> compare_sync_policy;
	// // ApproximateTime takes a queue size as its constructor argument, hence compare_sync_policy(10)
	Synchronizer<compare_sync_policy> sync(compare_sync_policy(20), ground_truth_sub, amcl_sub);
	sync.registerCallback(boost::bind(&callback, _1, _2));

	pub_error_amcl = node.advertise<geometry_msgs::Pose2D>("err_amcl",100);
	ros::spin();
	return 0;
}