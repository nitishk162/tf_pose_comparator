This package compares two poses of type geometry_msgs/PoseWithCovarianceStamped and geometry_msgs/PoseStamped
using the TimeSync policy from message filters. Basically the callback is only executed when message of approximate
same timestamp arrive which allows to compare poses more accurately.

If poses of another type have to compared, the callback and the type of the message filter subscriber has to be changed.
the way to run the node is to run the command 
rosrun tf_pose_comparator compare_node

This will subscribe two poses of the type and name "/ground_truth_pose", geometry_msgs::PoseStamped and
"/amcl_pose", geometry_msgs::PoseWithCovarianceStamped respectively. The topic names can be remapped in 
a launch file if that is the only modification required.