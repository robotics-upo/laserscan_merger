# Laserscan_merger

A simple ROS node to fuse different lasercan sources in a single laserscan message in a given frame. 

This node is primary inspired by the lasercan_multi_merger node of the [ira_laser_tools](http://www.ros.org/wiki/ira_laser_tools) package. 
This package includes two tools for laser handling in ROS:

 - laserscan_merger
 - republish_scans

**Laserscan_merger** allows to easily and dynamically (rqt_reconfigure) merge multiple,
single scanning plane, laser scans into a single one; and therefore, transformed to a given target frame.

**republish_scans** allows to unify the frequency of publication of multiple lasercan topics. This is done by republishing the scans
messages in new topics (with the suffix *_throttle*) at the indicated frequency. 

Both nodes compile under catkin in Melodic/Noetic ROS distros.


## Laserscan_merger Parameters

* **General Parameters**

	- *cloud_destination_topic*. Name of the output topic with the merged laserscans transformed to a pointcloud (Def: */merged_cloud*).
	- *scan_destination_topic*. Name of the output topic with the merged laserscans (Def: */scan_merged*).
	- *laserscan_topics*. String with the names of the input scan topics separated by a blank spaces. (Ex: "/rgbd_scan /scan_raw /rear_scan").
	- *destination_frame*. Target frame in which all the scans will be transformed.
	
* **laserscan Parameters**

The parameters of the output laserscan message. See the message documentation [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html).

	- max_update_time
	- angle_min
	- angle_max
	- angle_increment
	- scan_time
	- range_min
	- range_max
	 
## Lasercan_merger subscriptions

- To all the scan topics indicated in the string *laserscan_topics*.

	
## Lasercan_merger Publications

- *merged_cloud*. Pointcloud2 with the points of the merged laserscans. (Topic name can be modified by param).
- *scan_merged*. Laserscan with the fused laserscans. (Topic name can be modified by param).


## Republish_scans Parameters

- *publish_frequency*. Frequency (Hz) in which the laserscans will be published. 
- *scan_topic_X*. Name of a input scan topic in which the *X* must be replaced by the number of input scan (Ex: scan_topic_1, scan_topic_2, scan_topic_3, etc). 
	
## Republish_scans subscriptions

- To all the scans topics indicated by the parameters *scan_topic_X*. 
	
## Republish_scans publications

- It publishes the same number of input topics in new topics by adding the suffix *_throttle*. 




