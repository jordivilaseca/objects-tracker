#include <objects_tracker/utilities/ros.hpp>

visualization_msgs::Marker buildMarker(std::string frame_id, int id, uint32_t type, double pos[], double scale[], double color[]) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = type;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.position.x = pos[0];
	marker.pose.position.y = pos[1];
	marker.pose.position.z = pos[2];
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = scale[0];
	marker.scale.y = scale[1];
	marker.scale.z = scale[2];
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.color.a = color[3]; // Don't forget to set the alpha!
	return marker;
}