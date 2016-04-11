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

visualization_msgs::Marker buildLineMarker(std::string frame_id, int id, std::vector< std::vector<double> > pos, double width, double color[]) {
	visualization_msgs::Marker marker;
	marker.header.frame_id = frame_id;
	marker.header.stamp = ros::Time();
	marker.ns = "my_namespace";
	marker.id = id;
	marker.type = visualization_msgs::Marker::LINE_STRIP;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = width;
	marker.color.r = color[0];
	marker.color.g = color[1];
	marker.color.b = color[2];
	marker.color.a = color[3]; // Don't forget to set the alpha!

	// Set points
	for(int i = 0; i < pos.size(); i++) {
		geometry_msgs::Point p;
		p.x = pos[i][0];
		p.y = pos[i][1];
		p.z = pos[i][2];
 
 		marker.points.push_back(p);
	}
	return marker;
}

void buildLineMarkers(std::string frame_id, std::vector< std::vector< std::vector<double> > > positions, double width, std::vector<visualization_msgs::Marker> &markers) {
	markers = std::vector<visualization_msgs::Marker>();

	std::vector< std::vector<double> > colours;
	computeColors(positions.size(), colours);
	int markerid = 0;
	for(int i = 0; i < colours.size(); i++) {
		double colour[] = {colours[i][0], colours[i][1], colours[i][2], 255};
		markers.push_back(buildLineMarker(frame_id, markerid, positions[i], width, colour));
		markerid++;
	}
}

void buildMarkers(std::string frame_id, uint32_t type, std::vector< std::vector< std::vector<double> > > positions, double scale[], std::vector<visualization_msgs::Marker> &markers) {
	markers = std::vector<visualization_msgs::Marker>();

	std::vector< std::vector<double> > colours;
	computeColors(positions.size(), colours);
	int markerid = 0;
	for(int i = 0; i < colours.size(); i++) {
		for(int j = 0; j < colours[i].size(); j++) {
			double colour[] = {colours[i][0], colours[i][1], colours[i][2], 255};
			double pos[] = {positions[i][j][0], positions[i][j][1], positions[i][j][2]};
			markers.push_back(buildMarker(frame_id, markerid, type, pos, scale, colour));
			markerid++;
		}
	}
}