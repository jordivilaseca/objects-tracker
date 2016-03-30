#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <stdint.h>

#include <objects_tracker/utilities/utilities.hpp>

visualization_msgs::Marker buildMarker(std::string frame_id, int id, uint32_t type, double pos[], double scale[], int color[]);
visualization_msgs::Marker buildLineMarker(std::string frame_id, int id, std::vector< std::vector<double> > pos, double width, int color[]);
void buildLineMarkers(std::string frame_id, std::vector< std::vector< std::vector<double> > > positions, double width, std::vector<visualization_msgs::Marker> &markers);
void buildMarkers(std::string frame_id, uint32_t type, std::vector< std::vector< std::vector<double> > > positions, double scale[], std::vector<visualization_msgs::Marker> &markers);