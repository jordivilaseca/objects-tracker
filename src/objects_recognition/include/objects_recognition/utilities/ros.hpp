#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <stdint.h>

#include <objects_tracker/utilities/utilities.hpp>

double default_ori[] = {0.0, 0.0, 0.0, 1.0};
double default_points[][3] = {};

visualization_msgs::Marker buildMarker(std::string frame_id, int id, uint32_t type, double pos[], double scale[], double color[], double ori[] = default_ori, double points[][3] = default_points);
visualization_msgs::Marker buildText(std::string frame_id, int id, double pos[], double height, std::string text);
visualization_msgs::Marker buildLineMarker(std::string frame_id, int id, std::vector< std::vector<double> > pos, double width, double color[]);
void buildLineMarkers(std::string frame_id, std::vector< std::vector< std::vector<double> > > positions, double width, std::vector<visualization_msgs::Marker> &markers);
void buildMarkers(std::string frame_id, uint32_t type, std::vector< std::vector< std::vector<double> > > positions, double scale[], std::vector<visualization_msgs::Marker> &markers);