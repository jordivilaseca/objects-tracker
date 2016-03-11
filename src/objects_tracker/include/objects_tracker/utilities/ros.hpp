#include <ros/ros.h>
#include <visualization_msgs/Marker.h>

#include <string>
#include <stdint.h>

visualization_msgs::Marker buildMarker(std::string frame_id, int id, uint32_t type, double pos[], double scale[], double color[]);