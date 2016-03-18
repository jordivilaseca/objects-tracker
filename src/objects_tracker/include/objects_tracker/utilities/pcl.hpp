#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/common/distances.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/project_inliers.h>

// std
#include <queue>
#include <stack>
#include <sys/time.h>

#include <ros/ros.h>

#include <omp.h>

#include <objects_tracker/utilities/utilities.hpp>

long long getTime();

// Colour point cloud.
void colourPoint(pcl::PointXYZRGBA &point, int r, int g, int b);
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const boost::shared_ptr<std::vector<int> > &inliers, uint8_t r, uint8_t g, uint8_t b);
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices &inliers, uint8_t r, uint8_t g, uint8_t b);
void colourPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<pcl::PointIndices> &inliers);
void colourLine(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, const pcl::ModelCoefficients &coef, double minDis, int r, int g, int b);

// Point functions.
double triangleArea(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r);
double vectAngle3dEmbeddedPlane(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r, const std::vector<float> &n);

// Polygon functions;
void orderConvexPolygon(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::vector<float> &n, pcl::PointIndices &polygon);
void polygonSimplification(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &polygon, const std::vector<float> &n, int maxPoints, pcl::PointIndices &simPolygon);

// Point cloud functions.
void findLines(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::ModelCoefficients> &coef);
void findConvexHull(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, pcl::PointIndices &hullIndices);
void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::PointIndices> &clusterIndices);

// Plane functions
void findPlaneInliers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients modelCoef, float dist, pcl::IndicesPtr &inliers);
void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients::ConstPtr modelCoef, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &projCloud);