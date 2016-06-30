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
#include <pcl/segmentation/region_growing.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <pcl/features/normal_3d.h>
#include <pcl/features/cvfh.h>
#include <pcl/features/our_cvfh.h>
#include <pcl/features/shot.h>

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
pcl::PointXYZRGBA createXYZRGBA(double x, double y, double z);
double triangleArea(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r);
double vectAngle3dEmbeddedPlane(const pcl::PointXYZRGBA &p, const pcl::PointXYZRGBA &q, const pcl::PointXYZRGBA &r, const std::vector<float> &n);
int orient3d(const pcl::PointXYZRGBA &pa, const pcl::PointXYZRGBA &pb, const pcl::PointXYZRGBA &pc, const pcl::PointXYZRGBA &pd);
void correctNormal(const float origin[], const pcl::PointXYZRGBA &p, pcl::ModelCoefficients &coef);

// Polygon functions;
void orderConvexPolygon(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const std::vector<float> &n, pcl::PointIndices &polygon);
bool isInlier(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, int point, const std::vector<pcl::PointXYZRGBA> &polygon, const Eigen::Vector4f &n);
void polygonSimplification(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &polygon, const std::vector<float> &n, int maxPoints, pcl::PointIndices &simPolygon);
void polygonCenter(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<double> &center);

// Point cloud functions.
void findLines(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, std::vector<pcl::ModelCoefficients> &coef);
void findConvexHull(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, pcl::PointIndices &hullIndices);
void regionGrowing(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::IndicesPtr &indices, const pcl::PointCloud<pcl::Normal>::Ptr &normals, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, double angleThresh, double curvatureThresh, int nNeigh, int minClusSize, std::vector<pcl::PointIndices> &clusters);
void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, float tolerance, int minSize, std::vector<pcl::PointIndices> &clusterIndices);
void clustering(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices::ConstPtr &inputIndices, float tolerance, int minSize, std::vector<pcl::PointIndices> &clusterIndices);
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double thresh);
void estimateNormals(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::PointCloud<pcl::Normal>::Ptr &normals, double thresh);
void extractIndices(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &input, const pcl::PointIndices::ConstPtr &inliers, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud);
void subPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, pcl::PointIndices &indices);
void filterByNormal(const pcl::PointCloud<pcl::Normal>::ConstPtr cloud, const pcl::PointIndices::ConstPtr &input, const pcl::ModelCoefficients &planeCoefficients, float angleThresh, pcl::PointIndices::Ptr & output);
void removeNans(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud, pcl::PointIndices::Ptr &input);

// Plane functions
void findPlaneInliers(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::ModelCoefficients &modelCoef, float threshold, pcl::IndicesPtr &inliers);
void projectToPlane(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, pcl::ModelCoefficients::ConstPtr modelCoef, pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &projCloud);

// Descriptors functions.

void vfh(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &object, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::PointIndices::Ptr &indices, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptor);
void cvfh(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &object, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, double angleThresh, double curvatureThresh, bool scaleInvariant, pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors);
void ourcvfh(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &object, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, const pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr &tree, double angleThresh, double curvatureThresh, bool scaleInvariant, pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors);
void shot352(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &object, const pcl::PointCloud<pcl::Normal>::ConstPtr &normals, float radius, pcl::PointCloud<pcl::VFHSignature308>::Ptr &descriptors);