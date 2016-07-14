#include <objects_tracker/segmentation/PlaneSegmentation.hpp>

/**
 * @brief PlaneSegmentation class constructor.
 * 
 * @param orientation Vector parallel to which a plane must be found.
 * @param angleThreshold Maximum desviation accepted of the plane orientation in degree.
 * @param planeDistance Distance up to a point is considered part of a plane. It must be positive.
 */
PlaneSegmentation::PlaneSegmentation(Eigen::Vector3f orientation, float angleThreshold, float planeDistance) {
	this->orientation = orientation;
	this->angleThreshold = angleThreshold*3.1415/180.0;
	this->planeDistance = planeDistance;
	maskCumulative.resize(size);
	maskIndices = pcl::PointIndices::Ptr(new pcl::PointIndices());
}

/**
 * @brief Class destructor.
 */
PlaneSegmentation::~PlaneSegmentation() {

}

/**
 * @brief Compute the coefficients of a plane given cloud. 
 * 
 * @param cloud Point cloud in which the computation is based.
 */
void PlaneSegmentation::computeCoefficients(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

	// Set segmentation parameters.
	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setOptimizeCoefficients(true);
	seg.setAxis(orientation);
	seg.setEpsAngle(angleThreshold); 
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(5000);
	seg.setDistanceThreshold(planeDistance);
	seg.setInputCloud(remainingCloud);
	seg.segment(*inliers, coefficients);
}

/**
 * @brief Update the mask used for the computation of the plane boundaries.
 * 
 * @param cloud Point cloud used to update the mask.
 */
void PlaneSegmentation::updateMask(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	// Initializate mask if necessary.
	if(maskCumulative.size() == 0) maskCumulative.resize(cloud->width * cloud->height);
	
	// Get plane inliers.
	pcl::IndicesPtr inliers;
	findPlaneInliers(cloud, coefficients, planeDistance, inliers);

	// Add new inliers and update old ones.
	for(int j = 0; j < inliers->size(); j++) {
		int p = (*inliers)[j];
		if(maskCumulative[p] == 0) {
			maskIndices->indices.push_back(p);
		}
		maskCumulative[p]++;
	}
}

/**
 * @brief Get the coefficients of the computated plane. The result is only valid if the computation of them is previously done.
 * 
 * @param [out] coefficients Coefficients of the plane.
 */
void PlaneSegmentation::getCoefficients(pcl::ModelCoefficients &coefficients) {
	coefficients = this->coefficients;
}

/**
 * @brief It computes de boundary of a cloud. The mask must have at least one element.
 * 
 * @param cloud Cloud in which the boundary are computed.
 */
void PlaneSegmentation::computeBoundary(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	
	// Compute normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimateNormals(cloud, normals, 0.015);	

	// Ignore points with a different normal.
	pcl::PointIndices::Ptr filtIndices(new pcl::PointIndices());
	filterByNormal(normals, maskIndices, coefficients, 15.0, filtIndices);

	// Project point cloud to a plane.
	pcl::ModelCoefficients::ConstPtr coefPtr = boost::make_shared<pcl::ModelCoefficients>(coefficients);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr projectedCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>());
	projectToPlane(cloud, coefPtr, projectedCloud);

	// Clustering.
	std::vector<pcl::PointIndices> clusterIndices;
	clustering(projectedCloud, filtIndices, 0.02, 5000, clusterIndices);

	assert(clusterIndices.size() > 0);

	// Find the biggest cluster.
	int max_size = clusterIndices[0].indices.size();
	int max_pos = 0;
	for(int i = 0; i < clusterIndices.size(); i++) {
		if (clusterIndices[i].indices.size() > max_size) {
			max_size = clusterIndices[i].indices.size();
			max_pos = i;
		}
	}

	pcl::PointIndices::ConstPtr clusterIndicesPtr = boost::make_shared<pcl::PointIndices>(clusterIndices[max_pos]);

	// Compute the convex hull of the cluster.
	pcl::PointIndices hullIndices = pcl::PointIndices();
	findConvexHull(projectedCloud, clusterIndicesPtr, hullIndices);
	pcl::PointIndices::ConstPtr hullIndicesPtr = boost::make_shared<pcl::PointIndices>(hullIndices);

	// Simplify convex polygon.
	pcl::PointIndices boundaryIndices;
	polygonSimplification(projectedCloud, hullIndicesPtr, coefficients.values, 4, boundaryIndices);

	// Copy boundary points.
	boundary = std::vector<pcl::PointXYZRGBA>(boundaryIndices.indices.size());
	for(int j = 0; j < boundary.size(); j++) {
		boundary[j] = cloud->points[boundaryIndices.indices[j]];
	}
}

/**
 * @brief Obtain the previously computed boundary.
 * 
 * @param boundaries The boundary of the plane.
 */
void PlaneSegmentation::getBoundary(std::vector<pcl::PointXYZRGBA> &boundary) const {
	boundary = this->boundary;
}

/**
 * @brief Set the coefficients of the plane.
 * 
 * @param coefficients plane coefficients.
 */
void PlaneSegmentation::setCoefficients(const pcl::ModelCoefficients &coefficients) {
	this->coefficients = coefficients;
}