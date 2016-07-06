#include <objects_tracker/segmentation/MultiplePlaneSegmentation.hpp>

MultiplePlaneSegmentation::MultiplePlaneSegmentation(int nPlanes, Eigen::Vector3f orientation, float angleThreshold, float planeDistance) {
	this->nPlanes = nPlanes;
	this->orientation = orientation;
	this->angleThreshold = angleThreshold*3.1415/180.0;
	this->planeDistance = planeDistance;

	planes.reserve(nPlanes);
	for(int i = 0; i < nPlanes; i++) {
		planes.push_back(PlaneSegmentation(orientation));
	}
}

MultiplePlaneSegmentation::~MultiplePlaneSegmentation() {

}

void MultiplePlaneSegmentation::computePlanes(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {

	// Cloud containing the points without the planes.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	// Create the segmentation object.
	pcl::SACSegmentation<pcl::PointXYZRGBA> seg;

	// Set segmentation parameters.
	seg.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
	seg.setOptimizeCoefficients(true);
	seg.setAxis(orientation);
	seg.setEpsAngle(angleThreshold); 
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setMaxIterations(5000);
	seg.setDistanceThreshold(planeDistance);

	// Create the filtering object.
	pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

	// At each step, one plane is removed from remainingCloud.
	for(int i = 0; i < nPlanes; i++){

		pcl::ModelCoefficients coefficients;
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices());

		// Segment the largest planar component from the remaining cloud.
		seg.setInputCloud(remainingCloud);
		seg.segment(*inliers, coefficients);

		// Remove non valid points.
		removeNans(cloud,  inliers);

		// Make sure the normal is looking to the camera.
		float origin[] = {0,0,0};
		correctNormal(origin, cloud->points[inliers->indices[0]], coefficients);

		// With 0 inliers, nothing needs to be done.
		if (inliers->indices.size() == 0) break;

		// Extract the plane inliers from the remainingCloud.
		extract.setInputCloud(remainingCloud);
		extract.setIndices(inliers);
		extract.setNegative(true);
		extract.filter(*remainingCloud);

		// Update plane coefficients.
		planes[i].setCoefficients(coefficients);
	}
}

void MultiplePlaneSegmentation::updateMasks(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	for(int i = 0; i < nPlanes; i++) {
		planes[i].updateMask(cloud);
	}
}

void MultiplePlaneSegmentation::getCoefficients(std::vector<pcl::ModelCoefficients> &coefficients) {
	coefficients.resize(nPlanes);

	for(int i = 0; i < nPlanes; i++) {
		planes[i].getCoefficients(coefficients[i]);
	}
}

void MultiplePlaneSegmentation::computeBoundaries(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud) {
	for(int i = 0; i < nPlanes; i++) {
		planes[i].computeBoundary(cloud);
	}
}

void MultiplePlaneSegmentation::getBoundaries(std::vector<std::vector<pcl::PointXYZRGBA>> &boundaries) const {
	boundaries.resize(nPlanes);

	for(int i = 0; i < nPlanes; i++) {
		planes[i].getBoundary(boundaries[i]);
	}
}

void MultiplePlaneSegmentation::segment(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, std::vector<pcl::PointIndices> &clusterIndices) {
	
	std::vector<pcl::ModelCoefficients> coefficients;
	getCoefficients(coefficients);

	std::vector<std::vector<pcl::PointXYZRGBA>> boundaries;
	getBoundaries(boundaries);

	// std::cout << "remove_planes in" << std::endl;
	// Cloud containing the points without the planes.
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr remainingCloud = pcl::PointCloud<pcl::PointXYZRGBA>::Ptr(new pcl::PointCloud<pcl::PointXYZRGBA>(*cloud));

	// -1 -> part of a plane, 0 -> not part of an object, 1 -> part of an object.
	std::vector<char> mask = std::vector<char>(cloud->points.size(), 0);

	// std::cout << "pepe1" << endl;
	assert(coefficients.size() == boundaries.size());
	for(int i = 0; i < coefficients.size(); i++) {

		Eigen::Vector4f planeCoef = Eigen::Vector4f(coefficients[i].values.data());
		std::vector<pcl::PointXYZRGBA> planeBoundary = boundaries[i];

		#pragma omp parallel for firstprivate(planeCoef, planeBoundary) shared(cloud, mask) num_threads(4)
		for(size_t j = 0; j < cloud->points.size(); j++) {
			// Calculate the distance from the point to the plane normal as the dot product
			// D =(P-A).N/|N|

			// If the x value of the pointcloud or it is marked as a point in a plane it is not needed to
			// make further calculations, we don't want this point.
			if(isnan(cloud->points[j].x) or mask[j] == -1) continue;
			// std::cout << "pepess1" << std::endl;

			Eigen::Vector4f pt(cloud->points[j].x, cloud->points[j].y, cloud->points[j].z, 1);
			// std::cout << "pepess2" << std::endl;
			float distance = planeCoef.dot(pt);
			// std::cout << "pepess3" << std::endl;
			if (distance >= -0.02) {
				if (isInlier(cloud, j , planeBoundary, planeCoef)) {
					if (distance <= 0.02) {
						// If the point is at a distance less than X, then the point is in the plane, we mark it properly.
						mask[j] = -1;
					} else {
						// The point is not marked as being part of an object nor plane, if it is above it we mark it as object.
						mask[j] = 1;
					}
				}
			}
		}
	}

	// Parse inliers.
	pcl::PointIndices::Ptr inliers = pcl::PointIndices::Ptr(new pcl::PointIndices());
	inliers->indices.resize(cloud->points.size());
	int nr_p = 0;
	for(int i = 0; i < mask.size(); i++) {
		if(mask[i] == 1) inliers->indices[nr_p++] = i;
	}
	inliers->indices.resize(nr_p);

	// Clustering
	clusterIndices = std::vector<pcl::PointIndices>();
	clustering(cloud, inliers, 0.03, 200, clusterIndices);
}