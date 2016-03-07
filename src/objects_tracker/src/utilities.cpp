#include <objects_tracker/utilities.hpp>

long long getTime(){
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return mslong;
}

void colorPoint(pcl::PointXYZRGBA &point, int r, int g, int b) {
	point.r = r;
	point.g = g;
	point.b = b;
}

void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const boost::shared_ptr<std::vector<int> > &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers->size(); i++) {
		int p = (*inliers)[i];
		colorPoint(cloud->points[p], r, g, b);
	}
}

void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<int> &inliers, uint8_t r, uint8_t g, uint8_t b) {
	for(uint i = 0; i < inliers.size(); i++) {
		int p = inliers[i];
		colorPoint(cloud->points[p], r, g, b);
	}
}

void colorPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &cloud, const std::vector<pcl::PointIndices> &inliers) {
	int numIn = inliers.size();
	int step = (256*4)/numIn;
	int r, g, b;
	int i = 0;

	// Red and blue are fixed, incrementing green from 0 to 255.
	r = 255;
	g = 0;
	b = 0;
	for(; g <= 255 and i < numIn; g += step) {
		colorPointCloud(cloud, inliers[i].indices, r, g, b);
		i++;
	}

	// Green and blue are fixed, decrementing red from 255 to 0.
	r = 255 - (g - 256);
	g = 255;
	for(; r >= 0 and i < numIn; r -= step) {
		colorPointCloud(cloud, inliers[i].indices, r, g, b);
		i++;
	}

	// Red and green fixed, incrementing blue from 0 to 255.
	b = -r;
	r = 0;
	for(; b <= 255 and i < numIn; b += step) {
		colorPointCloud(cloud, inliers[i].indices, r, g, b);
		i++;
	}

	// Red and blue fixed, decrementing green from 255 to 0.
	g = 255 - (b - 256);
	b = 255;
	for(; g >= 0 and i < numIn; g -= step) {
		colorPointCloud(cloud, inliers[i].indices, r, g, b);
		i++;
	}

	int remaining = inliers.size() - i;
	if(remaining != 0) {
		ROS_WARN("colorPointCloud: Not all the regions have been painted, there are %i regions not painted.", remaining);
	}
}