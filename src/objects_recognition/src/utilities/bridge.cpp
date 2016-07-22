#include <objects_tracker/utilities/bridge.hpp>

using namespace cv;

/* It only works with organized point clouds*/
/**
 * @brief It converts an organized point cloud to an image. The valid points (points that do not have a nan as x coordinate) are copied to the image and
 * marked as valids in the mask.
 * 
 * @param cloud Point cloud to convert.
 * @param [out] image Obtained image.
 * @param [out] mask Mask of valid points of the image.
 */
void pointcloud2mat(const pcl::PointCloud<pcl::PointXYZRGBA> &cloud, cv::Mat &image, cv::Mat &mask) {
	image = cv::Mat(cloud.height, cloud.width, CV_8UC3, cv::Scalar(255, 255, 255));
	mask = cv::Mat(cloud.height, cloud.width, CV_8UC1, cv::Scalar(0));

	// Copy point cloud to image.
	for(int i = 0; i < cloud.height; i++) {
		for(int j = 0; j < cloud.width; j++) {
			pcl::PointXYZRGBA p = cloud.at(i*cloud.width + j);
			if (!isnan(p.x)) {
				mask.at<uchar>(i,j) = 255;

				cv::Vec3b color(p.b, p.g, p.r);
				image.at<cv::Vec3b>(i,j) = color;
			}
		}
	}
	cv::flip(image, image,-1);
	cv::flip(mask, mask,-1);
}