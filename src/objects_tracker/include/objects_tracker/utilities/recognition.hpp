#include <objects_tracker/utilities/utilities.hpp>
#include <objects_tracker/utilities/bridge.hpp>
#include <objects_tracker/utilities/pcl.hpp>
#include <objects_tracker/utilities/opencv.hpp>

// pcl includes.
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// opencv includes.
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/ml.hpp>

class Recogniser
{

public:
	enum DTYPE {COLOR, SHAPE, BOTH};

	int clustersPerObject = 10;
	float normalEstimationDist = 0.03;
	float RGAngle = 5.0;
	float RGCurvature = 1.0;
	int RGNumNeighbours = 40;
	int RGMinClusterSize = 50;
	float totalSumHisto = 500;
	
	Recogniser(DTYPE d = DTYPE::BOTH);
	Recogniser(cv::DescriptorMatcher *matcher, DTYPE d = DTYPE::BOTH);
	~Recogniser();
	void computeAll();
	void computeAllDescriptors();
	void setDescriptor(DTYPE d);
	uint getHBins();
	uint getSBins();
	void setHBins(uint bins);
	void setSBins(uint bins);
	std::vector<std::string> getTrainingNames();

	void computeDescriptors(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices &indices, cv::Mat &descriptors) const;
	void addObjects(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &objects, const std::vector<std::string> &objectsResults, const std::vector<pcl::PointIndices> &objectsIndices);
	void computeModel();
	void read(const std::string &path);
	void write(const std::string &path) const;
	std::string predict(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &object, const pcl::PointIndices &indices) const;

private:
	const std::string MODEL_NAME = "model.yml";
	const std::string MATCHER_NAME = "matcher.yml";
	const std::string NAMES_NAME = "names";
	const std::string VOCABULARY_NAME = "vocabulary.yml";
	const std::string DESCRIPTOR_NAME = "descriptor.yml";

	std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> objects;
	std::vector<std::string> objectsResults;
	std::vector<std::string> objectsNames;
	std::vector<pcl::PointIndices> objectsIndices;
	cv::Mat vocabulary;
	DTYPE dtype;
	uint dSize;
	int hBins = 13;
	int sBins = 6;

	std::vector<cv::Mat> objectDescriptors;

	cv::DescriptorMatcher *matcher;
	cv::Ptr<cv::ml::SVM> model;

	int getNumObjects() const;
	std::vector<std::string> getObjects() const;
	uint getDSize(DTYPE d);
};