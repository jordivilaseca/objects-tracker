#include <objects_tracker/recognition/Recognition.hpp>

int Recogniser::getNumObjects() const {
	int n = (objectsResults.size() > 0) ? 1 : 0;

	for (int i = 1; i < objectsResults.size(); i++) {
		if(objectsResults[i-1] != objectsResults[i]) n++;
	}

	return n;
}

std::vector<std::string> Recogniser::getObjects() const {
  std::vector<std::string> header(objectsResults.size());

  if(objectsResults.size() == 0) return header;

  header[0] = objectsResults[0];
  int nelem = 1;
  for(int i = 1; i < objectsResults.size(); i++) {
    if(objectsResults[i-1] != objectsResults[i]) header[nelem++] = objectsResults[i];
  }
  header.resize(nelem);

  return header;
}

uint Recogniser::getDSize(DTYPE d) {
	switch(d) {
		case COLOR:
			return hBins*sBins;
			break;

		case SHAPE:
			return 308;
			break;

		case BOTH:
			return 308 + hBins*sBins;
			break;
	}
}

Recogniser::Recogniser(DTYPE d) : objects(), objectsResults(), objectsIndices(), vocabulary(), matcher(new cv::BFMatcher(cv::NORM_L1)), model(cv::ml::SVM::create()), dtype(d) {
	dSize = getDSize(dtype);
}
Recogniser::Recogniser(cv::DescriptorMatcher *matcher, DTYPE d) : objects(), objectsResults(), objectsIndices(), vocabulary(), matcher(), model(), dtype(d) {
	dSize = getDSize(dtype);
	*(this->matcher) = *matcher;
}

Recogniser::~Recogniser() {
	delete matcher;
}

void Recogniser::setDescriptor(DTYPE d) {
	dtype = d;
	dSize = getDSize(d);
}

uint Recogniser::getHBins() {
	return hBins;
}

uint Recogniser::getSBins() {
	return sBins;
}

std::vector<std::string> Recogniser::getTrainingNames() {
	return objectsNames;
}

void Recogniser::setHBins(uint bins) {
	hBins = bins;
	this->setDescriptor(dtype);
}

void Recogniser::setSBins(uint bins) {
	sBins = bins;
	this->setDescriptor(dtype);
}

void Recogniser::computeDescriptors(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices &indices, cv::Mat &descriptors) const {
	// Estimate normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimateNormals(cloud, normals, normalEstimationDist);

	// Initialize KdTree.
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGBA>());
	pcl::IndicesPtr indicesP = boost::make_shared<std::vector<int>>(indices.indices);
	tree->setInputCloud(cloud, indicesP);

	// Compute clusters.
	std::vector<pcl::PointIndices> clusters;
	regionGrowing(cloud, indicesP, normals, tree, RGAngle / 180.0 * M_PI, RGCurvature, RGNumNeighbours, RGMinClusterSize, clusters);

	// Initialize descriptors.
	descriptors = cv::Mat(clusters.size(), dSize, CV_32F, 0.0);

	// Compute the descriptors for each cluster of the object.
	for (int i = 0; i < clusters.size(); i++) {
		pcl::PointIndices::Ptr regionIndices = boost::make_shared<pcl::PointIndices>(clusters[i]);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr pclDesc(new pcl::PointCloud<pcl::VFHSignature308>());
		cv::Mat cvDesc;

		// Compute VFH descriptors.
		if(dtype == BOTH or dtype == SHAPE) {
			vfh(cloud, normals, regionIndices, tree, pclDesc);
		}

		if (dtype == BOTH or dtype == COLOR) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr region(new pcl::PointCloud<pcl::PointXYZRGBA>());
			extractIndices(cloud, regionIndices, region);
	  		subPointCloud(region, *regionIndices);

			// Point cloud to image.
			cv::Mat image;
			cv::Mat mask;
			pointcloud2mat(*region, image, mask);
			cvtColor(image, image, cv::COLOR_BGR2HSV);

			// Compute color histogram.
			float incPerValue = totalSumHisto/regionIndices->indices.size();
			calcHsvHist(image, mask, hBins, sBins, incPerValue, cvDesc);
		}

		// Join descriptors.
		int j = 0;
		if (pclDesc->points.size() == 1) {
			for (float elem : pclDesc->points[0].histogram) {
				descriptors.at<float>(i,j++) = elem;
			}
		}
		for (int k = 0; k < cvDesc.total(); k++) {
			descriptors.at<float>(i, j++) = cvDesc.at<float>(k);
		}
	}
}

void Recogniser::addObjects(const std::vector<pcl::PointCloud<pcl::PointXYZRGBA>::Ptr> &objects, const std::vector<std::string> &objectsResults, const std::vector<pcl::PointIndices> &objectsIndices) {
	assert(objects.size() == objectsResults.size() and objectsResults.size() == objectsIndices.size());
	this->objects = objects;
	this->objectsResults = objectsResults;
	this->objectsIndices = objectsIndices;
	this->objectsNames = this->getObjects();
}

void Recogniser::computeAllDescriptors() {
	// Compute descriptors
	std::cout << "Computing objects descriptors... " << std::flush;
	objectDescriptors = std::vector<cv::Mat>(); // objectDescriptors[i][j] = descriptor of object i, cluster j.
	for (int i = 0; i < objects.size(); i++) {
		cv::Mat currentDescriptors;
		computeDescriptors(objects[i], objectsIndices[i], currentDescriptors);
		objectDescriptors.push_back(currentDescriptors);
	}
	std::cout << "ended" << std::endl;

	assert(objectDescriptors.size() > 0);
	assert(objectDescriptors[0].cols > 0);
}

void Recogniser::computeAll() {
	computeAllDescriptors();
	computeModel();
}

void Recogniser::computeModel() {
	// Copy descriptors to Mat.
	cv::Mat descriptors;
	cv::vconcat(objectDescriptors, descriptors);

	// Compute vocabulary of BoW.
	std::cout << "Computing vocabulary... " << std::flush;
	int nObjs = getNumObjects();
	cv::BOWKMeansTrainer trainer(clustersPerObject*nObjs);
	vocabulary = trainer.cluster(descriptors);
	std::cout << "ended" << std::endl;

	// Train matcher.
	std::cout << "Training matcher... " << std::flush;
	std::vector<cv::Mat> vocabularyVector = {vocabulary};
	matcher->clear();
	matcher->add(vocabularyVector);
	matcher->train();
	std::cout << "ended" << std::endl;

	// Compute model descriptors.
	std::cout << "Computing model descriptors... " << std::flush;
	cv::Mat modelDescriptors(objectDescriptors.size(), vocabulary.rows, CV_32FC1, cv::Scalar(0));
	int n = 0;
	for (const cv::Mat &m : objectDescriptors) {
		std::vector<cv::DMatch> matches;
		matcher->match(m, matches);

		for (const cv::DMatch &match : matches) {
			modelDescriptors.at<float>(n, match.trainIdx)++;
		}
		n++;
	}
	std::cout << "ended" << std::endl;

	// Calculate responses mat.
	cv::Mat responses(objectDescriptors.size(),1, CV_32SC1, cv::Scalar(0));
	n = 0;
	for (int i = 1; i < objectsResults.size(); i++) {
		if (objectsResults[i-1] != objectsResults[i]) n++;
		responses.at<int>(i,0) = n;
	}

	// Train model.
	std::cout << "Training model... " << std::flush;
	model->clear();
	cv::Ptr<cv::ml::TrainData> tdata = cv::ml::TrainData::create(modelDescriptors, cv::ml::ROW_SAMPLE, responses);
	model->trainAuto(tdata,5);
	std::cout << "ended" << std::endl;
}

void Recogniser::read(const std::string &path) {
	model = cv::Algorithm::load<cv::ml::SVM>(path + "/" + MODEL_NAME);
	readList(objectsNames, path + "/" + NAMES_NAME);

	cv::FileStorage fs(path + "/" + VOCABULARY_NAME, cv::FileStorage::READ);
	fs["Vocabulary"] >> vocabulary;
	fs.release();

	cv::FileStorage d_fs(path + "/" + DESCRIPTOR_NAME, cv::FileStorage::READ);
	int d;
    d_fs["Descriptor"] >> d;
    d_fs["HBins"] >> hBins;
    d_fs["SBins"] >> sBins;
    d_fs.release();
    this->setDescriptor( static_cast<DTYPE>(d));

	std::vector<cv::Mat> vocabularyVector = {vocabulary};
	matcher->add(vocabularyVector);
	matcher->train();
}

void Recogniser::write(const std::string &path) const {
	model->save(path + "/" + MODEL_NAME);
	writeList(objectsNames, path + "/" + NAMES_NAME);

	cv::FileStorage v_fs(path + "/" + VOCABULARY_NAME, cv::FileStorage::WRITE);
    v_fs << "Vocabulary" << vocabulary;
    v_fs.release();

    cv::FileStorage d_fs(path + "/" + DESCRIPTOR_NAME, cv::FileStorage::WRITE);
    d_fs << "Descriptor" << (int) dtype;
    d_fs << "HBins" << (int) hBins;
    d_fs << "SBins" << (int) sBins;
    d_fs.release();
}

std::string Recogniser::predict(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &object, const pcl::PointIndices &indices) const {
	// Compute descriptors.
	cv::Mat descriptors;
	computeDescriptors(object, indices, descriptors);

	// Compute matches.
	std::vector<cv::DMatch> matches;
	matcher->match(descriptors, matches);

	// Compute model Descriptor.
	cv::Mat modelDescriptor(1, vocabulary.rows, CV_32FC1, cv::Scalar(0.0));
	for (const cv::DMatch &match : matches) {
		modelDescriptor.at<float>(0,match.trainIdx)++;
	}

	// Predict.
	cv::Mat outputs;
	cv::Mat probs;
	int index = (int) model->predict(modelDescriptor, outputs);
	return objectsNames[(int) outputs.at<float>(0)];
}