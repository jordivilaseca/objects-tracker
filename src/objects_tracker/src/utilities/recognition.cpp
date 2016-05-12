#include <objects_tracker/utilities/recognition.hpp>

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
			return COLOR_NBINS*COLOR_NBINS*COLOR_NBINS;
			break;

		case SHAPE:
			return 308;
			break;

		case BOTH:
			return 308 + COLOR_NBINS*COLOR_NBINS*COLOR_NBINS;
			break;
	}
}

Recogniser::Recogniser(DTYPE d) : objects(), objectsResults(), objectsIndices(), vocabulary(), matcher(new cv::FlannBasedMatcher()), model(cv::ml::SVM::create()), dtype(d) {
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

void Recogniser::computeDescriptors(const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud, const pcl::PointIndices &indices, cv::Mat &descriptors) const {

	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudCopy(new pcl::PointCloud<pcl::PointXYZRGB>());
	pcl::copyPointCloud(*cloud, *cloudCopy);

	// Estimate normals.
	pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>());
	estimateNormals(cloudCopy, normals, normalEstimationDist);

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
		pcl::PointIndices::Ptr regionIndicies = boost::make_shared<pcl::PointIndices>(clusters[i]);
		pcl::PointCloud<pcl::VFHSignature308>::Ptr pclDesc(new pcl::PointCloud<pcl::VFHSignature308>());
		cv::Mat cvDesc;

		// Compute VFH descriptors.
		if(dtype == BOTH or dtype == SHAPE) {
			vfh(cloud, normals, regionIndicies, tree, pclDesc);
		}

		if (dtype == BOTH or dtype == COLOR) {
			pcl::PointCloud<pcl::PointXYZRGBA>::Ptr region(new pcl::PointCloud<pcl::PointXYZRGBA>());
			extractIndices(cloud, regionIndicies, region);
	  		subPointCloud(region, *regionIndicies);

			// Point cloud to image.
			cv::Mat image;
			cv::Mat mask;
			pointcloud2mat(*region, image, mask);

/*			cv::namedWindow("mask" + std::to_string(i), CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
			cv::namedWindow("image" + std::to_string(i), CV_WINDOW_AUTOSIZE|CV_WINDOW_FREERATIO);
			cv::imshow("image" + std::to_string(i), image);
	        cv::waitKey(250);
	        cv::imshow("mask" + std::to_string(i), mask);
	        cv::waitKey(250);*/

			// Compute color histogram.
			calcRgbHist(image, mask, COLOR_NBINS, cvDesc);
			cv::normalize(cvDesc, cvDesc, 0, 10, cv::NORM_MINMAX, -1, cv::Mat());
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
}

void Recogniser::computeModel() {
	assert(objects.size() > 0);

	objectsNames = getObjects();
	
	// Compute descriptors
	std::cout << "Computing objects descriptors... " << std::flush;
	std::vector<cv::Mat> objectDescriptors; // objectDescriptors[i][j] = descriptor of object i, cluster j.
	for (int i = 0; i < objects.size(); i++) {
		cv::Mat currentDescriptors;
		computeDescriptors(objects[i], objectsIndices[i], currentDescriptors);
		objectDescriptors.push_back(currentDescriptors);
	}
	std::cout << "ended" << std::endl;

	assert(objectDescriptors.size() > 0);
	assert(objectDescriptors[0].cols > 0);

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

	// cv::Mat outputs;//(1, vocabulary.rows, CV_32F, cv::Scalar(0));
	// cv::Mat probs;//(1, vocabulary.rows, CV_32F, cv::Scalar(0));
	// model->predictProb(descriptors.row(0), outputs, probs);
	// std::cout << std::endl << outputs << std::endl << probs << std::endl;
	// std::cout << "asdf " << (int) outputs.at<int>(0) << std::endl;

}

void Recogniser::read(const std::string &path) {
	model = cv::Algorithm::load<cv::ml::SVM>(path + "/" + MODEL_NAME);
	readList(objectsNames, path + "/" + NAMES_NAME);

	cv::FileStorage fs(path + "/" + VOCABULARY_NAME, cv::FileStorage::READ);
	fs["Vocabulary"] >> vocabulary;
	fs.release();

	std::vector<cv::Mat> vocabularyVector = {vocabulary};
	matcher->add(vocabularyVector);
	matcher->train();
}

void Recogniser::write(const std::string &path) const {
	model->save(path + "/" + MODEL_NAME);
	writeList(objectsNames, path + "/" + NAMES_NAME);

	cv::FileStorage fs(path + "/" + VOCABULARY_NAME, cv::FileStorage::WRITE);
    fs << "Vocabulary" << vocabulary;
    fs.release();
}

std::string Recogniser::predict(const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr &object, const pcl::PointIndices &indices) const {
	// Compute descriptors.
	cv::Mat descriptors;
	computeDescriptors(object, indices, descriptors);

	// Compute matches.
	std::vector<cv::DMatch> matches;
	matcher->match(descriptors, matches);

	// Compute model Descriptor.
	// std::cout << vocabulary << std::endl;
	cv::Mat modelDescriptor(1, vocabulary.rows, CV_32FC1, cv::Scalar(0.0));
	// std::cout << "pepe " << vocabulary.rows << std::endl;
	for (const cv::DMatch &match : matches) {
		// std::cout << match.trainIdx << " " << std::flush;
		modelDescriptor.at<float>(0,match.trainIdx)++;
	}
	// Predict.
	cv::Mat outputs;//(1, vocabulary.rows, CV_32F, cv::Scalar(0));
	cv::Mat probs;//(1, vocabulary.rows, CV_32F, cv::Scalar(0));
	int index = (int) model->predict(modelDescriptor, outputs);
	// std::cout << modelDescriptor << std::endl << outputs << std::endl << probs << std::endl;
	// std::cout << "asdf " << (int) outputs.at<int>(0) << std::endl;
	return objectsNames[(int) outputs.at<float>(0)];

	/*int N=12;
	cv::Ptr<cv::ml::NormalBayesClassifier> nb = cv::ml::NormalBayesClassifier::create();
	cv::Mat_<float> X(N,4);
    X << 1,2,3,4,  1,2,3,4,   1,2,3,4,    1,2,3,4,
         5,5,5,5,  5,5,5,5,   5,5,5,5,    5,5,5,5,
         4,3,2,1,  4,3,2,1,   4,3,2,1,    4,3,2,1;

    std::cout << "Read X" << std::endl;
    // labels:
    cv::Mat_<int> Y(N,1);
    Y << 0,0,0,0, 1,1,1,1, 2,2,2,2;
    nb->train(X, cv::ml::ROW_SAMPLE, Y);

    std::cout << "Single prediction" << std::endl;
    // single prediction:
    cv::Mat R1,P1;
    for (int i=0; i<N; i++)
    {
        cv::Mat r,p;
        nb->predictProb(X.row(i), r, p);
        R1.push_back(r);
        P1.push_back(p);
    }
    std::cout << R1 << P1 << std::endl;

    std::cout << "bulk prediction" << std::endl;

    // bulk prediction:
    cv::Mat R2,P2;
    nb->predictProb(X, R2, P2);
    std::cout << R2 << P2 << std::endl;*/
}