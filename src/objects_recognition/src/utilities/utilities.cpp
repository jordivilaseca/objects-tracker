#include <objects_tracker/utilities/utilities.hpp>

/**
 * @brief Get current milliseconds.
 * @return Milliseconds.
 */
long long getTime() {
    struct timeval tp;
    gettimeofday(&tp, NULL);
    long long mslong = (long long) tp.tv_sec * 1000L + tp.tv_usec / 1000;
    return mslong;
}

/**
 * @brief Compute the i-th colour of a total number of n. Colour interval of [0,255]
 * 
 * @param i Colour to be computed.
 * @param n Total number of colours to compute.
 * @param [out] color Obtained colour.
 */
void computeColor(int i, int n, std::vector<int> &color) {
	double param, fractpart, intpart;
	param = ((float) i)/((float) n)*4.0;
	fractpart = std::modf(param , &intpart);
	switch ((int) intpart) {
		case 0:
			color = {255, (int) (255*fractpart), 0};
			break;
		case 1:
			color = {255 - ((int) (255*fractpart)), 255, 0};
			break;
		case 2:
			color = {0, 255, (int) (255*fractpart)};
			break;
		case 3:
			color = {0, 255 - ((int) (255*fractpart)), 255};
			break;
		default:
			color = {255, 255, 255};
			break;
	}
}

/**
 * @brief Compute the i-th colour of a total number of n. Colour interval of [0,1]
 * 
 * @param i Colour to be computed.
 * @param n Total number of colours to compute.
 * @param [out] color Obtained colour.
 */
void computeColor(int i, int n, std::vector<double> &color) {
	double param, fractpart, intpart;
	param = ((float) i)/((float) n)*4.0;
	fractpart = std::modf(param , &intpart);
	switch ((int) intpart) {
		case 0:
			color = {1.0, 1.0*fractpart, 0};
			break;
		case 1:
			color = {1.0 - 1.0*fractpart, 1.0, 0};
			break;
		case 2:
			color = {0, 1.0, 1.0*fractpart};
			break;
		case 3:
			color = {0, 1.0 - 1.0*fractpart, 1.0};
			break;
		default:
			color = {1.0, 1.0, 1.0};
			break;
	}
}

/**
 * @brief Write different metrics to a file.
 * 
 * @param confMat Confusion matrix.
 * @param accur Accuracy.
 * @param precision Precision.
 * @param recall Recall.
 * @param fmeasure F-Measure.
 * @param trainingHeader Names of the elements of the training set.
 * @param testingHeader Names of the elememnts of the testing set.
 * @param path Path to place the file, with filename.
 */
void writeMetrics(const std::vector<std::vector<int>> &confMat, const std::vector<float> &accur,const std::vector<float> &precision, const std::vector<float> &recall, const std::vector<float> &fmeasure, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &testingHeader,const std::string &path) {
	std::ofstream fs;
	fs.open(path);

	// Print confusion matrix
	for(int i = 0; i < trainingHeader.size(); i++) {
		fs << ", " << trainingHeader[i];
	}
	fs << ",,Average\n";
	for(int i = 0; i < testingHeader.size(); i++) {
		fs << testingHeader[i];
		for(int j = 0; j < trainingHeader.size(); j++) {
		  fs << ", " << confMat[i][j];
		}
		fs << "\n";
	}

	fs << "\n";

	// Print accuracy.
	fs << "Accuracy,";
	for(int i = 0; i < accur.size(); i++) fs << accur[i] << ",";
	fs << "," << sum(accur) / (float) accur.size() << "\n";

	// Print precision.
	fs << "Precision,";
	for(int i = 0; i < precision.size(); i++) fs << precision[i] << ",";
	fs << "," << sum(precision) / (float) precision.size() << "\n";

	// Print recall.
	fs << "Recall,";
	for(int i = 0; i < recall.size(); i++) fs << recall[i] << ",";
	fs << "," << sum(recall) / (float) recall.size() << "\n";

	// Print fmeasure.
	fs << "F-measure,";
	for(int i = 0; i < fmeasure.size(); i++) fs << fmeasure[i] << ",";
	fs << "," << sum(fmeasure) / (float) fmeasure.size() << "\n";
}