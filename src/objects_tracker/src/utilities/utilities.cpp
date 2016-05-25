#include <objects_tracker/utilities/utilities.hpp>

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

void writeMetrics(const std::vector<std::vector<int>> &confMat, float accur,const std::vector<float> &precision, const std::vector<float> &recall, const std::vector<float> &fmeasure, const std::vector<std::string> &header,const std::string &path) {
	std::ofstream fs;
	fs.open(path);

	// Print confusion matrix
	for(int i = 0; i < confMat.size(); i++) {
		fs << ", " << header[i];
	}
	fs << ",,Average\n";
	for(int i = 0; i < confMat.size(); i++) {
		fs << header[i];
		for(int j = 0; j < confMat.size(); j++) {
		  fs << ", " << confMat[i][j];
		}
		fs << "\n";
	}

	fs << "\n";

	// Print accuracy.
	fs << "Accuracy,";
	for(int i = 0; i < confMat.size(); i++) fs << ",";
	fs << "," << accur << "\n";

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