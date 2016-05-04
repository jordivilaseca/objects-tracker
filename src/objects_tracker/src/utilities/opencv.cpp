#include <objects_tracker/utilities/opencv.hpp>

void calcRgbHistSep(const cv::Mat &image, const cv::Mat &mask, int histSize, cv::Mat &hist) {
	float range[] = {0, 256};
	const float* histRange[] = {range};

	// Split Mat into three Mats (one per color)
	std::vector<cv::Mat> bgr_planes;
  	cv::split(image, bgr_planes);

  	cv::Mat b_hist, g_hist, r_hist;

	// Compute the histograms:
	cv::calcHist( &bgr_planes[0], 1, 0, mask, b_hist, 1, &histSize, histRange, true, false);
	cv::calcHist( &bgr_planes[1], 1, 0, mask, g_hist, 1, &histSize, histRange, true, false);
	cv::calcHist( &bgr_planes[2], 1, 0, mask, r_hist, 1, &histSize, histRange, true, false);
	
	// From column matrix to row matrix.
	cv::transpose(b_hist, b_hist);
	cv::transpose(g_hist, g_hist);
	cv::transpose(r_hist, r_hist);

	// Join all the histograms.
	hist = b_hist;
	cv::hconcat(hist, g_hist, hist);
	cv::hconcat(hist, r_hist, hist);
}

void calcRgbHist(const cv::Mat &image, const cv::Mat &mask, int bins, cv::Mat &hist) {
	int histSize[] = {bins, bins, bins};
	float range[] = {0, 256};
	int channels[] = {0,1,2};
	const float* histRange[] = {range, range, range};

	// Split Mat into three Mats (one per color)
	cv::Mat auxHist;

	// Compute the histograms:
	cv::calcHist( &image, 1, channels, mask, auxHist, 3, histSize, histRange, true, false);

	// Fill histogram.
	hist = cv::Mat(1,bins*bins*bins,CV_32FC1,0.0);
	int nelem = 0;
	for(int i = 0; i < bins; i++) {
		for(int j = 0; j < bins; j++) {
			for(int k = 0; k < bins; k++) {
				float f = auxHist.at<float>(i,j,k);
				hist.at<float>(nelem++) = f;
			}
		}
	}
	
	/*// From column matrix to row matrix.
	cv::transpose(b_hist, b_hist);
	cv::transpose(g_hist, g_hist);
	cv::transpose(r_hist, r_hist);

	// Join all the histograms.
	hist = b_hist;
	cv::hconcat(hist, g_hist, hist);
	cv::hconcat(hist, r_hist, hist);*/
}