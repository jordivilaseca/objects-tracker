#include <opencv2/core.hpp>
#include <cv.h>

void calcRgbHistSep(const cv::Mat &image, const cv::Mat &mask, int histSize, cv::Mat &hist);
void calcRgbHist(const cv::Mat &image, const cv::Mat &mask, int bins, cv::Mat &hist);

void calcHsvHist(const cv::Mat &image, const cv::Mat &mask, int h_bins, int s_bins, cv::Mat &hist);