#include <opencv2/core/core.hpp>
#include <cv.h>

void calcRgbHistSep(const cv::Mat &image, const cv::Mat &mask, int histSize, cv::Mat &hist);
void calcRgbHist(const cv::Mat &image, const cv::Mat &mask, int bins, cv::Mat &hist);