#ifndef OBJECTS_UTILITIES_H
#define OBJECTS_UTILITIES_H

#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>

void computeColor(int i, int n, std::vector<int> &color);
void computeColor(int i, int n, std::vector<double> &color);

template <typename T>
void computeColor(int i, int n, T max, std::vector<int> &color) {
	double param, fractpart, intpart;
	param = ((float) i)/((float) n)*4.0;
	fractpart = std::modf(param , &intpart);
	switch ((int) intpart) {
		case 0:
			color = {max, max*fractpart, 0};
			break;
		case 1:
			color = {max - max*fractpart, max, 0};
			break;
		case 2:
			color = {0, max, max*fractpart};
			break;
		case 3:
			color = {0, max - max*fractpart, max};
			break;
		default:
			color = {max, max, max};
			break;
	}
}

template <typename T>
void computeColors(int n, std::vector< std::vector<T> > &colours) {
	static_assert(std::is_same<T, int>::value or std::is_same<T, double>::value, "Error computeColors. It only accepts floats or ints");
	T max;
	if (std::is_same<T, int>::value) {
		max = 255;
	} else {
		max = 1.0;
	}
	colours = std::vector< std::vector<T> >(n, std::vector<T>(3));

	for(int i = 0; i < n; i++) {
		computeColor(i, n, colours[i]);
	}
}

template <typename T>
T sum(const std::vector<T> &list) {
	T total = {};

	for(const T &elem : list) total += elem;

	return total;
}

template <typename T>
void quaternion2euler(const T q, T &e) {
	assert(e.size() == 3);
	e[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
	e[1] = asin(2*(q[0]*q[2] + q[3]*q[1]));
	e[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
}

template <typename T>
void writeList(const std::vector<T> &list, std::string path) {
	std::ofstream fs;
    fs.open (path);
    for (size_t i = 0; i < list.size (); ++i)
      fs << list[i] << "\n";
    fs.close();
}

template <typename T>
bool readList(std::vector<T> &list, std::string path) {
	std::ifstream fs;
	fs.open(path);
	if (!fs.is_open() || fs.fail()) return false;

	T value;
	while (fs >> value)
	{
		list.push_back(value);
	}
	fs.close();
	return true;
}

template <typename T>
void writeMatrix(const std::vector<std::vector<T>> &matrix, const std::string &path) {
	std::ofstream fs;
    fs.open (path);
    for (size_t i = 0; i < matrix.size(); ++i) {
    	for(size_t j = 0; j < matrix[i].size(); ++j) {
    		fs << matrix[i][j] << " ";
    	}
    	fs << "\n";
    }
    fs.close();
}

template <typename T>
bool readMatrix(std::vector<std::vector<T>> &matrix, const std::string &path) {

	std::ifstream fs;
	fs.open(path);
	if (!fs.is_open() || fs.fail()) return false;

	std::string line;
	while(std::getline(fs, line)) {

		std::vector<T> newTline;
		T elem;
		std::istringstream iss(line);

		while ( iss >> elem) {    
			newTline.push_back(elem);
		}

		matrix.push_back(newTline);
	}
	return true;
}

template <typename T>
void printList(const std::vector<T> &list) {
	for(T elem : list) std::cout << elem << " ";
	std::cout << std::endl;
}

void writeMetrics(const std::vector<std::vector<int>> &confMat, const std::vector<float> &accur,const std::vector<float> &precision, const std::vector<float> &recall, const std::vector<float> &fmeasure, const std::vector<std::string> &trainingHeader, const std::vector<std::string> &testingHeader,const std::string &path);

void quaternion2euler(const std::vector<float> q, std::vector<float> &e);

#endif