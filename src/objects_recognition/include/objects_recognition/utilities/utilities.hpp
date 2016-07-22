#ifndef OBJECTS_UTILITIES_H
#define OBJECTS_UTILITIES_H

#include <vector>
#include <cmath>
#include <fstream>
#include <iostream>
#include <sstream>
#include <numeric>
#include <sys/time.h>

long long getTime();

void computeColor(int i, int n, std::vector<int> &color);
void computeColor(int i, int n, std::vector<double> &color);

/**
 * @brief Compute n different colours.
 * 
 * @param n Total number of colours to compute.
 * @param colours Returned colours.
 * @tparam T Integer for [0,255] interval colours, and double for [0,1] interval colours.
 */
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

/**
 * @brief Sum the elements of a vector
 * 
 * @param list Vector of elements to add.
 * @tparam T It needs a type with the addition operator implemented.
 * @return Total value.
 */
template <typename T>
T sum(const std::vector<T> &list) {
	T total = {};

	for(const T &elem : list) total += elem;

	return total;
}

/**
 * @brief Convert quaternions to Euler angles.
 * 
 * @param q Quaternion.
 * @param [out] e Euler angles.
 * @tparam T Numerical type.
 */
template <typename T>
void quaternion2euler(const T q, T &e) {
	assert(e.size() == 3);
	e[0] = atan2(2*(q[0]*q[1] + q[2]*q[3]), 1 - 2*(q[1]*q[1] + q[2]*q[2]));
	e[1] = asin(2*(q[0]*q[2] + q[3]*q[1]));
	e[2] = atan2(2*(q[0]*q[3] + q[1]*q[2]), 1 - 2*(q[2]*q[2] + q[3]*q[3]));
}

/**
 * @brief Write list to a file
 * 
 * @param list List of elements to write
 * @param path Path where the file must be written.
 * @tparam T Type with the ofstream function implemented.
 */
template <typename T>
void writeList(const std::vector<T> &list, std::string path) {
	std::ofstream fs;
    fs.open (path);
    for (size_t i = 0; i < list.size (); ++i)
      fs << list[i] << "\n";
    fs.close();
}

/**
 * @brief Read list to a file
 * 
 * @param [out] list Output list.
 * @param path Path where the file must be read.
 * @tparam T Type with the ofstream function implemented.
 */
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

/**
 * @brief Write matrix to a file
 * 
 * @param matrix Matrix of elements to write
 * @param path Path where the file must be written.
 * @tparam T Type with the ofstream function implemented.
 */
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

/**
 * @brief Read matrix to a file
 * 
 * @param [out] matrix Output matrix.
 * @param path Path where the file must be read.
 * @tparam T Type with the ofstream function implemented.
 */
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

#endif