#ifndef OBJECTS_UTILITIES_H
#define OBJECTS_UTILITIES_H

#include <vector>
#include <cmath>

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

#endif