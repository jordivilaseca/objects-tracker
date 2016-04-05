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
		default:
			color = {255, 255, 255};
	}
}

void computeColors(int n, std::vector< std::vector<int> > &colours) {
	colours = std::vector< std::vector<int> >(n, std::vector<int>(3));

	for(int i = 0; i < n; i++) {
		computeColor(i, n, colours[i]);
	}
}