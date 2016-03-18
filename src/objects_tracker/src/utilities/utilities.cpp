#include <objects_tracker/utilities/utilities.hpp>

void computeColors(int n, std::vector< std::vector<int> > &colours) {
	colours = std::vector< std::vector<int> >(n, std::vector<int>(3));

	int step = (256*4)/n;
	int r, g, b;
	int i = 0;

	// Red and blue are fixed, incrementing green from 0 to 255.
	r = 255;
	g = 0;
	b = 0;
	for(; g <= 255 and i < n; g += step) {
		colours[i][0] = r; colours[i][1] = g; colours[i][2] = b;
		i++;
	}

	// Green and blue are fixed, decrementing red from 255 to 0.
	r = 255 - (g - 256);
	g = 255;
	for(; r >= 0 and i < n; r -= step) {
		colours[i][0] = r; colours[i][1] = g; colours[i][2] = b;
		i++;
	}

	// Red and green fixed, incrementing blue from 0 to 255.
	b = -r;
	r = 0;
	for(; b <= 255 and i < n; b += step) {
		colours[i][0] = r; colours[i][1] = g; colours[i][2] = b;
		i++;
	}

	// Red and blue fixed, decrementing green from 255 to 0.
	g = 255 - (b - 256);
	b = 255;
	for(; g >= 0 and i < n; g -= step) {
		colours[i][0] = r; colours[i][1] = g; colours[i][2] = b;
		i++;
	}
}