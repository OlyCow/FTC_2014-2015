#include "stdafx.h"
#include <vector>
#include <cmath>
#include <string>
#include <iostream>
#include <fstream>

#include "lodepng.h"

void RGB_to_HSV(int R, int G, int B, float& H, float& S, float& V);

int main()
{
	using std::vector;
	using std::string;
	using std::cin;
	using std::cout;
	using std::endl;
	
	vector<unsigned char> image_input;
	vector<unsigned char> image_output;
	vector<unsigned char> image_out_H;
	vector<unsigned char> image_out_S;
	vector<unsigned char> image_out_V;
	unsigned int width, height;
	string filename_input = "";
	string filename_buffer = "";
	float H_target, H_tolerance, S_threshold;

	cout << "This program separates HSV components of a PNG image\n";
	cout << "and performs a simple blob detection on the output.\n";
	cout << "NOTE: Files of the same name are overwritten w\\out\n";
	cout << "warning; to keep the output files move or rename them.\n";

	cout << endl << "Enter input filename (without extension): ";
	getline(cin, filename_input);
	filename_buffer = filename_input + ".png";

	cout << "Enter target hue:\t";
	cin >> H_target;
	cout << "Enter hue tolerance:\t";
	cin >> H_tolerance;
	cout << "Enter saturation threshold: ";
	cin >> S_threshold;

	cout << endl << "decoding..." << endl;
	lodepng::decode(image_input, width, height, filename_buffer.c_str());

	cout << endl << "Input image size:" << endl;
	cout << "width:  " << width << endl;
	cout << "height: " << height << endl;

	cout << endl << "processing..." << endl;
	int chunk_size = image_input.size() / width / height;
	for (unsigned int i = 0; i < image_input.size(); i += chunk_size) {
		unsigned int i_R = i;
		unsigned int i_G = i + 1;
		unsigned int i_B = i + 2;
		unsigned int R = image_input[i_R];
		unsigned int G = image_input[i_G];
		unsigned int B = image_input[i_B];
		float H_, S_, V_;
		RGB_to_HSV(R, G, B, H_, S_, V_);
		int H = static_cast<int>(roundf(H_ / 360.0f * 255));
		int S = static_cast<int>(roundf(S_ * 255));
		int V = static_cast<int>(roundf(V_ * 255));
		image_out_H.push_back(H);
		image_out_H.push_back(H);
		image_out_H.push_back(H);
		image_out_H.push_back(255);
		image_out_S.push_back(S);
		image_out_S.push_back(S);
		image_out_S.push_back(S);
		image_out_S.push_back(255);
		image_out_V.push_back(V);
		image_out_V.push_back(V);
		image_out_V.push_back(V);
		image_out_V.push_back(255);

		int intensity = 0;
		float diff = H_ - H_target;
		if (fabs(diff) > 180) {
			H_ -= 360 * copysignf(1.0f, diff); // diff can't be 0!
		}
		if (fabs(H_ - H_target) < H_tolerance) {
			if (V_ > 0.2) {
				if (S_ > S_threshold) {
					intensity = 255;
				}
			}
		}

		image_output.push_back(intensity);
		image_output.push_back(intensity);
		image_output.push_back(intensity);
		image_output.push_back(255);
	}

	cout << "writing..." << endl;
	filename_buffer = filename_input + "_H.png";
	lodepng::encode(filename_buffer, image_out_H, width, height);
	filename_buffer = filename_input + "_S.png";
	lodepng::encode(filename_buffer, image_out_S, width, height);
	filename_buffer = filename_input + "_V.png";
	lodepng::encode(filename_buffer, image_out_V, width, height);
	filename_buffer = filename_input + "_out.png";
	lodepng::encode(filename_buffer, image_output, width, height);

	cout << endl <<  "Done!" << endl;
	//cout << "Enter anything to exit. ";
	//int placeholder;
	//cin >> placeholder;
	return 0;
}


void RGB_to_HSV(int R, int G, int B, float& H, float& S, float& V)
{
	float R_ = static_cast<float>(R) / 255.0f;
	float G_ = static_cast<float>(G) / 255.0f;
	float B_ = static_cast<float>(B) / 255.0f;
	float Cmax = fmax(fmax(R_, G_), B_);
	float Cmin = fmin(fmin(R_, G_), B_);
	float delta = Cmax - Cmin;

	V = Cmax;

	if (Cmax == 0) {
		S = 0;
		H = -1;
		return;
	}
	else {
		S = delta / Cmax;
	}

	if (Cmax == R_) {
		H = (G_ - B_) / delta;
		H = fmod(H, 6.0f);
	}
	else if (Cmax == G_) {
		H = 2 + (B_ - R_) / delta;
	}
	else if (Cmax == B_) {
		H = 4 + (R_ - G_) / delta;
	}
	H *= 60;
	H = fmod(H, 360.0f);
	if (H < 0) {
		H += 360;
	}
}
