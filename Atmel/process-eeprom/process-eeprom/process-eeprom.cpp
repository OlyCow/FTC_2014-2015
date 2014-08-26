// process-eeprom.cpp : Does stuff.
#include "stdafx.h"
#include <iostream>
#include <string>
#include <fstream>
#include <cinttypes>

int convert_complement(uint16_t input);

int main()
{
	using namespace std;
	
	cout << "This program processes raw EEPROM data gathered from\n";
	cout << "an ATmega328(P) (in the appropriate format).\n";
	cout << "\nPlease enter the name of the input file:  --";
	string filename_input = "input.txt";
	cin >> filename_input;
	cout << "\nPlease enter the name of the output file: --";
	string filename_output = "output";
	cin >> filename_output;
	filename_output.append(".csv");

	cout << "\n\n\tProcessing...";
	ifstream input_file(filename_input, ifstream::binary);
	ofstream output_file(filename_output);

	output_file << "Value\n";

	while (input_file.good())
	{
		uint8_t buffer_low = 0;
		uint8_t buffer_high = 0;
		uint16_t buffer_total = 0;

		buffer_low = input_file.get();
		buffer_high = input_file.get();

		buffer_total = (buffer_high << 8) + buffer_low;
		int value = convert_complement(buffer_total);

		output_file << value << endl;
	}

	input_file.close();
	output_file.close();

	return 0;
}

int convert_complement(uint16_t input)
{
	int output = 0; // TODO: Is this a safe initialization value?

	// If MSB is 1:
	if (input>(0x8000 - 1)) {
		uint16_t buffer = ~input;
		++buffer;
		output = 0 - buffer;
	}
	else {
		output = input;
	}

	return output;
}
