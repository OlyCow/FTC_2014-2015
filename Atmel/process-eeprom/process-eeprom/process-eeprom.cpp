// process-eeprom.cpp : Does stuff.
#include "stdafx.h"
#include <iostream>
#include <string>
#include <fstream>
#include <cinttypes>

int convert_complement(uint16_t input);

const int MAX_GARBAGE_COUNT = 10;

int main()
{
	using namespace std;

	bool continue_running = false; // Make this work when I feel like it :P
	
	cout << "This program processes raw EEPROM data gathered from\n";
	cout << "an ATmega328(P) (in the appropriate format).\n";
	cout << "\nPlease enter the name of the input file:\n\t--";
	string filename_input = "input";
	cin >> filename_input;
	filename_input.append(".dat");
	cout << "\nPlease enter the name of the output file:\n\t--";
	string filename_output = "output";
	cin >> filename_output;
	filename_output.append(".csv");

	bool DOUBLE_BYTE = false;
	char confirm = 'N';
	cout << "\nIs the data in units of two (2) bytes each? [Y/N] ";
	cin >> confirm;
	if (confirm == 'Y' || confirm == 'y') {
		DOUBLE_BYTE = true;
	}
	bool CONVERT_COMPLEMENT = false;
	if (DOUBLE_BYTE) {
		cout << "\nIs the data in 2's complement format? [Y/N] ";
		cin >> confirm;
		if (confirm == 'Y' || confirm == 'y') {
			CONVERT_COMPLEMENT = true;
		}
	}

	cout << "\n\nProcessing...";
	ifstream input_file(filename_input, ifstream::binary);
	ofstream output_file(filename_output);

	output_file << "Value\n";

	int garbage_counter = 0;
	while (input_file.good() && garbage_counter<MAX_GARBAGE_COUNT)
	{
		uint8_t buffer_low = 0;
		uint8_t buffer_high = 0;
		uint16_t buffer_total = 0;

		buffer_low = input_file.get();
		if (DOUBLE_BYTE)
			buffer_high = input_file.get();

		if (DOUBLE_BYTE) {
			buffer_total = (buffer_high << 8) + buffer_low;
		} else {
			buffer_total = buffer_low;
		}
		

		if (CONVERT_COMPLEMENT)
			buffer_total = convert_complement(buffer_total);

		unsigned int final_value = static_cast<unsigned int>(buffer_total);
		output_file << final_value << endl;

		if (buffer_low == 0xFF) {
			++garbage_counter;
		}
		else {
			garbage_counter = 0;
		}
	}

	input_file.close();
	output_file.close();
	cout << endl << "\nFinished.\n";

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
