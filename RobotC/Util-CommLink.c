#pragma config(Sensor, S4,     sensor_protoboard, sensorI2CCustom9V)

#include "includes.h"

task CommLink(); // R/W to the prototype board as tightly as possible. TODO: Separate into its own library.
task Display(); // Updates the NXT's LCD display with useful info.

// For comm link:
// TODO: Make more efficient by putting vars completely inside bytes, etc.
// Possibly implement something like "std::vector<bool>".
const int NXT_LINE_NUM = 6;
typedef enum CardinalDirection {
	CARDINAL_DIR_N	= 0,
	CARDINAL_DIR_E	= 1,
	CARDINAL_DIR_S	= 2,
	CARDINAL_DIR_W	= 3,
	CARDINAL_DIR_NUM
} CardinalDirection;
typedef enum LineMode {
	LINE_MODE_UNUSED	= -1,
	LINE_MODE_ROT_XY	= 0,
	LINE_MODE_ROT_XYZ,
	LINE_MODE_ROT_Z,
	LINE_MODE_FLAGS,
	LINE_MODE_NUM
} LineMode;
LineMode c_lineMode[6] = {	LINE_MODE_ROT_XY,
							LINE_MODE_ROT_XYZ,
							LINE_MODE_ROT_Z,
							LINE_MODE_FLAGS,
							LINE_MODE_UNUSED,
							LINE_MODE_UNUSED	};

// Sensor info:
int c_angle_x = 0; // RobotC doesn't support unsigned types (other than ubyte).
int c_angle_y = 0; // This is equivalent to "0".
int c_angle_z = 0;
int c_pos_x = 0;
int c_pos_y = 0;
int c_pos_z = 0;

// Alert flags: (not redundant! hehe)
bool c_isOverheating = false;
bool c_isConfigReady = false;

// Config info:
// Changing this var is helpful for testing the MCU connection.
bool c_isRamp = true; // Whether or not to start on the ramp.
int c_autonDelay = 0; // How many seconds to delay the start of the autonomous program.

// Comm link debugging vars:
const ubyte mask_read = 0b00111111; // We read from the last 6 bits.
const ubyte mask_write = 0b11000000; // We write to the first 2 bits. TODO: Not actually needed to write?
ubyte f_byte_write = 0;
ubyte f_byte_read = 0;
bool isClockHigh = true;
bool isResync = true; // We start off with a resync.
int error_num = 0; // Incremented every time there's a consecutive error we can't correct.
bool header_write = false;
bool header_read[6] = {false, false, false, false, false, false};
ubyte frame_write[4] = {0x55,0x6F,0xE5,0x7A};
ubyte frame_read[6][4] = {{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0}};



task main()
{
	initializeGlobalVariables(); // Defined in "initialize.h", this intializes all struct members.
	Task_Kill(displayDiagnostics); // This is set separately in the "Display" task.
	Task_Spawn(CommLink);
	Task_Spawn(Display);

	while (true) {
		Time_Wait(100);
	}
}



void processCommTick()
{
	f_byte_write &= ~(1<<7); // Clear the clock bit.
	f_byte_write |= (isClockHigh<<7); // Set the clock bit to appropriate clock value.
	HTSPBwriteIO(sensor_protoboard, f_byte_write);
	f_byte_read = HTSPBreadIO(sensor_protoboard, mask_read);
	isClockHigh = !isClockHigh; // Cannot use XOR (bools are weird).
}
task CommLink()
{
	ubyte current_index_mask = 0; // Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
	ubyte byte_temp = 0;// Convenience variable. See specific uses. (DARK MAGIC; MIGHT NOT WORK)
	const int max_error_num = 5; // If we get more corrupted packets, we should restart transmission.
	bool wasCorrupted = false;
	// Check bits DO NOT include header bits!
	bool check_write = 0; // TODO: Switch to Hamming codes! (Mebbe?) :D
	bool check_read[6] = {0,0,0,0,0,0}; // Value read.
	bool check_read_ack[6] = {0,0,0,0,0,0}; // Value computed.
	bool isBadData[6] = {false, false, false, false, false, false};

	HTSPBsetupIO(sensor_protoboard, mask_write); // `mask_write` happens to conform to the expected format.

	// We don't want to wait for start here (we need to establish
	// a communication link as soon as possible).

	while (true) {

		// Restart the communication link.
		while (isResync==true) {
			// First make sure we're in sync.
			short sync_count = 0;
			int fail_count = 0; // TODO: If this gets too high, alert the drivers.
			while (sync_count<6) { // 3 high and 3 low. Has to be more than a frame? TODO
				f_byte_write |= (1<<6); // Set the data bit high.
				processCommTick();
				f_byte_read |= 0b11000000; // Make sure the "write" bits aren't random.
				switch (isClockHigh) { // We want all the bits to be high (0b11111111). The MAGIC_NUM depends on the clock.
					case false : // These may seem flipped, but that's because the clock is ready for the next tick.
						f_byte_read = f_byte_read^0b00000000; // MAGIC_NUM, kinda
						break;
					case true : // These may seem flipped, but that's because the clock is ready for the next tick.
						f_byte_read = f_byte_read^0b00111111; // MAGIC_NUM, kinda
						break;
				}
				if (f_byte_read==0b11111111) {
					sync_count++;
				} else {
					sync_count = 0;
					fail_count++;
				}
			}
			if (isClockHigh==true) {
				// If so, let it go for another tick.
				processCommTick();
			}

			// Now bring the data line low for 2 clock ticks.
			f_byte_write &= ~(1<<6); // Clear the data bit low.
			processCommTick();
			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
			if (f_byte_read!=0b00000000) {
				isResync = true;
				continue;
			} else {
				isResync = false;
			}
			processCommTick(); // Wait another tick...
			f_byte_read &= 0b00111111; // Make sure the "write" bits aren't random.
			if (f_byte_read!=0b00000000) { // 0, DUH...
				isResync = true;
				continue;
			} else {
				isResync = false;
			}
			// If everything is still good at this point, go on.
		}

		// Write header.
		f_byte_write &= ~(1<<6); // Clear the data bit.

		// TODO: use ubyte instead of bool and just use last bit?
		// A bool can be true but not have the last bit be on.
		if (header_write==true) {
			f_byte_write |= (1<<6);
		} // No "else" needed (already set to 0).
		processCommTick();

		// Read in all 6 data lines.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			current_index_mask = 1<<line;
			// No fancy shifting needed here (header_read is a bool):
			header_read[line] = Bit_FixBool((bool)(f_byte_read&current_index_mask)); // Theoretically, if >0 then true.
		}

		// Data:
		// Clear parity bits.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			check_read_ack[line] = false;
			for (int i=0; i<4; i++) {
				frame_read[line][i] = 0; // TODO: MAKE ATOMIC! IMPORTANT! <<====
			}
		}
		for (int bit=0; bit<32; bit++) {
			int frame = bit/8; // Intentional int division.
			int sub_bit = bit%8;

			// Set MOSI.
			f_byte_write &= ~(1<<6); // Clear the data bit.
			current_index_mask = 1<<(sub_bit); // Set the data bit; `i%8` because data is in bytes.

			// Intentional int division (returns intended byte) (see next statement).
			// Using a temp var because `true!=1` (can be any positive int); statement
			// also clears byte_temp because the mask was cleared (and now AND'd).
			byte_temp = (frame_write[frame])&current_index_mask; // TODO: Use current_index_mask instead of temp var?

			// TODO: combine the two shifts below into one shift.
			byte_temp = byte_temp>>(sub_bit); // Shift data bit over to bit 0.
			f_byte_write |= (byte_temp<<6); // Set the data bit.

			check_write = (Bit_FixBool((bool)byte_temp) != check_write);
			// TODO: For optimization, delete the following line. Right now it's to be safe.
			check_write = Bit_FixBool(check_write);
			processCommTick();

			// Read in all 6 data lines (MISO).
			for (int line=0; line<NXT_LINE_NUM; line++) {
				current_index_mask = 1<<line;
				byte_temp = f_byte_read&current_index_mask; // Isolating the bit we want.
				// TODO: combine the two shifts below into one shift. Actually, we might not even need byte_temp here.
				byte_temp = byte_temp>>line; // Shift the bit into bit 0.
				frame_read[line][frame] |= (byte_temp<<sub_bit); // Shift bit into appropriate place in frame.

				// Because `byte_temp` only has one bit now.
				check_read_ack[line] = (Bit_FixBool((bool)byte_temp) != check_read_ack[line]);
				// TODO: For optimization, delete the following line. Right now it's to be safe.
				check_read_ack[line] = Bit_FixBool(check_read_ack[line]);
			}
		}

		// Check bits.
		f_byte_write &= ~(1<<6); // Clear the data bit.
		if (check_write==true) {
			f_byte_write |= 0b01000000;
		} // No need for "else" condition (byte already cleared).
		processCommTick();
		check_write = false; // Clear this now that we've sent it already.

		// Read check bits.
		for (int line=0; line<NXT_LINE_NUM; line++) {
			current_index_mask = 1<<line; // Select the bit we want to find.
			check_read[line] = Bit_FixBool((bool)(f_byte_read&current_index_mask));
			if (check_read[line] == check_read_ack[line]) {
				isBadData[line] = false;
			} else {
				isBadData[line] = true;
				error_num++;
				wasCorrupted = true;
			}
		}
		// `check_read_ack[]` is reset before use each loop.

		if (error_num>max_error_num) {
			isResync = true; // This happens at the beginning of the next iteration.
			error_num = 0;
			wasCorrupted = false;
		} else if ((error_num!=0)&&(wasCorrupted==false)) {
			error_num = 0; // Not a consecutive error.
		} else if (error_num==0) {
			wasCorrupted = false; // This executes even if first condition was true (it checks again?)
		}

		for (int line=0; line<NXT_LINE_NUM; line++) {
			if (isBadData[line]==true) {
				continue;
			}
			if (header_read[line]==false) {
				switch (f_commLinkMode[line]) {
					case COMM_LINK_POS_XY :
						// TODO: Figure out CPU hogging, you pig.
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_pos_x = frame_read[line][3];
						f_pos_x = f_pos_x<<1; // There's one more bit of data we need to access.
						f_pos_x += ((frame_read[line][2]&0b10000000)>>7); // TODO: Optimize: masking unnecessary?
						f_angle_x = frame_read[line][2];
						f_angle_x &= 0b01111111;
						f_pos_y = frame_read[line][1];
						f_pos_y = f_pos_y<<1; // There's one more bit of data we need to access.
						f_pos_y += ((frame_read[line][0]&0b10000000)>>7); // TODO: Optimize: masking unnecessary?
						f_angle_y = frame_read[line][0];
						f_angle_y &= 0b01111111;
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_ROT_LIGHT :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_angle_z = frame_read[line][2];
						f_angle_z += ((frame_read[line][3]&0b00000001)<<8);
						f_pos_z = ((frame_read[line][3]&0b01111110)>>1);
						f_isRedAlliance = Bit_FixBool((bool)(frame_read[line][3]&0b10000000)); // TODO: only assign this at the beginning of the match.
						for (int i=0; i<4; i++) {
							f_lineSensor[0][i] = Bit_FixBool((bool)(frame_read[line][1]&(1<<i)));
							f_lineSensor[1][i] = Bit_FixBool((bool)(frame_read[line][1]&(1<<(i+4))));
						}
						for (int i=0; i<8; i++) {
							f_cubeDetected[i] = Bit_FixBool((bool)(frame_read[line][0]&(1<<i)));
						}
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_RANGE_AB :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_closeRange[CARDINAL_DIR_N] = (frame_read[line][3]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_N] = frame_read[line][2];
						f_longRange[CARDINAL_DIR_N] += ((frame_read[line][3]<<8)&0b00000001);
						f_closeRange[CARDINAL_DIR_E] = (frame_read[line][1]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_E] = frame_read[line][0];
						f_longRange[CARDINAL_DIR_E] += ((frame_read[line][1]<<8)&0b00000001);
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_RANGE_CD :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_closeRange[CARDINAL_DIR_S] = (frame_read[line][3]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_S] = frame_read[line][2];
						f_longRange[CARDINAL_DIR_S] += ((frame_read[line][3]<<8)&0b00000001);
						f_closeRange[CARDINAL_DIR_W] = (frame_read[line][1]>>1)&0b01111111;
						f_longRange[CARDINAL_DIR_W] = frame_read[line][0];
						f_longRange[CARDINAL_DIR_W] += ((frame_read[line][1]<<8)&0b00000001);
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_TELEOP :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						f_cubeNum = (frame_read[line][3]>>4)&0b00000111;
						//Task_ReleaseCPU();
						break;
					case COMM_LINK_BUMPERS :
						//Task_HogCPU(); // So that the main program doesn't try to access these vars.
						// TODO: Make these terrible masks better or something. IT HURTS
						f_isFlagBumped = Bit_FixBool((bool)((frame_read[line][3]>>7)&0b00000001));
						f_isHangBumped = Bit_FixBool((bool)((frame_read[line][3]>>6)&0b00000001));
						for (int i=CARDINAL_DIR_N; i<(int)CARDINAL_DIR_NUM; i++) {
							f_isBumped[i] = Bit_FixBool((bool)(frame_read[line][3]&(1<<i)));
						}
						//Task_ReleaseCPU();
						break;
					default :
						// Much to do about nothing. ;)
						break;
				}
			} else {
				// Handle special codes here.
			}
		}
	}
}



// Task for displaying data on the NXT's LCD screen.
// TODO: Put a lot of the display stuff into loops. Do we want to?
task Display()
{
	typedef enum DisplayMode {
		DISP_FCS,				// Default FCS screen.
		DISP_COMM_STATUS,		// Each line of each frame.
		DISP_COMM_DEBUG,		// The literal frames received.
		DISP_SENSORS,			// TODO: Might need to split this into two screens.
		DISP_NUM
	};

	Task_Spawn(displayDiagnostics); // Explicit here: this is only spawned when buttons are pressed.
	DisplayMode isMode = DISP_FCS;

	// We don't need to wait for start. ;)

	while (true) {
		Buttons_UpdateData();

		switch (isMode) {
			case DISP_FCS :
				break;
			case DISP_COMM_STATUS :
				switch (f_isRedAlliance) {
					case true :
						nxtDisplayCenteredTextLine(0, "RED ALLIANCE");
						break;
					case false :
						nxtDisplayCenteredTextLine(0, "BLUE ALLIANCE");
						break;
					default :
						nxtDisplayCenteredTextLine(0, "BOOL ERROR! DEBUG!");
						break;
				}
				switch (isResync) {
					case true :
						nxtDisplayTextLine(1, "Resyncing...");
						break;
					case false :
						nxtDisplayTextLine(1, "Transmitting...");
						break;
				}
				nxtDisplayTextLine(2, "lost pkts: %d", error_num);
				break;
			case DISP_COMM_DEBUG :
				nxtDisplayCenteredTextLine(0, "W %#4X  R %#4X", f_byte_write, f_byte_read);
				nxtDisplayTextLine(2, "F %2X-%2X-%2X-%2X", frame_read[5][3], frame_read[5][2], frame_read[5][1], frame_read[5][0]);
				nxtDisplayTextLine(3, "E %2X-%2X-%2X-%2X", frame_read[4][3], frame_read[4][2], frame_read[4][1], frame_read[4][0]);
				nxtDisplayTextLine(4, "D %2X-%2X-%2X-%2X", frame_read[3][3], frame_read[3][2], frame_read[3][1], frame_read[3][0]);
				nxtDisplayTextLine(5, "C %2X-%2X-%2X-%2X", frame_read[2][3], frame_read[2][2], frame_read[2][1], frame_read[2][0]);
				nxtDisplayTextLine(6, "B %2X-%2X-%2X-%2X", frame_read[1][3], frame_read[1][2], frame_read[1][1], frame_read[1][0]);
				nxtDisplayTextLine(7, "A %2X-%2X-%2X-%2X", frame_read[0][3], frame_read[0][2], frame_read[0][1], frame_read[0][0]);
				break;
			case DISP_SENSORS :
				nxtDisplayTextLine(0, "%1d cubes", f_cubeNum);
				nxtDisplayTextLine(2, "Position:");
				nxtDisplayTextLine(3, "(%+5d,%+5d,%+3d)", f_pos_x, f_pos_y, f_pos_z);
				nxtDisplayTextLine(5, "Rotation:");
				int temp_x = f_angle_x - 30;	// TODO: Make this not ugly.
				int temp_y = f_angle_y - 30;
				nxtDisplayTextLine(6, "(%3d,%3d,%3d)", temp_x, temp_y, f_angle_z);
				break;
			default :
				nxtDisplayCenteredTextLine(3, "Doesn't work...");
				nxtDisplayCenteredTextLine(4, "Yet. >:(");
				break;
		}

		if (Buttons_Released(NXT_BUTTON_L)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM-1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		if (Buttons_Released(NXT_BUTTON_R)==true) {
			Display_Clear();
			isMode = (DisplayMode)((isMode+DISP_NUM+1)%DISP_NUM);
			if (isMode==DISP_FCS) {
				Task_Spawn(displayDiagnostics);
			} else {
				Task_Kill(displayDiagnostics);
			}
		}
		Time_Wait(50); // MAGIC_NUM: Prevents the LCD from updating itself to death.
	}
}
