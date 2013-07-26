/*
 * Low level control for Microstrain 3DM-GX1
 */

#ifndef IMU_3DMGX1_CPP
#define IMU_3DMGX1_CPP

#include <termios.h> // terminal io (serial port) interface
#include <fcntl.h>   // File control definitions
#include <errno.h>   // Error number definitions
#include <assert.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <cmath>

using namespace std;
class IMU_3DMGX1
{
	const char* port;
	static const double G = 9.81; // Gravity (m/sec^2)
	short AccelGainScale;
	short GyroGainScale;

// Enumeration of possible IMU commands
	enum cmd
	{
		CMD_RAW = 0x01,
	//gyro-stabilized magnetic field and acceleration vectors, and the bias-corrected angular rate vector
		CMD_STAB_Vectors = 0x02,
		CMD_INST_Vectors = 0x03,
		CMD_INST_Quaternion = 0x04,
	//gyro-stabilized orientation quaternion
		CMD_STAB_Quaternion = 0x05,
	//capture the current gyroscope outputs and store these values as the gyro bias estimate
		CMD_CAPTURE_BIAS = 0x06,
		CMD_TEMPERATURE = 0x07,
		CMD_READ_EEPROM = 0x08,
		CMD_WRITE_EEPROM = 0x09,
		CMD_INST_Orientation = 0x0A,
	//gyro-stabilized orientation matrix
		CMD_STAB_Orientation = 0x0B,
	//gyro-stabilized orientation quaternion, the instantaneous magnetic field and acceleration vectors, and the bias corrected angular rate vector
		CMD_STAB_QuatVect = 0x0C,
		CMD_INST_Euler = 0x0D,
	//gyro-stabilized Euler Angles
		CMD_STAB_Euler = 0x0E,
	//tare its coordinate system such that the local body fixed coordinate system will be aligned with Earth-Fixed reference coordinate system (X north, Y east, Z down).
		CMD_TARE = 0x0F,
		CMD_CONTINUOUS_MODE = 0x10,
		CMD_REMOVE_TARE = 0x11,
	//gyro-stabilized orientation quaternion, Instantaneous MagField, Accel, AngRate Vectors
		CMD_STAB_Quat_INST_Vect = 0x12,
		CMD_WRITE_GAINS = 0x24,
		CMD_READ_GAINS = 0x25,
		CMD_SELF_TEST = 0x27,
		CMD_READ_EEPROM_CHECKSUM = 0x28,
		CMD_WRITE_EEPROM_CHECKSUM = 0x29,
	//gyro-stabilized Euler Angles, Instantaneous Acceleration Vector, drift compensated Angular Rate vector
		CMD_STAB_Euler_Accel_Rate = 0x31,
		CMD_INIT_HARDIRONFIELD = 0x40,
		CMD_COLLECT_HARDIRONFIELD_DATA = 0x41,
		CMD_COMPUTE_HARDIRONFIELD = 0x42,
		CMD_VERSION = 0xF0,
		CMD_SERIAL_NO = 0xF1
	}; // enum cmd

public:
	struct EulerAngles
	{
		float roll;
		float pitch;
		float yaw;
	};
	struct Quaternion
	{
		float w;
		float x;
		float y;
		float z;
	};
	struct AngularRate
	{
		float x;
		float y;
		float z;
	};
	struct Acceleration
	{
		float x;
		float y;
		float z;
	};
	typedef int ComPortHandle;
	typedef unsigned char Byte;


//=======================================================================================
// OpenComPort
//---------------------------------------------------------------------------------------
// Opens a com port with the correct settings for communicating with a MicroStrain
// 3DM-GX1 sensor
//=======================================================================================
ComPortHandle OpenComPort(const char* comPortPath)
{
	port = comPortPath;
	ComPortHandle comPort = open(comPortPath, O_RDWR | O_NOCTTY);

	if (comPort== -1) //Opening of port failed
	{
		cerr << "Unable to open com Port\n"
		        "Error:(" << errno << ") " << strerror(errno) << endl;
		return -1;
	}

	//Get the current options for the port...
	termios options;
	tcgetattr(comPort, &options);

	//set the baud rate
	int baudRate = B38400;
	cfsetospeed(&options, baudRate);
	cfsetispeed(&options, baudRate);

	//set the number of data bits.
	options.c_cflag &= ~CSIZE;  // Mask the character size bits
    options.c_cflag |= CS8;

	//set the number of stop bits to 1
	options.c_cflag &= ~CSTOPB;
	
	 //Set parity to None
	options.c_cflag &=~PARENB;
	
	//set for non-canonical (raw processing, no echo, etc.)
   	options.c_iflag = IGNPAR; // ignore parity check close_port(int
   	options.c_oflag = 0; // raw output
	options.c_lflag = 0; // raw input

	//Time-Outs -- won't work with NDELAY option in the call to open
	options.c_cc[VMIN]  = 0;   // block reading until RX x characers. If x = 0, it is non-blocking.
	options.c_cc[VTIME] = 10;   // Inter-Character Timer -- i.e. timeout= x*.1 s

	//Set local mode and enable the receiver
	options.c_cflag |= (CLOCAL | CREAD);

	//Purge serial port buffers
	tcflush(comPort,TCIOFLUSH);

	//Set the new options for the port...
	int status=tcsetattr(comPort, TCSANOW, &options);
	if (status != 0) //For error message
	{
    	cerr << "Configuring comport failed" << endl;
    	return status;
    }

	//Purge serial port buffers
	tcflush(comPort,TCIOFLUSH);

	return comPort;
}

//=======================================================================================
// CloseComPort
//---------------------------------------------------------------------------------------
// Closes a port that was previously opened with OpenComPort
//=======================================================================================
void CloseComPort(ComPortHandle comPort)
{
//restore timer tick interval to default 6.5536ms
	cout<<"Restoring IMU default time interval settings..."<<endl;
	while(!writeEEPROM(comPort, 238, 16));
	usleep(10000);
	while(!writeEEPROM(comPort, 240, 16));
	usleep(10000);
	while(!writeEEPROM(comPort, 242, 256));
	usleep(10000);
	while(!writeEEPROM(comPort, 246, 1));
	usleep(10000);

	while(!removeTare(comPort));

	cout<<"Closing port"<<endl;
	close(comPort);
}

//=======================================================================================
// init
//---------------------------------------------------------------------------------------
// Initialize IMU, 
// 1. get scaling factors
// 2. set timer tick interval to 10ms (gives fastest output across most commands)
// 3. try to zero the gyro bias and ensure the G reading is correct
//=======================================================================================
bool init(ComPortHandle comPort)
{
/* 1 */
	while(!readEEPROM(comPort, 230, AccelGainScale));
	while(!readEEPROM(comPort, 130, GyroGainScale));
/* 2 */
	while(!writeEEPROM(comPort, 238, 4));
	usleep(10000);
	while(!writeEEPROM(comPort, 240, 10));
	usleep(10000);
	while(!writeEEPROM(comPort, 242, 250));
	usleep(10000);
	while(!writeEEPROM(comPort, 246, 10));
	usleep(10000);
/* 3 */
//	tareCoord(comPort); // uncomment if there's a need to zero the readings when at current position
	return ( CaptureBias(comPort) && ensureG(comPort) );
}

//Function to ensure acceleration measured is gravitational acceleration G
bool ensureG(ComPortHandle comPort)
{
	Acceleration accel_;
	AngularRate angRate_;
	double measured_g;
	ReadAccelAngRate(comPort, accel_, angRate_);
	measured_g = sqrt(accel_.x*accel_.x + accel_.y*accel_.y + accel_.z*accel_.z);
	printf("Measured G = (%f, %f, %f) = %f \n",accel_.x,accel_.y,accel_.z, measured_g);
	return ( abs(G-measured_g)< 0.05 );
}

//Function to combine two bytes and make a signed short
short MakeShort(Byte msb, Byte lsb)
{
	//short must be a 2 byte integer
	assert(sizeof(short) == 2);
	
	short s = 0;
	
	//map the short to a byte array
	Byte* tmp = (Byte*)&s;
	tmp[1] = msb;
	tmp[0] = lsb;
	
	return s;
}

//=======================================================================================
// Read Gyro-Stabilized orientation quaternion, the instantaneous magnetic field and acceleration
// vectors, and the bias corrected angular rate vector
//=======================================================================================
bool ReadQuatVect(ComPortHandle comPort, Quaternion& quat, Acceleration& accel, AngularRate& angRate)
{
	static const Byte COMMAND_BYTE  = CMD_STAB_QuatVect;
	static const int RESPONSE_SIZE = 31;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) {
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "ReadQuatVect: Invalid response size: " << size << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("ReadQuatVect: Invalid response: %#x\n",response[0]);
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[29], response[30]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "ReadQuatVect: Invalid Checksum"  << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	//conversion factor used to convert the returned values to physical units
	static const float ACCEL_SCALE_CONSTANT = 32768000.0/ AccelGainScale;
	static const float ANGRATE_SCALE_CONSTANT = 32768000.0 / GyroGainScale;

	quat.w = MakeShort(response[1], response[2]) / 8192.0;
	quat.x = MakeShort(response[3], response[4]) / 8192.0;
	quat.y = MakeShort(response[5], response[6]) / 8192.0;
	quat.z = MakeShort(response[7], response[8]) / 8192.0;

	accel.x = MakeShort(response[15], response[16]) / ACCEL_SCALE_CONSTANT * G; // convert to m/s^2
	accel.y = MakeShort(response[17], response[18]) / ACCEL_SCALE_CONSTANT * G;
	accel.z = MakeShort(response[19], response[20]) / ACCEL_SCALE_CONSTANT * G;

	angRate.x = MakeShort(response[21], response[22]) / ANGRATE_SCALE_CONSTANT; // in rad/s
	angRate.y = MakeShort(response[23], response[24]) / ANGRATE_SCALE_CONSTANT;
	angRate.z = MakeShort(response[25], response[26]) / ANGRATE_SCALE_CONSTANT;

	return true;
} // bool ReadQuatVect(ComPortHandle comPort, Quaternion& quat, Acceleration& accel, AngularRate& angRate)

//=======================================================================================
// Read Gyro-Stabilized Acceleration and bias-corrected Angular Rate vectors
//=======================================================================================
bool ReadAccelAngRate(ComPortHandle comPort, Acceleration& accel, AngularRate& angRate)
{
	static const Byte COMMAND_BYTE  = CMD_STAB_Vectors;
	static const int RESPONSE_SIZE = 23;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) {
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "ReadAccelAngRate: Invalid response size: " << size << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("ReadAccelAngRate: Invalid response: %#x\n",response[0]);
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[21], response[22]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "ReadAccelAngRate: Invalid Checksum"  << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	//conversion factor used to convert the returned values to physical units
	static const float ACCEL_SCALE_CONSTANT = 32768000.0/ AccelGainScale;
	static const float ANGRATE_SCALE_CONSTANT = 32768000.0 / GyroGainScale;

	accel.x = MakeShort(response[7], response[8]) / ACCEL_SCALE_CONSTANT * G; // convert to m/s^2
	accel.y = MakeShort(response[9], response[10]) / ACCEL_SCALE_CONSTANT * G;
	accel.z = MakeShort(response[11], response[12]) / ACCEL_SCALE_CONSTANT * G;

	angRate.x = MakeShort(response[13], response[14]) / ANGRATE_SCALE_CONSTANT; // in rad/s
	angRate.y = MakeShort(response[15], response[16]) / ANGRATE_SCALE_CONSTANT;
	angRate.z = MakeShort(response[17], response[18]) / ANGRATE_SCALE_CONSTANT;

	return true;
} // bool ReadAccelAngRate(ComPortHandle comPort, Acceleration& accel, AngularRate& angRate)

//=======================================================================================
// Read Gyro-Stabilized Quaternion
//=======================================================================================
bool ReadQuaternion(ComPortHandle comPort, Quaternion& quaternion)
{
	static const Byte COMMAND_BYTE  = CMD_STAB_Quaternion;
	static const int RESPONSE_SIZE = 13;

	if(write(comPort, &COMMAND_BYTE, 1)!=1)
	{
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "ReadQuaternion: Invalid response size: " << size << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("ReadQuaternion: Invalid response: %#x\n",response[0]);
		tcflush(comPort,TCIOFLUSH);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[11], response[12]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "ReadQuaternion: Invalid Checksum"  << endl;
		tcflush(comPort,TCIOFLUSH);
		return false;
	}

	quaternion.w = MakeShort(response[1], response[2]) / 8192.0;
	quaternion.x = MakeShort(response[3], response[4]) / 8192.0;
	quaternion.y = MakeShort(response[5], response[6]) / 8192.0;
	quaternion.z = MakeShort(response[7], response[8]) / 8192.0;

	return true;
} // bool ReadQuaternion(ComPortHandle comPort, Quaternion& quaternion)

//=======================================================================================
// Read Gyro-Stabilized Euler Angles
//=======================================================================================
bool ReadEulerAngles(ComPortHandle comPort, EulerAngles& eulerAngles)
{
	static const Byte COMMAND_BYTE  = CMD_STAB_Euler;
	static const int RESPONSE_SIZE = 11;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;

	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "ReadEulerAngles: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("ReadEulerAngles: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[9], response[10]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "ReadEulerAngles: Invalid Checksum"  << endl;
		return false;
	}
	
	//conversion factor used to convert the returned values to degrees
	static const float SCALE_AS_DEGREES = 360.0/65536;
	
	eulerAngles.roll  = MakeShort(response[1], response[2]) * SCALE_AS_DEGREES;
	eulerAngles.pitch = MakeShort(response[3], response[4]) * SCALE_AS_DEGREES;
	eulerAngles.yaw   = MakeShort(response[5], response[6]) * SCALE_AS_DEGREES;

	return true;
} // bool ReadEulerAngles(ComPortHandle comPort, EulerAngles& eulerAngles)

//=======================================================================================
// store current gyro outputs as the gyro bias estimate (make sure IMU is stationary)
//=======================================================================================
bool CaptureBias(ComPortHandle comPort)
{
	cout<<"Zeroing gyro now, don't move the IMU. This will take a few seconds..."<<endl;

	static const Byte COMMAND_BYTE = CMD_CAPTURE_BIAS;
	static const int RESPONSE_SIZE = 5;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	
	Byte response[RESPONSE_SIZE] = {0};

	int size;
	do {
		size = read(comPort, &response[0], RESPONSE_SIZE);
	} while(size==0);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "CaptureBias: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("CaptureBias: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[3], response[4]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "CaptureBias: Invalid Checksum"  << endl;
		return false;
	}

	usleep(500000);
	cout<< "Gyro bias captured" << endl;
	return true;
} // bool CaptureBias(ComPortHandle comPort)

//=======================================================================================
// align local body fixed coordinate system with Earth-Fixed reference coordinate system (X north, Y east, Z down).
//=======================================================================================
bool tareCoord(ComPortHandle comPort)
{
	static const Byte COMMAND_BYTE = CMD_TARE;
	Byte COMMAND_DATA[10];
	static const int RESPONSE_SIZE = 5;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	COMMAND_DATA[0] = 0xC1;
	COMMAND_DATA[1] = 0xC3;
	COMMAND_DATA[2] = 0xC5;
	if(write(comPort, COMMAND_DATA, 3)!=3) return false;
	
	cout<< "Tare coordinate system. Do not move IMU. This will take several seconds..." << endl;
	sleep(5);

	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "tareCoord: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("tareCoord: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[3], response[4]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "tareCoord: Invalid Checksum"  << endl;
		return false;
	}
	return true;
} // bool tareCoord(ComPortHandle comPort)

//=======================================================================================
// realign body fixed coordinates to factory default
//=======================================================================================
bool removeTare(ComPortHandle comPort)
{
	static const Byte COMMAND_BYTE = CMD_REMOVE_TARE;
	Byte COMMAND_DATA[10];
	static const int RESPONSE_SIZE = 5;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	COMMAND_DATA[0] = 0xC1;
	COMMAND_DATA[1] = 0xC3;
	COMMAND_DATA[2] = 0xC5;
	if(write(comPort, COMMAND_DATA, 3)!=3) return false;
	
	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "removeTare: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("removeTare: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[3], response[4]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "removeTare: Invalid Checksum"  << endl;
		return false;
	}
	cout<< "Body fixed coordinates reset to factory default" << endl;
	return true;
} // bool removeTare(ComPortHandle comPort)

//=======================================================================================
// read 2 byte signed integer in EEPROM at specified address
//=======================================================================================
bool readEEPROM(ComPortHandle comPort, short addr, short &data)
{
	static const Byte COMMAND_BYTE = CMD_READ_EEPROM_CHECKSUM;
	Byte COMMAND_DATA[10];
	static const int RESPONSE_SIZE = 7;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	COMMAND_DATA[0] = (Byte) ((addr & 0xFF00) >> 8); // address MSB
	COMMAND_DATA[1] = (Byte) (addr & 0x00FF); // address LSB
	if(write(comPort, COMMAND_DATA, 2)!=2) return false;
	
	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "readEEPROM: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("readEEPROM: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[5], response[6]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "readEEPROM: Invalid Checksum"  << endl;
		return false;
	}

	data = MakeShort(response[1], response[2]);

	return true;
} // bool readEEPROM(ComPortHandle comPort, short addr, short &data)

//=======================================================================================
// write integer value to EEPROM at specified address
//=======================================================================================
bool writeEEPROM(ComPortHandle comPort, short addr, short data)
{
	static const Byte COMMAND_BYTE = CMD_WRITE_EEPROM_CHECKSUM;
	Byte COMMAND_DATA[10];
	static const int RESPONSE_SIZE = 7;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	COMMAND_DATA[0] = 0x71;
	COMMAND_DATA[1] = (Byte) ((addr & 0xFF00) >> 8); // address MSB
	COMMAND_DATA[2] = (Byte) (addr & 0x00FF); // address LSB
	COMMAND_DATA[3] = (Byte) ((data & 0xFF00) >> 8); // data MSB
	COMMAND_DATA[4] = (Byte) (data & 0x00FF); // data LSB
	COMMAND_DATA[5] = 0xAA;
	if(write(comPort, COMMAND_DATA, 6)!=6) return false;
	
	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "writeEEPROM: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("writeEEPROM: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[5], response[6]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "writeEEPROM: Invalid Checksum"  << endl;
		return false;
	}

	return true;
} // bool writeEEPROM(ComPortHandle comPort, short addr, short data)

//=======================================================================================
// get firmware version
//=======================================================================================
bool getVersion(ComPortHandle comPort)
{
	static const Byte COMMAND_BYTE = CMD_VERSION;
	static const int RESPONSE_SIZE = 5;

	if(write(comPort, &COMMAND_BYTE, 1)!=1) return false;
	
	Byte response[RESPONSE_SIZE] = {0};
	int size = read(comPort, &response[0], RESPONSE_SIZE);

	//must get all the bytes we want, or it's not a valid read
	if(size != RESPONSE_SIZE)
	{
		cerr << "getVersion: Invalid response size: " << size << endl;
		return false;
	}
	
	// verify first byte matches the command byte
	if(response[0] != COMMAND_BYTE)
	{
		printf("getVersion: Invalid response: %#x\n",response[0]);
		return false;
	}
	
	//Verify the checksum
	short responseChecksum = MakeShort(response[3], response[4]);
	short calculatedChecksum = COMMAND_BYTE;
	for(int i = 1; i< RESPONSE_SIZE-2; i+=2)
	{
		calculatedChecksum += MakeShort(response[i], response[i+1]);
	}
		
	if(calculatedChecksum != responseChecksum)
	{
		cerr << "getVersion: Invalid Checksum"  << endl;
		return false;
	}

	short version = MakeShort(response[1],response[2]);
	cout<< "IMU Version " << version << endl;
	return true;
} // bool getVersion(ComPortHandle comPort)


}; // class IMU_3DMGX1
#endif
