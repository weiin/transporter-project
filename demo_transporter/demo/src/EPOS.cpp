/*
 *  EPOS Class 
 */
#include <exception>
#include "transporter/EPOS.h"

#define BAUDRATE B57600 
//#define DEBUG

////////////////////////////////////////////////////////////////////////////////
// Constructor
EPOS::EPOS()
{
	nodeid=0x01;
}

////////////////////////////////////////////////////////////////////////////////
// Destructor
EPOS::~EPOS()
{
	closePort();
}


////////////////////////////////////////////////////////////////////////////////
// Open the EPOS port
int
EPOS::openPort(const char* port_name)
{
  	// Open the port
	fd = open(port_name, O_RDWR | O_NOCTTY );
	if (fd <0)
	{
		printf("Cannot open port %s \n", port_name);     
	}
	else
	{
		port=port_name; //save a copy of the port name
		printf("Port %s opened with fd=%i\n", port,fd);

		tcgetattr(fd,&oldtio); /* save current serial port settings */
		bzero(&newtio, sizeof(newtio)); /* clear struct for new port settings */

		/* 
			BAUDRATE: Set bps rate. You could also use cfsetispeed and cfsetospeed.
			CS8     : 8n1 (8bit,no parity,1 stopbit)
			CLOCAL  : local connection, no modem contol
			CREAD   : enable receiving characters
		*/
		newtio.c_cflag = BAUDRATE | CS8 | CLOCAL | CREAD;
		newtio.c_oflag = 0;
		     
		newtio.c_cc[VMIN] = 0;	/* min number of char read, 0 means enable timeout */
		newtio.c_cc[VTIME] = 1;	/* timeout in deciseconds */

		/* 
			now clean the Tx and Rx line and activate the settings for the port
		*/
		tcflush(fd, TCIOFLUSH);
		tcsetattr(fd,TCSANOW,&newtio);
	}
	return fd;
} //EPOS::openPort


////////////////////////////////////////////////////////////////////////////////
// Close the EPOS port
bool
EPOS::closePort()
{
	if (fd != -1)
	{    
		tcsetattr(fd,TCSANOW,&oldtio);  //restore port settings
		if (close(fd) != 0)
		{
			printf("Unable to close serial port %s, fd=%i. Error: %s\n",port,fd,strerror(errno));
			fd = -1;
			return false;
		}
		else 
		{
			printf("Port %s, fd=%i is closed successfully\n",port,fd);
			fd = -1;
			return true;
		}
	}
	else //printf("Serial port already closed\n");
	return true;
}


////////////////////////////////////////////////////////////////////////////////
// Calculate CRC of data
unsigned short 
EPOS::CRCCalc(unsigned short buf[],unsigned short len)
{
  	unsigned short shifter, c;
  	unsigned short carry;
   	unsigned short crc = 0;            
   	int i = 0;
   	while (len > 0)
   	{
    	shifter = 0x8000;
        c = buf[i++];
        do
        {
         	carry = (unsigned short) (crc & 0x8000);
            crc <<= 1;
            if ((c & shifter)>0) crc++;
            if (carry > 0) crc ^= 0x1021;
            shifter >>= 1;
       	} while (shifter > 0);
        len--;
 	}
    return crc;
}

////////////////////////////////////////////////////////////////////////////////
// Check ACK of data sent
bool 
EPOS::checkAck()
{
	int cnt = 0;
	int res;
    unsigned char rd;

   	while (true)
    {
		res = read(fd,&rd,1); 
		if (rd == 0x4f)    // 'O' = 0x4f
		{
#ifdef DEBUG
	printf("Rcvd 0x%x\n",rd);
#endif
         	break;
      	}
      	else
		{
#ifdef DEBUG
	printf("Instead of ACK I got 0x%x\n",rd);
#endif
        	cnt++;
          	if (cnt < 10)
          	{
             	continue;
           	}
           	else
           	{
             	return false;
          	}
      	}
  	}
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Check for reply of opcode 
bool 
EPOS::checkReplyOpCode()
{
	int cnt = 0;
    unsigned char rd;
	int res;

    while (true)
    {
		res = read(fd,&rd,1); 
      	if (res!=0 && rd == 0x00)
      	{
#ifdef DEBUG
	printf("opCode 0x%x received\n",rd);
#endif
         	break;
       	}
      	else
       	{
#ifdef DEBUG
	printf("Try %i, res=%i: Instead of opCode 0x00 I got 0x%x\n",cnt+1,res,rd);
#endif
       		cnt++;
           	if (cnt < 10)
           	{
            	continue;
          	}
           	else
           	{
             	return false;
           	}
       	}
  	}
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Read Data 
bool 
EPOS::readData(unsigned char buf[],int len)
{
	int cnt = 0;
	int res;
	unsigned char rd;

    while (len > 0)
    {
    	res = read(fd,&rd,1);
        buf[cnt] = rd;
		cnt++;
		len--;
   	}
    return true;
}

////////////////////////////////////////////////////////////////////////////////
// Read Object 
bool 
EPOS::readObject(unsigned char inxhigh, unsigned char inxlow, unsigned char subindex,unsigned char bparam[],int& lx)
{
	unsigned char sbuff[20];
	int res;
	unsigned short x, crc;
	int len;
                
  	// send opcode 
    sbuff[0] = 0x10; // opcode
	res=write(fd,sbuff,1);

    // wait for acknowledge from EPOS
    if (!checkAck())
    {
#ifdef DEBUG
	printf("readObject: Did not get acknowledge after opcode sent\n");
#endif
    	return false;
    }
	
    // send len-1
    sbuff[0] = 0x01; // len -1 
    res=write(fd,sbuff,1);
               
	// send data
    sbuff[0] = inxlow; // index low byte
    sbuff[1] = inxhigh; // index high byte
    sbuff[2] = subindex; // subindex
    sbuff[3] = nodeid; // node id nodeid
	res=write(fd,sbuff,4);
    
    // calc crc and send
    data[0] = 0x1001;
    x = (unsigned short)(inxhigh * 256);
    data[1] = (unsigned short)(x + inxlow);
    x = (unsigned short)(nodeid * 256);
    data[2] = (unsigned short)(x + subindex);
    data[3] = 0x0000;  // zero data
    x = CRCCalc(data,4);
    crc = x;
    sbuff[0] = (unsigned char)(x & 0x00ff);
    sbuff[1] = (unsigned char)((x & 0xff00) >> 8);
    res=write(fd,sbuff,2);

	// wait for end acknowledge from EPOS
    if (!checkAck())
    {
#ifdef DEBUG
	printf("readObject: Did not get acknowledge after data sent\n");
#endif
     	return false;
    }
	
    // receive response from epos
    if (!checkReplyOpCode())
    {
#ifdef DEBUG
	printf("readObject: Did not get opcode reply\n");
#endif
     	return false;
    }
              
	// send ack
    sbuff[0] = 0x4F; // ack
	res=write(fd,sbuff,1);

   	// receive len - 1
    if (!readData(sbuff,1))
    {
#ifdef DEBUG
	printf("readObject: Did not get len of data after ack\n");
#endif
     	return false;
    }
	
    len = sbuff[0] + 1;
    lx = len;
    len = len * 2;
	
    // read data of length len
    if (!readData(bparam, len))
    {
#ifdef DEBUG
	printf("readObject: Did not get data\n");
#endif
     	return false;
    }
	
    // read crc
    if (!readData(sbuff, 2))
    {
#ifdef DEBUG
	printf("readObject: Did not get crc\n");
#endif
     	return false;
    }
	
    // check crc and send ack
    // calc crc
    data[0] = 0x0003;
    x = (unsigned short)(bparam[1] * 256);
    data[1] = (unsigned short)(x + bparam[0]);
    x = (unsigned short)(bparam[3] * 256);
    data[2] = (unsigned short)(x + bparam[2]);
    x = (unsigned short)(bparam[5] * 256);
    data[3] = (unsigned short)(x + bparam[4]);
    x = (unsigned short)(bparam[7] * 256);
    data[4] = (unsigned short)(x + bparam[6]);
    data[5] = 0x0000;  // zero data
    x = CRCCalc(data, 6);
    crc = x;

    x = (unsigned short)(sbuff[1] * 256);
    x = (unsigned short)(x + sbuff[0]);
    if (x == crc)
    {
    	// send okay ack
        sbuff[0] = 0x4F;
        res=write(fd,sbuff,1);
        return true;
    }
    else
    {
        sbuff[0] = 0x46; // send failed
        res=write(fd,sbuff,1);
#ifdef DEBUG
	printf("readObject: Wrong crc\n");
#endif
        return false;
    }
}

////////////////////////////////////////////////////////////////////////////////
// Write Object 
bool 
EPOS::writeObject(unsigned char inxhigh, unsigned char inxlow, unsigned char subindex,unsigned char bparam[])
{
	unsigned char sbuff[20];               
    unsigned short x, crc, err, err1;
   	int len,res;            

    // send opcode 
    sbuff[0] = 0x11; // opcode
    res=write(fd,sbuff,1);

    // wait for acknowledge from slave
    if (!checkAck())
    {
#ifdef DEBUG
	printf("writeObject: Did not get acknowledge after opcode sent\n");
#endif
     	return false;
    }
	
    // send len-1
    sbuff[0] = 0x03; // len -1 
	res=write(fd,sbuff,1);
                
    // send data
    sbuff[0] = inxlow; // index low byte
    sbuff[1] = inxhigh; // index high byte
    sbuff[2] = subindex; // subindex
    sbuff[3] = nodeid; // node id
    sbuff[4] = bparam[0];    // 4 data bytes
    sbuff[5] = bparam[1];    // 
    sbuff[6] = bparam[2];    // 
    sbuff[7] = bparam[3];    // 
	res=write(fd,sbuff,8);

    // calc crc and send
    data[0] = 0x1103;
    x = (unsigned short)(inxhigh * 256);
    data[1] = (unsigned short)(x + inxlow);
    x = (unsigned short)(nodeid * 256);
    data[2] = (unsigned short)(x + subindex);
    x = (unsigned short)(bparam[1] * 256);
    data[3] = (unsigned short)(x + bparam[0]);
    x = (unsigned short)(bparam[3] * 256);
    data[4] = (unsigned short)(bparam[2] + x);
    data[5] = 0x0000;  // zero data
    x = CRCCalc(data, 6);
    crc = x;
    sbuff[0] = (unsigned char)(x & 0x00ff);
    sbuff[1] = (unsigned char)((x & 0xff00) >> 8);
	res=write(fd,sbuff,2);

    // wait for end acknowledge from slave
    if (!checkAck())
    {
#ifdef DEBUG
	printf("writeObject: Did not get acknowledge after data sent\n");
#endif
		return false;
    }

    // receive response from epos
    if (!checkReplyOpCode())
    {
#ifdef DEBUG
	printf("writeObject: Did not get opcode reply\n");
#endif
   		return false;
    }

    // send ack
    sbuff[0] = 0x4F; // ack
    res=write(fd,sbuff,1);
    // receive len - 1
    if (!readData(sbuff, 1))
    {
#ifdef DEBUG
	printf("writeObject: Did not get len of data after ack\n");
#endif
     	return false;
    }
    len = (sbuff[0] + 1) * 2;
    // read data of length len
    if (!readData(sbuff, len))
    {
#ifdef DEBUG
	printf("writeObject: Did not get data\n");
#endif
    	return false;
    }
    // error code
    err = (unsigned short)(sbuff[1] * 256);
    err = (unsigned short)(err + sbuff[0]);
    err1 = (unsigned short)(sbuff[3] * 256);
    err1 = (unsigned short)(err + sbuff[2]);
	
    // read crc
    if (!readData(sbuff, 2))
    {
#ifdef DEBUG
	printf("writeObject: Did not get crc\n");
#endif
     	return false;
    }
    // check crc and send ack
    // calc crc
    data[0] = 0x0001;
    data[1] = err;
    data[2] = err1;
    data[3] = 0x0000;  // zero data
    x = CRCCalc(data, 4);
    crc = x;
	
    x = (unsigned short)(sbuff[1] * 256);
    x = (unsigned short)(x + sbuff[0]);
    if (x == crc)
    {
    	// send okay ack
        sbuff[0] = 0x4F;
        res=write(fd,sbuff,1);
        return true;
    }
    else
    {
        sbuff[0] = 0x46; // send failed
        res=write(fd,sbuff,1);
#ifdef DEBUG
	printf("writeObject: Wrong crc\n");
#endif
        return false;
    }
}


////////////////////////////////////////////////////////////////////////////////
// Get Error 
unsigned int 
EPOS::getError(unsigned char bparam[])
{

 	unsigned int err, temp;

    err = (unsigned int)(bparam[0]);
    temp = (unsigned int)(bparam[1]);
    err = err + (temp << 8);
    temp = (unsigned int)(bparam[2]);
    err = err + (temp << 16);
    temp = (unsigned int)(bparam[3]);
    err = err + (temp << 24);
    return err;
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetCurrentRegulatorPGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xf6, 0x01,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	CurrentRegulatorPGain = (short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetCurrentRegulatorPGain(short pgain)
{

 	bool rs;
    unsigned char bparam[10];

    CurrentRegulatorPGain = pgain;
    bparam[0] = (unsigned char)(0x00ff & pgain);
    bparam[1] = (unsigned char)((0xff00 & pgain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xf6, 0x01, bparam);
#ifdef DEBUG	
	if (!rs)
	{                
    	printf("Error in Setting Current Regulator P-Gain");
	}
#endif
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetCurrentRegulatorIGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xf6, 0x02,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	CurrentRegulatorIGain = (short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetCurrentRegulatorIGain(short pgain)
{

 	bool rs;
    unsigned char bparam[10];

    CurrentRegulatorIGain = pgain;
    bparam[0] = (unsigned char)(0x00ff & pgain);
    bparam[1] = (unsigned char)((0xff00 & pgain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xf6, 0x02, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Current Regulator I-Gain");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetEncoderCount()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x22, 0x10, 0x01,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	IncrementalEncoderCount = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetEncoderCount(unsigned short count)
{

 	bool rs;
    unsigned char bparam[10];

    IncrementalEncoderCount = count;
    bparam[0] = (unsigned char)(0x00ff & count);
    bparam[1] = (unsigned char)((0xff00 & count) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x22, 0x10, 0x01, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Encoder Count");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetEncoderType()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x22, 0x10, 0x02,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionSensorType = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetEncoderType(unsigned short type)
{

 	bool rs;
    unsigned char bparam[10];

    PositionSensorType = type;
    bparam[0] = (unsigned char)(0x00ff & type);
    bparam[1] = (unsigned char)((0xff00 & type) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x22, 0x10, 0x02, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Encoder Type");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetMotorType()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x64, 0x02, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	MotorType = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetMotorType(unsigned short type)
{

 	bool rs;
    unsigned char bparam[10];

    MotorType = type;
    bparam[0] = (unsigned char)(0x00ff & type);
    bparam[1] = (unsigned char)((0xff00 & type) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x64, 0x02, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Motor Type");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetMotorMaxContCurrent()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x64, 0x10, 0x01,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	MotorMaxContinuousCurrent = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetMotorMaxContCurrent(unsigned short amp)
{

 	bool rs;
    unsigned char bparam[10];

    MotorMaxContinuousCurrent = amp;
    bparam[0] = (unsigned char)(0x00ff & amp);
    bparam[1] = (unsigned char)((0xff00 & amp) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x64, 0x10, 0x01, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Max Continuous Current");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetMotorMaxPeakCurrent()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x64, 0x10, 0x02,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	MotorMaxPeakCurrent = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetMotorMaxPeakCurrent(unsigned short amp)
{

 	bool rs;
    unsigned char bparam[10];

    MotorMaxPeakCurrent = amp;
    bparam[0] = (unsigned char)(0x00ff & amp);
    bparam[1] = (unsigned char)((0xff00 & amp) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x64, 0x10, 0x02, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Max Peak Current");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetMaxFollowError()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x65, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	MaxFollowError = (unsigned int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7] * 16777216);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetMaxFollowError(unsigned int error)
{

 	bool rs;
    unsigned char bparam[10];

    MaxFollowError = error;
    bparam[0] = (unsigned char)(0x000000ff & error);
    bparam[1] = (unsigned char)((0x0000ff00 & error) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & error) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & error) >> 24);

    rs = writeObject(0x60, 0x65, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Max Follow Error");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionPGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xfb, 0x01,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionPGain = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionPGain(short gain)
{

 	bool rs;
    unsigned char bparam[10];

    PositionPGain = gain;
    bparam[0] = (unsigned char)(0x00ff & gain);
    bparam[1] = (unsigned char)((0xff00 & gain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xfb, 0x01, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position P-Gain");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionIGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xfb, 0x02,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionIGain = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionIGain(short gain)
{

 	bool rs;
    unsigned char bparam[10];

    PositionIGain = gain;
    bparam[0] = (unsigned char)(0x00ff & gain);
    bparam[1] = (unsigned char)((0xff00 & gain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xfb, 0x02, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position I-Gain");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionDGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xfb, 0x03,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionDGain = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionDGain(short gain)
{

 	bool rs;
    unsigned char bparam[10];

    PositionDGain = gain;
    bparam[0] = (unsigned char)(0x00ff & gain);
    bparam[1] = (unsigned char)((0xff00 & gain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xfb, 0x03, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position D-Gain");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetVelocityPGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xf9, 0x01,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	VelocityPGain = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetVelocityPGain(short gain)
{

 	bool rs;
    unsigned char bparam[10];

    VelocityPGain = gain;
    bparam[0] = (unsigned char)(0x00ff & gain);
    bparam[1] = (unsigned char)((0xff00 & gain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xf9, 0x01, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Velocity P-Gain");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetVelocityIGain()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0xf9, 0x02,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	VelocityIGain = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetVelocityIGain(short gain)
{

 	bool rs;
    unsigned char bparam[10];

    VelocityIGain = gain;
    bparam[0] = (unsigned char)(0x00ff & gain);
    bparam[1] = (unsigned char)((0xff00 & gain) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0xf9, 0x02, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Velocity I-Gain");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetStatusWord()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x41, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	StatusWord = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
	else printf("Failed to GetStatusWord\n");
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetStatusWord(unsigned short swd)
{

 	bool rs;
    unsigned char bparam[10];

    StatusWord = swd;
    bparam[0] = (unsigned char)(0x00ff & swd);
    bparam[1] = (unsigned char)((0xff00 & swd) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0x41, 0x00, bparam);
#ifdef DEBUG	
    if (!rs)
    {                
     	printf("Failed to SetStatusWord\n");
    }
#endif
}

////////////////////////////////////////////////////////////////////////////////
 
bool 
EPOS::GetControlWord()
{
 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x40, 0x00,bparam,lx))
    {
     	// check for error code		
        err = getError(bparam);
        if (err == 0)
        {
         	ControlWord = (unsigned short)(bparam[4] + bparam[5] * 256);
			return true;
        }
		else return false;
  	}
	else 
	{
		printf("Failed to GetControlWord\n");
		return false;
	}
}

////////////////////////////////////////////////////////////////////////////////
//  
bool 
EPOS::SetControlWord(unsigned short cwd)
{

 	bool rs;
    unsigned char bparam[10];

    ControlWord = cwd;
    bparam[0] = (unsigned char)(0x00ff & cwd);
    bparam[1] = (unsigned char)((0xff00 & cwd) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;
	
    rs = writeObject(0x60, 0x40, 0x00, bparam);
#ifdef DEBUG	
    if (!rs)
    {                
     	printf("Failed to SetControlWord\n");
		return false;
    }
#endif
	return true;
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetCurrentModeSetting()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x20, 0x30, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	CurrentModeSetting = (short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetCurrentModeSetting(short current)
{

 	bool rs;
    unsigned char bparam[10];

    CurrentModeSetting = current;
    bparam[0] = (unsigned char)(0x00ff & current);
    bparam[1] = (unsigned char)((0xff00 & current) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x20, 0x30, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Current Mode");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetThermalTimeConstantWinding()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

	
    if (readObject(0x64, 0x10, 0x05,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
			
         	ThermalTimeConstantWinding = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetThermalTimeConstantWinding(unsigned short time)
{

 	bool rs;
    unsigned char bparam[10];

    ThermalTimeConstantWinding = time;
    bparam[0] = (unsigned char)(0x00ff & time);
    bparam[1] = (unsigned char)((0xff00 & time) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x64, 0x10, 0x05, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Thermal Time Constant Winding");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetRS232BaudRate()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x20, 0x02, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	RS232BaudRate = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetRS232BaudRate(unsigned short rate)
{

 	bool rs;
    unsigned char bparam[10];

    RS232BaudRate = rate;
    bparam[0] = (unsigned char)(0x00ff & rate);
    bparam[1] = (unsigned char)((0xff00 & rate) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x20, 0x02, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting RS232 BaudRate");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetCurrent()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x78, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	ActualCurrent = (short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPosition()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x64, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	ActualPosition = (int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7] * 16777216);
#ifdef DEBUG
			printf("in GetPosition actual pos = %i\n",ActualPosition);
#endif
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetVelocity()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x6c, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	ActualVelocity = (int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7] * 16777216);
#ifdef DEBUG
			printf("in GetVelocity actual vel = %i\n",ActualVelocity);
#endif
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionProfileVelocity()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x81, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionProfileVelocity = (unsigned int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7]* 16777216);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//
void 
EPOS::SetPositionProfileVelocity(unsigned int velocity)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileVelocity = velocity;
    bparam[0] = (unsigned char)(0x000000ff & velocity);
    bparam[1] = (unsigned char)((0x0000ff00 & velocity) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & velocity) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & velocity) >> 24);

    rs = writeObject(0x60, 0x81, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position Profile Velocity");
    //}
}

////////////////////////////////////////////////////////////////////////////////
//
void 
EPOS::SetMaxProfileVelocity(unsigned int velocity)
{

 	bool rs;
    unsigned char bparam[10];

    MaxProfileVelocity = velocity;
    bparam[0] = (unsigned char)(0x000000ff & velocity);
    bparam[1] = (unsigned char)((0x0000ff00 & velocity) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & velocity) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & velocity) >> 24);

    rs = writeObject(0x60, 0x7f, 0x00, bparam);
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionProfileDeceleration()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x84, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionProfileDeceleration = (unsigned int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7]* 16777216);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionProfileDeceleration(unsigned int deceleration)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileDeceleration = deceleration;
    bparam[0] = (unsigned char)(0x000000ff & deceleration);
    bparam[1] = (unsigned char)((0x0000ff00 & deceleration) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & deceleration) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & deceleration) >> 24);

    rs = writeObject(0x60, 0x84, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position Profile deceleration");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionProfileAcceleration()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x83, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionProfileAcceleration = (unsigned int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7]* 16777216);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionProfileAcceleration(unsigned int acceleration)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileAcceleration = acceleration;
    bparam[0] = (unsigned char)(0x000000ff & acceleration);
    bparam[1] = (unsigned char)((0x0000ff00 & acceleration) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & acceleration) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & acceleration) >> 24);

    rs = writeObject(0x60, 0x83, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position Profile acceleration");
    //}
}

////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionWindow()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x67, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionProfileWindow = (unsigned int)(bparam[4] + bparam[5] * 256 + bparam[6] * 65536 + bparam[7]* 16777216);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionWindow(unsigned int window)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileWindow = window;
    bparam[0] = (unsigned char)(0x000000ff & window);
    bparam[1] = (unsigned char)((0x0000ff00 & window) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & window) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & window) >> 24);

    rs = writeObject(0x60, 0x67, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position Position window");
    //}
}


////////////////////////////////////////////////////////////////////////////////
 
void 
EPOS::GetPositionWindowTime()
{

 	unsigned char bparam[10];
    int lx=0;
    unsigned int err; 

    if (readObject(0x60, 0x68, 0x00,bparam,lx))
    {
     	// check for error code
        err = getError(bparam);
        if (err == 0)
        {
         	PositionProfileWindowTime = (unsigned short)(bparam[4] + bparam[5] * 256);
        }
  	}
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetPositionWindowTime(unsigned short time)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileWindowTime = time;
    bparam[0] = (unsigned char)(0x00ff & time);
    bparam[1] = (unsigned char)((0xff00 & time) >> 8);
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0x68, 0x00, bparam);
    //if (!rs)
    //{                
    // 	printf("Error in Setting Position Window TIme");
    //}
}


////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetOperationMode(char mode)
{
	bool rs;
    unsigned char bparam[10];

    OperationMode = mode;
    bparam[0] = mode;
    bparam[1] = 0x00;
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0x60, 0x00, bparam);
}
////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetMotionProfileType(char profileType)
{
	bool rs;
    unsigned char bparam[10];

    MotionProfileType = profileType;
    bparam[0] = profileType;
    bparam[1] = 0x00;
    bparam[2] = 0x00;
    bparam[3] = 0x00;

    rs = writeObject(0x60, 0x86, 0x00, bparam);
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetTargetPosition(int position)
{

 	bool rs;
    unsigned char bparam[10];

    PositionProfileTargetPosition = position;
    bparam[0] = (unsigned char)(0x000000ff & position);
    bparam[1] = (unsigned char)((0x0000ff00 & position) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & position) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & position) >> 24);

    rs = writeObject(0x60, 0x7a, 0x00, bparam);
    
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SetTargetVelocity(int velocity)
{

	bool rs;
	unsigned char bparam[10];

#ifdef DEBUG
	printf("setting velocity to %i...\n",velocity);
#endif
    bparam[0] = (unsigned char)(0x000000ff & velocity);
    bparam[1] = (unsigned char)((0x0000ff00 & velocity) >> 8);
    bparam[2] = (unsigned char)((0x00ff0000 & velocity) >> 16);
    bparam[3] = (unsigned char)((0xff000000 & velocity) >> 24);

    rs = writeObject(0x60, 0xff, 0x00, bparam);
    
}


////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::MoveToEnableOperation()
{
 	unsigned short statwd = 0x0000;

    GetStatusWord();
    statwd = StatusWord;
#ifdef DEBUG	
	printf("Unmasked StatusWord: %x\n",statwd);
#endif
    statwd = (unsigned short)(statwd & 0x417f);  // maskoff don't care bits
#ifdef DEBUG	
	printf("StatusWord: %x\n",statwd);
#endif

    switch (statwd)
    {
    	case 0x0000: // start state
        	break;
        case 0x0100: // Not Ready to Switch On
          	break;
        case 0x0140: // Switch On Disabled
            ShutDown();
			usleep(50000);
            SwitchOn();
			usleep(50000);
            EnableOperation();
			usleep(50000);
            break;
        case 0x0121: // Ready to Switch On
            SwitchOn();
			usleep(50000);
            EnableOperation();
            break;
        case 0x0123: // Switched On
            EnableOperation();
            break;
        case 0x4123: // Refresh
            break;
        case 0x4133: // Measure Init
            break;
       	case 0x0137: // Operation Enable
            break;
      	case 0x0117: // Quick Stop Active
          	EnableOperation();
        	break;
  		case 0x010f: // Fault Reaction Active (disabled)
          	break;
      	case 0x011f: // Fault Reaction Active (enabled)
           	break;
      	case 0x0108: // Fault
           	FaultReset();
			usleep(50000);
         	ShutDown();
			usleep(50000);
          	SwitchOn();
			usleep(50000);
          	EnableOperation();
          	break;
		default:
			FaultReset();
			usleep(50000);
         	ShutDown();
			usleep(50000);
          	SwitchOn();
			usleep(50000);
          	EnableOperation();
			break;
  	} //switch (statwd)

} //MoveToEnableOperation()

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::ShutDown()
{
 	unsigned short cwd = 0x0000;
    GetControlWord();
    cwd = ControlWord;
    cwd = (unsigned short)(cwd & 0xff7e);
    cwd = (unsigned short)(cwd | 0x0006);
    SetControlWord(cwd);
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::SwitchOn()
{
 	unsigned short cwd = 0x0000;
    GetControlWord();
    cwd = ControlWord;
    cwd = (unsigned short)(cwd & 0xff7f);
    cwd = (unsigned short)(cwd | 0x0007);
    SetControlWord(cwd);
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::DisableOperation()
{
 	unsigned short cwd = 0x0000;
    GetControlWord();
    cwd = ControlWord;
    cwd = (unsigned short)(cwd & 0xff77);
    cwd = (unsigned short)(cwd | 0x0007);
    SetControlWord(cwd);
}

////////////////////////////////////////////////////////////////////////////////
//  
void 
EPOS::EnableOperation()
{
 	unsigned short cwd = 0x0000;
    GetControlWord();
    cwd = ControlWord;
    cwd = (unsigned short)(cwd & 0xff7f);
    cwd = (unsigned short)(cwd | 0x000f);
    SetControlWord(cwd);
}

////////////////////////////////////////////////////////////////////////////////
//  
bool 
EPOS::FaultReset()
{
 	unsigned short cwd = 0x0000;

    if (!GetControlWord())
	{
		printf("No reply from EPOS. Check connection!\n");
		return false;
	}
	
    cwd = ControlWord;
    cwd = (unsigned short)(cwd & 0xff7f);  
    SetControlWord(cwd);

    cwd = ControlWord;
    cwd = (unsigned short)(cwd | 0x0080);
	return SetControlWord(cwd);
	
}
