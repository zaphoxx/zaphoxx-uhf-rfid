/*
	CC1101A_CPP
	Basic CC1101 library for Arduino Due. It makes use of the Arduino SPI.h library.
	CC1101A uses the standart SPI library and definitions
*/

#ifndef CC1101A_CPP
#define CC1101A_CPP

#include "CC1101A.h"
#include "CC1101_REGS.h"


/****************************************************************
*FUNCTION NAME:SpiInit
*FUNCTION     :spi communication initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void CC1101A::SpiInit()
{
  SPI.end();
  
  pinMode(SCK_PIN, OUTPUT);
  pinMode(MOSI_PIN, OUTPUT);
  pinMode(MISO_PIN, INPUT);
  pinMode(RX_PIN, OUTPUT); // CSN / CSS / SS 
  
  /* ********************* */
  pinMode(4, OUTPUT); 
  pinMode(10, OUTPUT);
  pinMode(52, OUTPUT);
  
  /* Initialize SPI For RX CC1101 */
  SPI.begin(RX_PIN);
  SPI.setDataMode(RX_PIN, SPI_MODE0);
  SPI.setClockDivider(RX_PIN, 21); 	// 4MHz // default is 4MHz higher rates are possible but playing it safe here
  SPI.setBitOrder(RX_PIN, MSBFIRST);
}

/****************************************************************
*FUNCTION NAME: GDO_Set()
*FUNCTION     : set GDO0_TX,GDO2_TX pin
*INPUT        : none
*OUTPUT       : none
****************************************************************/
void CC1101A::GDO_Set (void)
{
	pinMode(RX_GDO0_PIN, INPUT);
	pinMode(RX_GDO2_PIN, INPUT);
}

/****************************************************************
*FUNCTION NAME:Reset
*FUNCTION     :CC1101 reset //details refer datasheet of CC1101/CC1100//
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void CC1101A::Reset(void)
{
	Serial.println("[CC1101A] RESET *");
	digitalWrite(SCK_PIN, HIGH);
    digitalWrite(MOSI_PIN, LOW);
	digitalWrite(RX_PIN, LOW);
	delay(1);
	digitalWrite(RX_PIN, HIGH);
	delay(1);
	digitalWrite(RX_PIN, LOW);
    while(digitalRead(MISO_PIN));
	SpiStrobe(CC1101_SRES);
    while(digitalRead(MISO_PIN));
    delay(1);
	//Serial.println("[RX] ***** RESET COMPLETE *****");
/*
	Note that the above reset procedure is
	only required just after the power supply is
	first turned on. If the user wants to reset
	the CC1101 after this, it is only necessary to
	issue an SRES command strobe
*/
}

/****************************************************************
*FUNCTION NAME:Init
*FUNCTION     :CC1101 initialization
*INPUT        :none
*OUTPUT       :none
****************************************************************/
byte CC1101A::Init()
{
	SpiInit();						
	GDO_Set();										
	//SpiStrobe(CC1101_SRES);
	Reset();
  
    // print out basic cc1101 info: version & partnum as a simple check
    // partnum should be 0x14 or 0x04; version is usually 0x00
    this->version = SpiReadStatus(CC1101_VERSION);
    this->xpart   = SpiReadStatus(CC1101_PARTNUM);

    RegConfigSettings();							
    SpiWriteBurstReg(CC1101_PATABLE,PaTable,8);
    SpiStrobe(CC1101_SCAL);
    return 0;
}


/****************************************************************
*FUNCTION NAME:SpiWriteReg
*FUNCTION     :CC1101 write data to register
*INPUT        :addr: register address; value: register value
*OUTPUT       :none
****************************************************************/
void CC1101A::SpiWriteReg(byte addr, byte value)
{
  SPI.transfer(RX_PIN, addr, SPI_CONTINUE);
  SPI.transfer(RX_PIN, value);
}

/****************************************************************
*FUNCTION NAME:SpiWriteBurstReg
*FUNCTION     :CC1101 write burst data to register
*INPUT        :addr: register address; buffer:register value array; num:number to write
*OUTPUT       :none
****************************************************************/
void CC1101A::SpiWriteBurstReg(byte addr, byte *buffer, byte num)
{
	byte i, temp;

	temp = addr | WRITE_BURST;
    SPI.transfer(RX_PIN, temp, SPI_CONTINUE);
    for (i = 0; i < num; i++)
 	{
        if (i == (num - 1)){
			SPI.transfer(RX_PIN, buffer[i]);
		}else{
			SPI.transfer(RX_PIN, buffer[i], SPI_CONTINUE);
		}
	}
}

/****************************************************************
*FUNCTION NAME:SpiStrobe
*FUNCTION     :CC1101 Strobe
*INPUT        :strobe: command; //refer define in CC1101.h//
*OUTPUT       :none
****************************************************************/
void CC1101A::SpiStrobe(byte strobe)
{
	SPI.transfer(RX_PIN, strobe);
}

/****************************************************************
*FUNCTION NAME:SpiReadReg
*FUNCTION     :CC1101 read data from register
*INPUT        :addr: register address
*OUTPUT       :register value
****************************************************************/
byte CC1101A::SpiReadReg(byte addr) 
{
	byte temp, value;
    temp = addr|READ_SINGLE;
	value = SPI.transfer(RX_PIN, temp, SPI_CONTINUE);
	value = SPI.transfer(RX_PIN, 0x00);
	return value;
}

/****************************************************************
*FUNCTION NAME:SpiReadBurstReg
*FUNCTION     :CC1101 read burst data from register
*INPUT        :addr: register address; buffer:array to store register value; num: number to read
*OUTPUT       :none
****************************************************************/
void CC1101A::SpiReadBurstReg(byte addr, byte *buffer, byte num)
{
	byte i,temp;
	temp = addr | READ_BURST;
	SPI.transfer(RX_PIN, temp, SPI_CONTINUE);
	for(i=0;i<num;i++)
	{
		if (i < (num-1)) {
			buffer[i]=SPI.transfer(RX_PIN, 0x00, SPI_CONTINUE);
		}else{
			buffer[i]=SPI.transfer(RX_PIN, 0x00);
		}
	}
}

/****************************************************************
*FUNCTION NAME:SpiReadStatus
*FUNCTION     :CC1101 read status register
*INPUT        :addr: register address
*OUTPUT       :status value
****************************************************************/
byte CC1101A::SpiReadStatus(byte addr) 
{
	byte value,temp;

	temp = addr | READ_BURST;
	value = SPI.transfer(RX_PIN, temp, SPI_CONTINUE);
	value = SPI.transfer(RX_PIN, 0x00);
	return value;
}

/****************************************************************
*FUNCTION NAME:RegConfigSettings
*FUNCTION     :CC1101 register config //details refer datasheet of CC1101/CC1100//
*INPUT        :none
*OUTPUT       :none
****************************************************************/
void CC1101A::RegConfigSettings(){
/* custom initial basic register settings for CC1101 
* Register Setup for
*  868e6 Hz
*  datarate 80kBaud
*  ASK/OOK modulation
*  no crc
*  no preamble
*  no sync
*  infinite packet length selected
*  manual calibration (once in setup) needs only be updated if power adjustments are done.
*/	
    // sync word and automatic gain control
    const byte sync1 = 0xAD;
    const byte sync0 = 0x23;
    const byte agc0 = 0x90; 
    const byte agc2 = 0x04;
  
	SpiWriteReg(CC1101_IOCFG2,      0x00);  /* Associated to the RX FIFO: Asserts when RX FIFO is filled at or above the RX FIFO threshold. De-asserts when RX FIFO 
                                              is drained below the same threshold. */
	SpiWriteReg(CC1101_IOCFG0,      0x06);  /*Asserts when sync word has been sent / received, and de-asserts at the end of the packet. 
                                          In RX, the pin will also deassert when a packet is discarded due to address or maximum length 
                                          filtering or when the radio enters RXFIFO_OVERFLOW state. In TX the pin will de-assert if the 
                                          TX FIFO underflows. */ 
	SpiWriteReg(CC1101_PKTLEN,      0xFF);  //Packet Length
	SpiWriteReg(CC1101_PKTCTRL0,    0x00);  //Packet Automation Control // fixed packet length; length controlled in PKTLEN register
	SpiWriteReg(CC1101_PKTCTRL1,    0x00);
	SpiWriteReg(CC1101_CHANNR,      0x00);
	SpiWriteReg(CC1101_FSCTRL1,     0x06);  //Frequency Synthesizer Control
	SpiWriteReg(CC1101_FREQ2,       0x21);  //Frequency Control Word, High Byte
	SpiWriteReg(CC1101_FREQ1,       0x62);  //Frequency Control Word, Middle Byte
	SpiWriteReg(CC1101_FREQ0,       0x76);  //Frequency Control Word, Low Byte
	SpiWriteReg(CC1101_MDMCFG4,     0x8B);  // originally 0x8B -> 200kHz BW, 0x2B 500kHz BW;  Modem Configuration  // DR 80kBaud
	SpiWriteReg(CC1101_MDMCFG3,     0x93);  //Modem Configuration // DR 80kBaud
	SpiWriteReg(CC1101_MDMCFG2,     0x30);  //Modem Configuration // ASK / OOK nopreamble nosync , no crc appended
	SpiWriteReg(CC1101_MDMCFG1,     0x00);  //channel spacing 
	SpiWriteReg(CC1101_MDMCFG0,     0xff);  // channel spacing 
	SpiWriteReg(CC1101_MCSM1,       0x30);  // default settings 
	SpiWriteReg(CC1101_MCSM0,       0x29);  //Main Radio Control State Machine Configuration
	SpiWriteReg(CC1101_FOCCFG,      0x1D);  //Frequency Offset Compensation Configuration
	SpiWriteReg(CC1101_BSCFG,       0x1C);  //Bit Synchronization Configuration
	
    SpiWriteReg(CC1101_AGCCTRL2,    0x03);  //AGC Control // mainly used for RX mode
	SpiWriteReg(CC1101_AGCCTRL1,    0x00);  //AGC Control // mainly used for RX mode
	SpiWriteReg(CC1101_AGCCTRL0,    0x90);  //AGC Control // mainly used for RX mode
	
    SpiWriteReg(CC1101_FREND1,      0x56);  //Front End RX Configuration
	SpiWriteReg(CC1101_FREND0,      0x11);  // the last two bits define the index from the patable to use
	SpiWriteReg(CC1101_FSCAL3,      0xEA);  //Frequency Synthesizer Calibration
	SpiWriteReg(CC1101_FSCAL2,      0x2A);  //Frequency Synthesizer Calibration
	SpiWriteReg(CC1101_FSCAL1,      0x00);  //Frequency Synthesizer Calibration
	SpiWriteReg(CC1101_FSCAL0,      0x1F);  //Frequency Synthesizer Calibration
	SpiWriteReg(CC1101_TEST2,       0x81);  //Various Test Settings
	SpiWriteReg(CC1101_TEST0,       0x35);  //Various Test Settings
	SpiWriteReg(CC1101_TEST0,       0x09);  //Various Test Settings
	SpiWriteReg(CC1101_FIFOTHR,     0x00);  // rx threshold 32bytes
	
	SpiWriteReg(CC1101_MCSM0,0x28); 
    SpiWriteReg(CC1101_MCSM1,0x35); 
    SpiWriteReg(CC1101_FREND0,0x16);
    SpiWriteReg(CC1101_PKTCTRL0, 0x00);
    SpiWriteReg(CC1101_PKTLEN, 0x0c);
    SpiWriteReg(CC1101_IOCFG0, 0x06); 
    SpiWriteReg(CC1101_MDMCFG2, 0x32);
    /* syncronization word - preamble + Framesync */
    SpiWriteReg(CC1101_SYNC1, sync1);
    SpiWriteReg(CC1101_SYNC0, sync0);
    SpiWriteReg(CC1101_AGCCTRL2, agc2); 
    SpiWriteReg(CC1101_AGCCTRL1, 0x00);
    SpiWriteReg(CC1101_AGCCTRL0, agc0);
    SpiWriteReg(CC1101_FIFOTHR, 0x07);

	SpiStrobe(CC1101_SCAL);
}


/****************************************************************
*FUNCTION NAME: UpdateFifo
*FUNCTION     : Add Data To Fifo When TX is in infiniteMode
*INPUT        : data: pointer to data; nbytes: number of bytes to write to fifobuffer
*OUTPUT       : none
****************************************************************/
/* updated 12.02.2021 */
void CC1101A::UpdateFifo(byte *data, int nbytes)
{
  int index = 0;
  while (index < nbytes)
  {
    while(RX_GDO2_STATE);
    SpiWriteReg(CC1101_TXFIFO, data[index]);
    index++;
  }
}

/****************************************************************
* FUNCTION NAME: SendCW
* FUNCTION     : send continous high signal
* INPUT        : duration: number of bytes to send where 1 byte corresponds 8*12.5µs = 100µs
* OUTPUT       : none
*****************************************************************/
void CC1101A::SendCW(byte duration)
{
	for (int i = 0; i < duration; i++)
	{
		UpdateFifo(CW, 1);
	}
}

/****************************************************************
* FUNCTION NAME: SendCW
* FUNCTION     : send continous high signal
* INPUT        : duration: number of bytes to send where 1 byte corresponds 8*12.5µs = 100µs
*              : lastbyteduration: number of bits of last byte where 1 bit = 12.5µs (bigEndian notation)
* OUTPUT       : none
*****************************************************************/
/*
void CC1101A::SendCW(byte duration, byte lastbyteduration)
{
	byte lastbyte = 0x00;
    byte mask = 0x80;    
    if (lastbyteduration < 8)
    {
        for (int j = 0; j < lastbyteduration; j++)
        {
            lastbyte = lastbyte | mask;
            mask = mask >> 1;
        }
        
        for (int i = 0; i < duration-1; i++)
	    {
            UpdateFifo(CW, 1);
        }
        UpdateFifo(&lastbyte, 1);
    }
    else
    {
        SendCW(duration);
    }
}
*/
/****************************************************************
* FUNCTION NAME: SendCWBlock
* FUNCTION     : send continous high signal of duration n
* INPUT        : duration: number of bytes to send where 1 byte corresponds 8*12.5µs = 100µs
* OUTPUT       : none
*****************************************************************/
void CC1101A::SendCWBurst(byte duration)
{
	SpiWriteBurstReg(CC1101_TXFIFO, CW, duration);
}

/****************************************************************
* FUNCTION NAME: SendIdle
* FUNCTION     : send continous low signal
* INPUT        : duration: number of bytes to send where 1 byte corresponds 8*12.5µs = 100µs
 *OUTPUT       : none
****************************************************************/
void CC1101A::SendIdle(byte duration)
{
	for (int i = 0; i < duration; i++)
	{
		UpdateFifo(OFF, 1);
	}
}

/****************************************************************
* FUNCTION NAME: SendByte
* FUNCTION     : send single signal
* INPUT        : b: byte signal to send
 *OUTPUT       : none
****************************************************************/
/* use only if in TX mode 
 * definition for TX_GDO0/2_PIO and TX_GDO0/2_STATE needs to be added 
 * if used.
 */

void CC1101A::SendByte(byte value)
{
	while(RX_GDO2_STATE);
  SpiWriteReg(CC1101_TXFIFO, value);
}


/****************************************************************
* FUNCTION NAME: DecodeFM0
* FUNCTION     : FM0 decodes a sequence of bits
* INPUT        : data: pointer to a byte array of data; data_size: number of bytes that should be decoded from data
* OUTPUT       : bitArray: pointer to an array where the result is saved as sequence of bits
****************************************************************/
/*  ATTENTION: It does not check if state is properly changed according to FM0 for each data unit yet. It assumes the
               provided data is a legitimate format. The lack of this check is basically compensated by the usually 
               obligatory CRC check when communicating with a tag.
 */
void CC1101A::DecodeFM0(byte *bitArray, byte *data, byte data_size)
{
  byte bit_index = 0;
  byte mask = 0xC0; // 1100 0000
  byte FM0 = 0;
  byte prev = 0;
  byte curr = 0;
  for (byte i = 0; i < data_size; i++)
  {
    Serial.println(data[i],HEX);
    for (byte shift = 0; shift < 4; shift++)
    {
      FM0 = (data[i] & (mask >> (shift*2))) >> (6-(shift*2));
      switch(FM0){
        case 3:
          bitArray[(i*4)+shift] = 1;
          break;
        case 0:
          bitArray[(i*4)+shift] = 1;
          break;
        case 2:
          bitArray[(i*4)+shift] = 0;
          break;
        case 1:
          bitArray[(i*4)+shift] = 0;
          break;
        default:
          Serial.println("[ERROR] Error when decoding FM0 data!");
          break;
      }
    }
  }
}

CC1101A RX_UNIT;

#endif



