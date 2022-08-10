/*
* Global Vars
*/

#ifndef VARS_H
#define VARS_H

#define BUFFER_SIZE 255
#define THRESHOLD 32
#define FIFO 64
#define MAX_PKT 256
#define FM0_FACTOR 2
#define RX_PADDING 12 // taking rn16 , handle and header bit into account plus some extra space to avoid false signals
// RX_PADDING helps to prevent reading noise - If RX_PADDING is too small then the Receiver might read the rf signal incorrect for the last few bytes of the packet - not understood yet why that happens though.

/* ********** TAG SPECIFIC - PC MASKS *********** */
#define L_MASK    0xF800
#define T_MASK    0x0100
#define XI_MASK   0x0200
#define UMI_MASK  0x0400
/* ********************************************** */


/* Structure for storing tag information */
struct TAG_INFO{
  uint16_t 	StoredPC 		= 0;
  uint8_t	RN16[2] 		= {};
  uint8_t	EPC_Data[12] 	= {}; // EPC Length actually depends on StoredPC/PacketPC entry
  uint8_t	CRC16[2] 		= {};
  bool 		CRC_OK 			= false;
};
/* ********************************************** */


enum READER_STATES
{
  R_INIT,
  R_LOCK,
  R_WAIT,
  R_START,
  R_ACCESS,
  R_SELECT,
  R_QUERY,
  R_QUERYREP,
  R_REQRN,
  R_READ,
  R_WRITE,
  R_QUERYADJ,
  R_SEARCH_RN16,
  R_ACK,
  R_NAK,
  R_CW,
  R_POWERUP,
  R_POWERDOWN,
  R_READDATA,
  R_TEST,
  R_QUERY_BAK,  // old code as backup
  R_TEARS,
  R_TEARS2,       // experimental STATE
  R_TEARLOCK
};



// CONSTANTS (READER CONFIGURATION)

// Fixed number of slots (2^(FIXED_Q))  
const byte FIXED_Q              = 0;

// Query command (Q is set in code)
const byte QUERY_CODE[4] = {1,0,0,0};                       // QUERY command
const byte DR            = 0;                               // 0: TRcal divide ratio (8); 1: TRcal divide ratio (64/3)
const byte M[4][2]       = {{0,0},{0,1},{1,0},{1,1}};       // cycles per symbol (FM0 encoding = {0,0})
const byte TREXT         = 1;                               // 1: pilot tone; 0: no pilot tone
const byte SEL_ALL[2]    = {0,0};                            // which Tags respond to the Query: ALL TAGS {0,0} and {0,1}
const byte SEL_SL[4][2]  = {{0,0},{0,1},{1,0},{1,1}};       // which Tags respond to the Query: SELECTED TAGS
const byte SESSION[4][2] = {{0,0},{0,1},{1,0},{1,1}};       // session for the inventory round SL0,SL1,SL2,SL3 respectively
const byte TARGET        = 0;                               // inventoried flag is A or B

// valid values for Q
const byte Q_VALUE [16][4] =  
{
	{0,0,0,0}, {0,0,0,1}, {0,0,1,0}, {0,0,1,1}, 
	{0,1,0,0}, {0,1,0,1}, {0,1,1,0}, {0,1,1,1}, 
	{1,0,0,0}, {1,0,0,1}, {1,0,1,0}, {1,0,1,1},
	{1,1,0,0}, {1,1,0,1}, {1,1,1,0}, {1,1,1,1}
};  

const byte MEM_BANK [4][2] =
{
	{0,0}, // Reserved
	{0,1}, // EPC
	{1,0}, // TID
	{1,1} // USER
};


/*
* The settings below assume a pulsewidth of 12.5µs (cc1101 datarate at 80 kBaud)
* 1 TARI = 25µs <--> data0 duration = 25µs
*/
const byte DATA0[] = {1,0};
const byte DATA1[] = {1,1,1,0};
const byte DELIM[] = {0};
const byte RTCAL[] = {1,1,1,1,1,0}; // length of (data0 + data1)
const byte TRCAL[] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,0}; // max 3 * length(RTCAL) 16*12.5µs = 200µs --> BLF = 8/200µs = 40kHz (BLF = DR/RTCAL)

/*
* FRAMESYNC: delim + data0 + rtcal 
*/
const byte FRAMESYNC[] = {0,1,0,1,1,1,1,1,0}; // delim + data0 + rtcal
/*
/*
* ACK 
*/
const byte ACK[] = {0,1}; 

/*
 * REQ_RN
 */
const byte REQ_RN[] = {1,1,0,0,0,0,0,1};

/*
 * READ
 */
const byte READ_CMD[] = {1,1,0,0,0,0,1,0};

/*
 * WRITE
 */
const byte WRITE_CMD[] = {1,1,0,0,0,0,1,1};

/* 
 * BLOCKWRITE
 */
const byte BLOCKWRITE_CMD[] = {1,1,0,0,0,1,1,1};

/*
 * LOCK 11000101
 */
const byte LOCK_CMD[] = {1,1,0,0,0,1,0,1};

/*
 * ACCESS 11000110
 */
const byte ACCESS_CMD[] = {1,1,0,0,0,1,1,0};

/*
* QUERYREP: framesync + 4 x data0
*/
const byte QUERYREP[] = {0,1,0,1,1,1,1,1,0,1,0,1,0,1,0,1,0};

/*
* PREAMBLE: delim + data0 + rtcal + trcal
*/
const byte PREAMBLE[] = {0,1,0,1,1,1,1,1,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0};

/*
 * ERROR CODES - as per Table I-2: Tag error codes (pg 150) - https://www.gs1.org/sites/default/files/docs/epc/gs1-epc-gen2v2-uhf-airinterface_i21_r_2018-09-04.pdf
*/
const String ERRORCODE[] = {
            "Other error Catch - all for errors not covered by other codes.",
            "Not supported - The Tag does not support the specified parameters or feature.",
            "Insufficient privileges - The Interrogator did not authenticate itself with sufficient privileges for the Tag to perform the operation.",
            "Memory overrun - The Tag memory location does not exist, is too small, or the Tag does not support the specified EPC length.",
            "Memory locked - The Tag memory location is locked or permalocked and is either not writeable1 or not readable.",
            "Crypto suite error - Catch-all for errors specified by the cryptographic suite.",
            "Command not encapsulated - The Interrogator did not encapsulate the command in an AuthComm or SecureComm as required.",
            "ResponseBuffer overflow - The operation failed because the ResponseBuffer overflowed.",
            "Security timeout - The command failed because the Tag is in a security timeout.",
            "NA",
            "NA",
            "Insufficient power - The Tag has insufficient power to perform the operation.",
            "NA",
            "NA",
            "NA",
            "NA",
            "Non-specific error - The Tag does not support error-specific codes"
};

READER_STATES reader_state = R_START;

#endif
