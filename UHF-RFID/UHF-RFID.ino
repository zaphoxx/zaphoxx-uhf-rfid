/*  Name: UHF-RFID
 *  Description: UHF-RFID Reader based on two TI1101 RF modules and an Arduino Due
 *  Author: Manfred Heinz
 */

#include "./libs/CC1101A.cpp" // library for SPI controlled RF module 1
#include "./libs/CC1101B.cpp" // library for USART controlled RF module 2
#include "./global_vars/GLOBAL_VARS.h"
#include "UHF-RFID.h"

/* ********************* SHIELD SPECIFIC DEFINITIONS ********************** */
/* LED PINS */
#define LED_BLUE 53
#define LED_RED 51
#define LED_GREEN 49
/* fast digital write for LEDS */
#define LED_BLUE_ON (PIOB->PIO_SODR = (1 << 14))
#define LED_RED_ON  (PIOC->PIO_SODR = (1 << 12))
#define LED_BLUE_OFF (PIOB->PIO_CODR = (1 << 14))
#define LED_RED_OFF (PIOC->PIO_CODR = (1 << 12))
#define LED_GREEN_ON  (PIOC->PIO_SODR = (1 << 14))
#define LED_GREEN_OFF (PIOC->PIO_CODR = (1 << 14))
/* OTHERS */
/* ************************************************************************ */
#define AGC0 0x90
#define AGC2 0x40
/* **************************** Variables ********************************* */
EPC_DATA tag_data;
byte CWA[64];               // byte array for holding high signal values for continues mode
unsigned int counter = 0;   //
uint16_t lpass,hpass;       // variables for access action for holding lower and higher password values
byte data[512];             // data buffer
byte tx_version, rx_version;// version numbers of rf chip units

byte memoryblock;
byte nWords;
byte blockaddr;

/* array holding the RN16/HANDLE bits */
byte RN16_Bits[16];
byte HANDLE_Bits[16];

/* Flags for controlling actions*/
bool read_flag, write_flag, lock_flag, block_flag, access_flag, tears_flag, tearlock, reqrn_flag;
bool epc_found = false;

/* write specific variables */
byte write_delay = 64;
unsigned int repetitions = 500;
byte wordcount = 0;
word dataword = 0;
int T5 = 64;

/* access specific variables */
byte mask_bits[10], action_bits[10];

/* TEARS vars */
int curr_writes;
int num_writes;
int off_writes;
int bytes_to_send;
int bits_to_send;
byte last_byte;
int delay_microsec;
word tearword;
bool tears_rewrite = false;
bool next_write = false;


/* ************************************************************************ */


void setup()
{
  // delay to avoid known reset issue - see also https://forum.arduino.cc/index.php?topic=256771.75
  delay(1000);
  Serial.begin(115200);
  
  pinMode(LED_BLUE, OUTPUT);
  pinMode(LED_RED, OUTPUT);
  pinMode(LED_GREEN, OUTPUT);
  
  /* predefine CWA values */
  for (int i = 0; i < 64; i++) { CWA[i] = 0xff; }

  /* set initial reader state */
  reader_state = R_WAIT;
  counter = 0;

  read_flag = false;
  write_flag = false;
  block_flag = false;
  lock_flag = false;
  access_flag = false;
  tears_flag = false;
  
  lpass = 0;
  hpass = 0;
  
  for (int i = 0; i < sizeof(data); i++) data[i] = 0;
  reader_state = R_WAIT;
}

void loop() {
  switch(reader_state)
  {
    case R_INIT:
      epc_found = false;
      
      /* initialize RX and TX units */
      RX_UNIT.Init(); // Based on Arduino SPI.h and CC1101A.h lib 
      TX_UNIT.Init(); // Based on CC1101B.h lib
      tx_version = TX_UNIT.getVersion();
      rx_version = RX_UNIT.getVersion();
      Serial.print("[RX] version: "); Serial.println(rx_version,HEX);    
      Serial.print("[TX] version: "); Serial.println(tx_version,HEX);
      if ((tx_version == 0x14 or tx_version == 0x04) and (rx_version == 0x14 or rx_version == 0x04))
      {
        Serial.println("[+] Modules Check - OK");
        TX_UNIT.SpiStrobe(CC1101_SFSTXON);
        RX_UNIT.SpiStrobe(CC1101_SFSTXON);
        delay(100);
        debug("[START]");
        reader_state = R_START;  
      }
      else
      {
        debug("[-] Error on CC1101 module initialization - check version above to identify unit!");
        Serial.println("[-] Error on CC1101 module initialization");
        reader_state = R_WAIT;
      }
      delay(100);
      break;

    case R_START:
      LED_RED_OFF;
      LED_BLUE_OFF;

      wordcount = 0;
      /* Set modules into idle mode */
      /*this section is necessary otherwise the rx-module will not capture any more signals */
      /* Terminate TX Session - Go Into Standby */
      //debug("[R_START] terminate TX - idle rf modules");
      while ((TX_UNIT.SpiReadStatus(CC1101_TXBYTES) & 0x7f) > 0);
      TX_UNIT.SpiStrobe(CC1101_SIDLE);
      RX_UNIT.SpiStrobe(CC1101_SIDLE);
      delay(5);

      for (int i = 0; i < 16; i++) RN16_Bits[i] = 0;
      for (int i = 0; i < 16; i++) HANDLE_Bits[i] = 0;
      
      LED_GREEN_ON;
      /* ********** START TX AND SEND CW FOR TAG SETTLE ********** */
      TX_UNIT.SpiStrobe(CC1101_SFTX);
      RX_UNIT.SpiStrobe(CC1101_SFRX);
      delay(5);
      TX_UNIT.SpiStrobe(CC1101_STX);
      TX_UNIT.SpiWriteBurstReg(CC1101_TXFIFO, CWA, 64);
      /* ********** *********************************** ********** */
      {
        reader_state = R_QUERY;
      }
      break;

 /* ********************************************************************** */
/* ****************************** QUERY CMD ***************************** */
/* ********************************************************************** */
/* QUERY REQUEST
 * R_QUERY: 
 * 1) send QUERY request and read tag response (RN16)
 * 2) send ACK and read tags EPC response / if only EPC reading then this is 
 *    the last step otherwise see 3)
 * 3) send REQ_RN and transition tag into OPEN/SECURED state (depending on 
 *    access passwd setting)
 * 4) delegate to next command (READ, WRITE, LOCK, ACCESS etc.)
 */
 /* ********************************************************************** */
    case R_QUERY:
      if (counter < repetitions)
      {
        reader_state = R_START;
      }
      else
      {
        reader_state = R_POWERDOWN;
      }
      // 1) SEND QUERY REQUEST
      send_default_query();
      //    READ TAG RESPONSE / RN16
      if (read_RN16(RN16_Bits))
      {
        LED_BLUE_ON;
        // 2) SEND ACK
        send_ack(RN16_Bits);
        //    READ TAG RESPONSE / EPC DATA
        if (read_epc(&tag_data))
        {
          debug("[ACKNOWLEDGED]");
          epc_found = true;
          LED_RED_ON;
          
          if ( !read_flag && !write_flag && !lock_flag && !tears_flag && !tearlock && !reqrn_flag){
            counter = repetitions;
          }
          else
          {
            // 3) SEND REQ_RN
            send_req_rn(RN16_Bits);
            //    READ HANDLE
            if (read_Handle(HANDLE_Bits))
            {
              // TAG SHOULD NOW BE either in OPEN or SECURED STATE
              debug("[OPEN/SECURED]");
              /* add some CW to ensure tag stays powered up */
              
              TX_UNIT.SendCW(64); //32
              //debug(" > ACTION");  
              // 4) DELEGATE TO NEXT COMMAND
              if (access_flag)
              {
                debug("SWITCH TO ACCESS");
                reader_state = R_ACCESS;
                counter = 0;
              }
              else if (read_flag)
              {
                debug("[READ]");
                reader_state = R_READ;
                TX_UNIT.FillFifo();
              }
              else if(write_flag)
              {
                reader_state = R_WRITE;
                counter = 0;
              }
              else if(lock_flag)
              {
                reader_state = R_LOCK;
                counter = 0;
              }
              else if(tears_flag)
              {
                reader_state = R_TEARS;
                counter = 0;
              }
              else if(tearlock)
              {
                reader_state = R_TEARLOCK;
                debug("[R_TEARLOCK]");
                counter = 0;
              }
              else if(reqrn_flag)
              {
                reader_state = R_REQRN;
                debug("[R_REQRN]");
                reqrn_flag = false;
                counter = 0;
              }
              else
              {
                reader_state = R_POWERDOWN;
                counter = repetitions;
              }
            }
            else
            {
              counter++;
              LED_BLUE_OFF;
              LED_RED_OFF;
            }
          }
        }
        else 
        {
            counter++;
            LED_BLUE_OFF;
            LED_RED_OFF; 
        }
      }
      else
      {
        counter++;
        LED_BLUE_OFF;
        LED_RED_OFF;
      }
      TX_UNIT.SendCW(64);
      break;


/* ********************************************************************** */
/* ****************************** ACCESS CMD **************************** */
/* ********************************************************************** */
    case R_ACCESS:
      /* access sequence
       * 1) send req_rn / recv new RN16
       * 2) send access_cmd with [password (lower word) xor RN16] / check crc16 of response
       * 3) send req_rn / recv new RN16
       * 4) send access_cmd with upper word / check crc16 response
       * 5) if all good tag goes into secured state / if not goes back to arbitrate state (full query sequence needs to be repeated)
       */
      if (counter < repetitions)
      {
        reader_state = R_QUERY;
        // in case of errors restart full query sequence
      }
      else
      {
        debug("[ACCESS] FAILED!");
        reader_state = R_POWERDOWN;
      }

      /* initiate first access sequence with req_rn(handle) */
      send_req_rn(HANDLE_Bits);
      if(read_Handle(RN16_Bits))
      {
        send_access(lpass, HANDLE_Bits, RN16_Bits);
        if (search_access_ack())
        {
          debug("[ACCESS] OK");
          /* send second access sequence */
          send_req_rn(HANDLE_Bits);
          if(read_Handle(RN16_Bits))
          {
            send_access(hpass, HANDLE_Bits, RN16_Bits);
            if (search_access_ack())
              debug("[SECURED]");
              TX_UNIT.FillFifo();
              if (read_flag)
              {
                reader_state = R_READ;
                counter = 0;
              }
              else if (write_flag)
              {
                reader_state = R_WRITE;
                counter = 0;
              }
              else if (lock_flag)
              {
                reader_state = R_LOCK;
                counter = 0;
              }
              else if (tearlock)
              {
                reader_state = R_TEARLOCK;
                counter = 0;
              }
              else
              {
                reader_state = R_POWERDOWN;
                counter = repetitions;
              }
            }
            else
            {
              counter++;
              LED_BLUE_OFF;
              LED_RED_OFF;
              break;
            }
          }
          else
          {
            counter++;
            LED_BLUE_OFF;
            LED_RED_OFF;
            break;
          }
        }
        else
        {
          counter++;
          LED_BLUE_OFF;
          LED_RED_OFF;
          break;
        }     
      break;

  
/* ********************************************************************** */
/* ****************************** WRITE CMD ***************************** */
/* ********************************************************************** */
    case R_WRITE:
      /* ***** WRITE DATA TO TAG ***** */
      if (counter < repetitions)
      {
        reader_state = R_WRITE;
        debug("[Round]: "+String(counter));
      }
      else
      {
        if (wordcount >= nWords)
        {
          reader_state = R_READ;
        }
        else
        {
          reader_state = R_POWERDOWN;  
        }
        break;
      }
      debug("[WRITE] *");
      
      while ((wordcount < (nWords)) && (counter < repetitions))
      {
        debug("[REQ_RN] *");
        send_req_rn(HANDLE_Bits);
        if(read_Handle(RN16_Bits))
        {
          debug("[REQ_RN(Handle)]");
          TX_UNIT.FillFifo();
          dataword = data[(wordcount*2)+1] << 8 | data[(wordcount*2)];
          send_write(memoryblock, blockaddr+wordcount, dataword, HANDLE_Bits, RN16_Bits);
          debug("[SEARCH ACK]");
          if (search_write_ack())
          {
            TX_UNIT.FillFifo();
            wordcount++;
          }
        }
        counter++;
        LED_BLUE_OFF;
        LED_RED_OFF;
      }
      TX_UNIT.FillFifo();
      counter = repetitions;
      write_flag = false;
      break;

/* ********************************************************************** */
/* ****************************** READ CMD ****************************** */
/* ********************************************************************** */
    case R_READ:
      TX_UNIT.SendCW(64);
      debug("[READ2]");
      if (counter < repetitions)
      {
        reader_state = R_START;  // restart process - initially only jump back to R_READ
      }
      else
      {
        reader_state = R_POWERDOWN;
      }
      for (int i = 0; i < sizeof(data); i++) data[i]=0;
      /* **** READ DATA FROM TAG ***** */
      TX_UNIT.FillFifo();
      debug("[SEND READ]");
      send_read(memoryblock,blockaddr,nWords,HANDLE_Bits);
      debug("[RCV READ]");
      if (read_data(data, nWords)) //add 4 bytes for crc16 and handle + 1 byte because of the 0 header
      {
        TX_UNIT.FillFifo();
        Serial.println("#READDATA");
        Serial.write(data,nWords*2);
        reader_state = R_POWERDOWN;
        counter = repetitions;
      }
      else
      {
        counter++;
      }
      TX_UNIT.FillFifo();
      read_flag = false;
      break;

/* ********************************************************************** */
/* ****************************** LOCK CMD ****************************** */
/* ********************************************************************** */
    case R_LOCK:
      /* ******** LOCK CMD ******** */
      if (counter < repetitions)
      {
        reader_state = R_LOCK;
      }
      else
      {
        reader_state = R_POWERDOWN;
        break;
      }
      while (counter < repetitions)
      {
        send_lock(mask_bits, action_bits, HANDLE_Bits);
        if (search_lock_ack())
        {        
          counter = repetitions;
          break;
        }
        counter++;
        LED_BLUE_OFF;
        LED_RED_OFF;
        TX_UNIT.FillFifo();
        break;
      }
      lock_flag = false;
      break;

   


/* ********************************************************************** */
/* ******************************* TEARING ****************************** */
/* ********************************************************************** */
   case R_TEARS:
      reader_state = R_INIT;
      tears_flag = true;
      bits_to_send = (curr_writes + off_writes) % 8;
      last_byte = 0xff << ( 8 - bits_to_send );
      bytes_to_send = ( curr_writes + off_writes ) / 8;

      if (next_write) // DO A TEARED WRITE
      {
        next_write = false;
        debug("[WRITE ++] normal write");
        send_req_rn(HANDLE_Bits);
        if(read_Handle(RN16_Bits))
        {
          debug("[REQ_RN(Handle)]");
          TX_UNIT.FillFifo();
          dataword = 0xdead;
          send_write(memoryblock, blockaddr, dataword, HANDLE_Bits, RN16_Bits);
          debug("[TEAR SIGNAL]");
          /* SKIP SEARCH ACK */
          /* SEND CW BYTES */
          TX_UNIT.SendCW(bytes_to_send);
          /* SEND CW FINAL BYTE */
          TX_UNIT.SendByte(last_byte);
          /* SEND IDLE SIGNAL TO TERMINATE TAG POWER */
          TX_UNIT.SendIdle(64);
          LED_RED_ON;LED_BLUE_ON;
        }        
      }
      else // READ DATA AFTER TEARED WRITE
      {
        send_read(memoryblock,blockaddr,nWords,HANDLE_Bits);      
        if (read_data(data, nWords))
        {
          TX_UNIT.FillFifo();
          Serial.println("#PRETEAR");
          Serial.write(bits_to_send + (bytes_to_send * 8));
          Serial.write(data,nWords*2);
          debug("[READ ++]",data,nWords*2);
          reader_state = R_INIT;
          debug("---");
          curr_writes++;
          next_write = true;
        }
        else
        {
          // dummy data value 0xbaad
          TX_UNIT.FillFifo();
          data[0] = 0xad; data[1] = 0xba;
          Serial.println("#PRETEAR");
          Serial.write(bits_to_send + (bytes_to_send * 8));
          Serial.write(data,nWords*2);
          debug("[READ ++]",data,nWords*2);
          reader_state = R_INIT;
          debug("---");
          curr_writes++;
          next_write = true;          
        }
      }
      debug("curr_writes : ",curr_writes);
      debug("num_writes  : ",num_writes);
      if (curr_writes >= num_writes)
      {
        reader_state = R_POWERDOWN;
      }
      
      break;
 

/* ********************************************************************** */
/* ****************************** POWERDOWN ***************************** */
/* ********************************************************************** */
    case R_POWERDOWN:
      if (epc_found){
        Serial.println("#TAGDATA");
        Serial.write(tag_data.stored_pc,2);
        Serial.write(tag_data.epc,12);
        Serial.write(tag_data.crc16,2);
      }
      /* GRACEFULLY TERMINATE TX UNIT */
      /* wait till fifo is empty before going into standby */
      //while ((TX_UNIT.SpiReadStatus(CC1101_TXBYTES) & 0x7f) > 0);
      /* Terminate TX Session - Go Into Standby */
      //debug("[POWERDOWN] set rf IDLE");
      TX_UNIT.SpiStrobe(CC1101_SIDLE);
      RX_UNIT.SpiStrobe(CC1101_SIDLE);
      reader_state = R_WAIT;
      LED_BLUE_OFF;
      LED_RED_ON;
      LED_GREEN_OFF;

      Serial.println("#END");  
      break;



/* ********************************************************************** */
/* ***************************** Serial Loop **************************** */
/* ********************************************************************** */
    case R_WAIT:
      digitalWrite(LED_RED, HIGH);
      reader_state = processSerial();
      break;
      
    default:
      reader_state = R_WAIT;
      break; 
  }
}

/* ********************************************************************** */
/* **************************** Process Serial ************************** */
/* ********************************************************************** */
/* ************* processing of incoming serial commands ************* */
READER_STATES processSerial()
{
  String cmd_string;
  if (Serial.available()) 
  {
    cmd_string = Serial.readStringUntil('#');
    if (cmd_string.equals("RUN"))
    {
      debug("[+] running ... ");
      reader_state = R_INIT;
    }
    else if(cmd_string.equals("T5"))
    {
      cmd_string = Serial.readStringUntil('#');
      T5 = cmd_string.toInt();
    }
    else if (cmd_string.equals("REP"))
    {
      cmd_string = Serial.readStringUntil('#');
      repetitions = cmd_string.toInt();
    }
    else if(cmd_string.equals("READ"))
    {
      read_flag = true;
      write_flag = false;
      lock_flag = false;
      block_flag = false;
      access_flag = false;
      cmd_string = Serial.readStringUntil('#');
      memoryblock = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      blockaddr = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      nWords = (byte) cmd_string.toInt();
      if (( nWords * 4 ) > 248 )
      {
        error("[!] number of words too large! Please choose a smaller amount of data to read 1");
        read_flag = false;
        reader_state = R_WAIT;
        Serial.write("#END");
      }
      debug("[+] Read Mode");      
    }
    else if (cmd_string.equals("WRITE"))
    {
      write_flag = true;
  
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      memoryblock = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      blockaddr = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      nWords = (byte) cmd_string.toInt();
      for (int i = 0; i < sizeof(data); i++) data[i] = 0;
      Serial.readBytes(data,nWords * 2);
      //dataword = data[1] << 8 | data[0];
      debug("[+] WRITE MODE");
    }
    /* Section for processing of access sequence */
    else if (cmd_string.equals("READACCESS"))
    {
      access_flag = true;
      read_flag = true;
      /* Read in passwd as lower and upper word */
      byte tmp[2];
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      lpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      hpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
  
      /* read in standart parameters */
      cmd_string = Serial.readStringUntil('#');
      memoryblock = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      blockaddr = (byte) cmd_string.toInt();
      //debug(String(blockaddr));
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      nWords = (byte) cmd_string.toInt();
      //debug(String(nWords));
      debug("[READACCESS]");
    }
    else if (cmd_string.equals("WRITEACCESS"))
    {
      access_flag = true;
      write_flag = true;
      
      byte tmp[2];
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      lpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      hpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
      
      cmd_string = Serial.readStringUntil('#');
      memoryblock = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      blockaddr = (byte) cmd_string.toInt();
      cmd_string = Serial.readStringUntil('#');
      //Serial.println(cmd_string);
      nWords = (byte) cmd_string.toInt();
      for (int i = 0; i < sizeof(data); i++) data[i] = 0;
      Serial.readBytes(data,nWords * 2);
    }
    else if (cmd_string.equals("LOCK"))
    {
      lock_flag = true;
      
      // Read in the 20 mask/action bits
      Serial.readBytes(mask_bits,10);
      Serial.readBytes(action_bits,10);
      
      debug("[+] LOCK MODE");
    }
    else if (cmd_string.equals("LOCKACCESS"))
    {
      access_flag = true;
      lock_flag = true;
  
      byte tmp[2];
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      lpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
      Serial.readBytes(tmp,2);
      debug(tmp,2);
      hpass = (tmp[1] << 8) | tmp[0];
      cmd_string = Serial.readStringUntil('#');
      
      // Read in the 20 mask/action bits
      Serial.readBytes(mask_bits,10);
      Serial.readBytes(action_bits,10);
      
      debug("[LOCKACCESS]");
    }
    else if (cmd_string.equals("TEARS"))
    {
      tears_flag = true;
      read_flag = false;
      write_flag = false;
      lock_flag = false;
      block_flag = false;
      access_flag = false;
      
      cmd_string = Serial.readStringUntil('#');
      memoryblock = (byte) cmd_string.toInt();
      
      cmd_string = Serial.readStringUntil('#');
      blockaddr = (byte) cmd_string.toInt();
      
      cmd_string = Serial.readStringUntil('#');
      nWords = (byte) cmd_string.toInt();
      
      for (int i = 0; i < sizeof(data); i++) data[i] = 0;
      Serial.readBytes(data,nWords * 2);
      tearword = data[1] << 8 | data[0];
      
      cmd_string = Serial.readStringUntil('#');
      
      
      cmd_string = Serial.readStringUntil('#');
      off_writes = cmd_string.toInt();
      
      cmd_string = Serial.readStringUntil('#');
      num_writes = cmd_string.toInt();
  
      cmd_string = Serial.readStringUntil('#');
      tears_rewrite = cmd_string.toInt();
      
      curr_writes = 0;
      next_write = true;
      debug("[TEARS]");
    }
    else
    {
      reader_state = R_WAIT;
      delay(100);
    }
  }
  else
  { 
    reader_state = R_WAIT;
    delay(100);
  }
  return reader_state;
}

/*
      else if (cmd_string.equals("REP"))
      {
        cmd_string = Serial.readStringUntil('#');
        repetitions = cmd_string.toInt();
      }
      else if (cmd_string.equals("TXP"))
      {
        cmd_string = Serial.readStringUntil('#');
        tx_power = (byte) cmd_string.toInt();
      }
      else if (cmd_string.equals("T5"))
      {
        cmd_string = Serial.readStringUntil('#');
        T5 = cmd_string.toInt();
      }
      // Reading data from memory blocks
      else if (cmd_string.equals("READ"))
      {
        //Serial.println("#READ#");
        read_flag = true;
        write_flag = false;
        lock_flag = false;
        block_flag = false;
        access_flag = false;
        cmd_string = Serial.readStringUntil('#');
        memoryblock = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        blockaddr = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        nWords = (byte) cmd_string.toInt();
        debug("[+] Read Mode");
      }
      // Writing data to tag
      else if (cmd_string.equals("WRITE"))
      {
        write_flag = true;
        block_flag = false;
        monza_flag =false;
        lock_flag = false;
        access_flag = false;
        
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        memoryblock = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        blockaddr = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        nWords = (byte) cmd_string.toInt();
        for (int i = 0; i < sizeof(data); i++) data[i] = 0;
        Serial.readBytes(data,nWords * 2);
        //dataword = data[1] << 8 | data[0];
        debug("[+] WRITE MODE");
      }
      
      else if (cmd_string.equals("READACCESS"))
      {
        access_flag = true;
        read_flag = true;
        write_flag = false;
        lock_flag = false;
        
        byte tmp[2];
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        lpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        hpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');

        
        cmd_string = Serial.readStringUntil('#');
        memoryblock = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        blockaddr = (byte) cmd_string.toInt();
        //debug(String(blockaddr));
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        nWords = (byte) cmd_string.toInt();
        //debug(String(nWords));
        debug("[READACCESS]");
      }
      else if (cmd_string.equals("WRITEACCESS"))
      {
        access_flag = true;
        read_flag = false;
        write_flag = true;
        lock_flag = false;
        byte tmp[2];
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        lpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        hpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');
        
        cmd_string = Serial.readStringUntil('#');
        memoryblock = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        blockaddr = (byte) cmd_string.toInt();
        cmd_string = Serial.readStringUntil('#');
        //Serial.println(cmd_string);
        nWords = (byte) cmd_string.toInt();
        for (int i = 0; i < sizeof(data); i++) data[i] = 0;
        Serial.readBytes(data,nWords * 2);
      }
      
      // Section if only basic EPC read should be performed
      else if (cmd_string.equals("EPC"))
      {
        Serial.println("#READ_EPC");
        write_flag = false;
        read_flag = false;
        block_flag = false;
        access_flag = false;
        debug("[+] Read EPC");
      }
      else if (cmd_string.equals("LOCK"))
      {
        write_flag = false;
        read_flag = false;
        lock_flag = true;
        access_flag = false;
        
        
        // Read in the 20 mask/action bits
        Serial.readBytes(mask_bits,10);
        Serial.readBytes(action_bits,10);
        
        debug("[+] LOCK MODE");
      }
      else if (cmd_string.equals("LOCKACCESS"))
      {
        access_flag = true;
        write_flag = false;
        read_flag = false;
        lock_flag = true;

        byte tmp[2];
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        lpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');
        Serial.readBytes(tmp,2);
        debug(tmp,2);
        hpass = (tmp[1] << 8) | tmp[0];
        cmd_string = Serial.readStringUntil('#');
        
        // Read in the 20 mask/action bits
        Serial.readBytes(mask_bits,10);
        Serial.readBytes(action_bits,10);
        
        debug("[LOCKACCESS]");
      }
      else if (cmd_string.equals("TEARS"))
      {
        tears_flag = true;
        
        cmd_string = Serial.readStringUntil('#');
        memoryblock = (byte) cmd_string.toInt();
        
        cmd_string = Serial.readStringUntil('#');
        blockaddr = (byte) cmd_string.toInt();
        
        cmd_string = Serial.readStringUntil('#');
        nWords = (byte) cmd_string.toInt();
        
        for (int i = 0; i < sizeof(data); i++) data[i] = 0;
        Serial.readBytes(data,nWords * 2);
        tearword = data[1] << 8 | data[0];
        
        cmd_string = Serial.readStringUntil('#');
        
        
        cmd_string = Serial.readStringUntil('#');
        off_writes = cmd_string.toInt();
        
        cmd_string = Serial.readStringUntil('#');
        num_writes = cmd_string.toInt();

        cmd_string = Serial.readStringUntil('#');
        tears_rewrite = cmd_string.toInt();
        
        curr_writes = 0;
        
        debug("[TEARS]");
        
        
 
      }
      else if (cmd_string.equals("TEARLOCK"))
      {
        tearlock_flag = true;

        cmd_string = Serial.readStringUntil('#');
        off_writes = cmd_string.toInt();
        //debug(String(off_writes));
        
        cmd_string = Serial.readStringUntil('#');
        num_writes = cmd_string.toInt();
        //debug(String(num_writes));

        Serial.readBytes(mask_bits,10);
        Serial.readBytes(action_bits,10);

        debug(mask_bits,10);
        debug("[+] TEARLOCK MODE");
      }
      else if (cmd_string.equals("REQRN"))
      {
        reqrn_flag = true;
        debug("[+] REQRN MODE");
      }
      else
      { 
        reader_state = R_WAIT;
        //flushSerial();
        delay(100);
      }
      cmd_string = "";
      break;
      
    default:
      reader_state = R_WAIT;
      break;
    }
}
*/
