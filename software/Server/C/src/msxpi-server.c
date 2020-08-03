/*
 ;|===========================================================================|
 ;|                                                                           |
 ;| MSXPi Interface                                                           |
 ;|                                                                           |
 ;| Version : 1.0                                                             |
 ;|                                                                           |
 ;| Copyright (c) 2015-2020 Ronivon Candido Costa (ronivon@outlook.com)       |
 ;|                                                                           |
 ;| All rights reserved                                                       |
 ;|                                                                           |
 ;| Redistribution and use in source and compiled forms, with or without      |
 ;| modification, are permitted under GPL license.                            |
 ;|                                                                           |
 ;|===========================================================================|
 ;|                                                                           |
 ;| This file is part of MSXPi Interface project.                             |
 ;|                                                                           |
 ;| MSX PI Interface is free software: you can redistribute it and/or modify  |
 ;| it under the terms of the GNU General Public License as published by      |
 ;| the Free Software Foundation, either version 3 of the License, or         |
 ;| (at your option) any later version.                                       |
 ;|                                                                           |
 ;| MSX PI Interface is distributed in the hope that it will be useful,       |
 ;| but WITHOUT ANY WARRANTY; without even the implied warranty of            |
 ;| MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             |
 ;| GNU General Public License for more details.                              |
 ;|                                                                           |
 ;| You should have received a copy of the GNU General Public License         |
 ;| along with MSX PI Interface.  If not, see <http://www.gnu.org/licenses/>. |
 ;|===========================================================================|
 ;
 ; File history :
 ; 1.0    : Support to /wait signal on MSX bus
 ; 0.8.1  : MSX-DOS working properly.
 ; 0.8    : Rewritten with new protocol-v2
 ;          New functions, new main loop, new framework for better reuse
 ;          This version now includes MSX-DOS 1.03 driver
 ; 0.7    : Commands CD and MORE working for http, ftp, nfs, win, local files.
 ; 0.6d   : Added http suport to LOAD and FILES commands
 ; 0.6c   : Initial version commited to git
 ;
 
 PI pinout x GPIO:
 http://abyz.co.uk/rpi/pigpio/index.html
 
 
 Library required by this program and how to install:
 http://abyz.co.uk/rpi/pigpio/download.html
 
 Steps:
 sudo apt-get install libcurl4-nss-dev
 wget abyz.co.uk/rpi/pigpio/pigpio.tar
 tar xf pigpio.tar
 cd PIGPIO
 make -j4
 sudo make install
 
 To compile and run this program:
 cc -Wall -pthread -o msxpi-server msxpi-server.c -lpigpio -lrt -lcurl
 
 */

#include <stdio.h>
#include <pigpio.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>
#include <sys/stat.h>
#include <curl/curl.h>
#include <assert.h>
#include <time.h>
#include <unistd.h>
#include <stdbool.h>
#include <sys/mman.h>
#include <fcntl.h>

#define TZ (0)
#define version "1.0"
#define build "20200803.00000"

//#define V07SUPPORT
#define DISKIMGPATH "/home/pi/msxpi/disks"
#define HOMEPATH "/home/pi/msxpi"

/* GPIO pin numbers used in this program */

#define cs    21
#define sclk  20
#define mosi  16
#define miso  12
#define rdy   25

#define SPI_SCLK_LOW_TIME 0
#define SPI_SCLK_HIGH_TIME 0
#define HIGH 1
#define LOW 0
#define command 1
#define binary  2

#define GLOBALRETRIES      5

#define SPI_INT_TIME            3000
#define PIWAITTIMEOUTOTHER      120      // seconds
#define PIWAITTIMEOUTBIOS       60      // seconds
#define SYNCTIMEOUT             5
#define BYTETRANSFTIMEOUT       5
#define SYNCTRANSFTIMEOUT       3

#define RC_SUCCESS              0xE0
#define RC_INVALIDCOMMAND       0xE1
#define RC_CRCERROR             0xE2
#define RC_TIMEOUT              0xE3
#define RC_INVALIDDATASIZE      0xE4
#define RC_OUTOFSYNC            0xE5
#define RC_FILENOTFOUND         0xE6
#define RC_FAILED               0xE7
#define RC_INFORESPONSE         0xE8
#define RC_WAIT                 0xE9
#define RC_READY                0xEA
#define RC_SUCCNOSTD            0XEB
#define RC_FAILNOSTD            0XEC
#define RC_UNDEFINED            0xEF

#define st_init                 0       // waiting loop, waiting for a command
#define st_cmd                  1       // transfering data for a command
#define st_recvdata             2
#define st_senddata             4
#define st_synch                5       // running a command received from MSX
#define st_runcmd               6
#define st_shutdown             99

// commands
#define CMDREAD         0x00
#define LOADROM         0x01
#define LOADCLIENT      0x02

// from 0x03 to 0xF reserver
// 0xAA - 0xAF : Control code
#define STARTTRANSFER   0xA0
#define SENDNEXT        0xA1
#define ENDTRANSFER     0xA2
#define READY           0xAA
#define ABORT           0xAD
#define WAIT            0xAE

#define WIFICFG         0x1A
#define CMDDIR          0x1D
#define CMDPIFSM        0x33
#define CMDGETSTAT      0x55
#define SHUTDOWN        0x66
#define DATATRANSF      0x77
#define CMDSETPARM      0x7A
#define CMDSETPATH      0x7B
#define CMDPWD          0x7C
#define CMDMORE         0x7D
#define CMDPATHERR1     0x7E
#define UNKERR          0x98
#define FNOTFOUND       0x99
#define PI_READY        0xAA
#define NOT_READY       0xAF
#define RUNPICMD        0xCC
#define CMDSETVAR       0xD1
#define NXTDEV_INFO     0xE0
#define NXTDEV_STATUS   0xE1
#define NXTDEV_RSECT    0xE2
#define NXTDEV_WSECT    0xE3
#define NXTLUN_INFO     0xE4
#define CMDERROR        0xEE
#define FNOTFOUD        0xEF
#define CMDLDFILE       0xF1
#define CMDSVFILE       0xF5
#define CMDRESET        0xFF
#define RAW     0
#define LDR     1
#define CLT     2
#define BIN     3
#define ROM     4
#define FSLOCAL     1
#define FSUSB1      2
#define FSUSB2      3
#define FSNFS       4
#define FSWIN       5
#define FSHTTP      6
#define FSHTTPS     7
#define FSFTP       8
#define FSFTPS      9

// MSX-DOS2 Error Codes
#define __NOFIL     0xD7
#define __DISK      0xFD
#define __SUCCESS   0x00

struct transferStruct {
    unsigned char rc;
    int  datasize;
    unsigned char *data;
};

typedef struct {
    unsigned char deviceNumber;
    unsigned char mediaDescriptor;
    unsigned char logicUnitNumber;
    unsigned char sectors;
    int           initialSector;
} DOS_SectorStruct;

struct DiskImgInfo {
    int rc;
    char dskname[65];
    unsigned char *data;
    unsigned char deviceNumber;
    double size;
};

struct psettype {
    char var[16];
    char value[254];
};

struct curlMemStruct {
    unsigned char *memory;
    size_t size;
};

typedef struct curlMemStruct MemoryStruct;

unsigned char appstate = st_init;
unsigned char msxbyte;
unsigned char msxbyterdy;
unsigned char pibyte;
int debug;
bool CHECKTIMEOUT = false;
bool PIEXCHANGETIMEDOUT = false;

//Tools for waiting for a new command
pthread_mutex_t newComMutex = PTHREAD_MUTEX_INITIALIZER;
pthread_cond_t newComCond  = PTHREAD_COND_INITIALIZER;

// Create structure for data transfers and allocate the required memory for the data
struct transferStruct *newtransferStruct (size_t sz) {
    // Try to allocate vector structure.

    struct transferStruct *dataInfo = malloc (sizeof (struct transferStruct));
    if (dataInfo == NULL)
        return NULL;

    // Try to allocate vector data, free structure if fail.

    dataInfo->data = malloc (sz * sizeof (unsigned char));
    if (dataInfo->data == NULL) {
        free (dataInfo);
        return NULL;
    }

    // Set size and return.

    dataInfo->datasize = sz;
    return dataInfo;
}

void delay(unsigned int secs) {
    unsigned int retTime = time(0) + secs;   // Get finishing time.
    while (time(0) < retTime);               // Loop until it arrives.
}

char *replace(char *s,unsigned char c, unsigned char n) {
    int i;
    for(i=0;i<strlen(s);i++)
        if(s[i]==c)
            s[i] = n;
    
    return s;
}

char** str_split(char* a_str, const char a_delim) {
    char** result    = 0;
    size_t count     = 0;
    char* tmp        = a_str;
    char* last_comma = 0;
    char delim[2];
    delim[0] = a_delim;
    delim[1] = 0;
    
    /* Count how many elements will be extracted. */
    while (*tmp)
    {
        if (a_delim == *tmp)
        {
            count++;
            last_comma = tmp;
        }
        tmp++;
    }
    
    // fix for bug when delimiter is "/"
    if (a_delim==0x2f)
        count--;
    
    /* Add space for trailing token. */
    count += last_comma < (a_str + strlen(a_str) - 1);
    
    /* Add space for terminating null string so caller
     knows where the list of returned strings ends. */
    count++;
    
    result = malloc(sizeof(*result) * count);
    
    if (result)
    {
        size_t idx  = 0;
        char* token = strtok(a_str, delim);
        
        while (token)
        {
            assert(idx < count);
            *(result + idx++) = strdup(token);
            token = strtok(0, delim);
            //printf("token,idx,count = %s,%i,%i\n",token,idx,count);
        }
        
        //assert(idx == count - 1);
        *(result + idx) = 0;
    }
    
    return result;
}

char *strdup (const char *s) {
    char *d;
    d = malloc(255*sizeof(*d));          // Space for length plus nul
    if (d == NULL) return NULL;          // No memory
    strcpy (d,s);                        // Copy the characters
    return d;                            // Return the new string
}

int isDirectory(const char *path) {
    struct stat statbuf;
    if (stat(path, &statbuf) != 0)
        return 0;
    return S_ISDIR(statbuf.st_mode);
}

void init_spi_bitbang(void) {
    gpioSetMode(cs, PI_INPUT);
    gpioSetMode(sclk, PI_OUTPUT);
    gpioSetMode(mosi, PI_INPUT);
    gpioSetMode(miso, PI_OUTPUT);
    gpioSetMode(rdy, PI_OUTPUT);
    
    gpioSetPullUpDown(cs, PI_PUD_UP);
    gpioSetPullUpDown(mosi, PI_PUD_UP);
    
}

void write_MISO(unsigned char bit) {
    gpioWrite(miso, bit);
}

void tick_sclk(void) {
    gpioWrite(sclk,HIGH);
    gpioDelay(SPI_SCLK_HIGH_TIME);
    gpioWrite(sclk,LOW);
    gpioDelay(SPI_SCLK_LOW_TIME);
}

// This is where the SPI protocol is implemented.
// This function will transfer a byte (send and receive) to the MSX Interface.
// It receives a byte as input, return a byte as output.
// It is full-duplex, sends a bit, read a bit in each of the 8 cycles in the loop.
// It is tightely linked to the register-shift implementation in the CPLD,
// If something changes there, it must have changes here so the protocol will match.

uint8_t receive_byte() {
    uint8_t byte_in;
    uint8_t bit;
    unsigned rdbit;

    gpioWrite(miso,HIGH);
    while(gpioRead(cs)) 
        {}

    tick_sclk();

    for (bit = 0x80; bit; bit >>= 1) {
        gpioWrite(sclk,HIGH);
        gpioDelay(SPI_SCLK_HIGH_TIME);
        
        rdbit = gpioRead(mosi);
        if (rdbit == HIGH)
            byte_in |= bit;
        
        gpioWrite(sclk,LOW);
        gpioDelay(SPI_SCLK_LOW_TIME);
    }

    // tick rdyPin once to flag to MSXPi that transfer is completed
    gpioWrite(rdy,LOW);
    gpioDelay(SPI_SCLK_LOW_TIME);
    gpioWrite(rdy,HIGH);
    gpioWrite(miso,LOW);

    printf("%x\n",byte_in);

    return byte_in;
}

uint8_t send_byte(uint8_t byte_out) {
    uint8_t bit;

    gpioWrite(miso,HIGH);
    while(gpioRead(cs)) 
        {}

    tick_sclk();

    for (bit = 0x80; bit; bit >>= 1) {
        write_MISO((byte_out & bit) ? HIGH : LOW);
        gpioWrite(sclk,HIGH);
        gpioDelay(SPI_SCLK_HIGH_TIME);
        gpioWrite(sclk,LOW);
        gpioDelay(SPI_SCLK_LOW_TIME);
    }
    
    // tick rdyPin once to flag to MSXPi that transfer is completed
    gpioWrite(rdy,LOW);
    gpioDelay(SPI_SCLK_LOW_TIME);
    gpioWrite(rdy,HIGH);
    gpioWrite(miso,LOW);

    return byte_out;
}

uint8_t piexchangebyte(uint8_t byte_out) {
    uint8_t byte_in = receive_byte();
    send_byte(byte_out);
    return byte_in;
}

/* senddatablock
 ---------------
 21/03/2017
 
 Send a block of data to MSX. Read the data from a pointer passed to the function.
 Do not retry if it fails (this should be implemented somewhere else).
 Will inform the block size to MSX (two bytes) so it knows the size of transfer.
 
 Logic sequence is:
 1. read MSX status (expect SENDNEXT)
 2. send lsb for block size
 3. send msb for block size
 4. read (lsb+256*msb) bytes from buffer and send to MSX
 5. exchange crc with msx
 6. end function and return status
 
 Return code will contain the result of the oepration.
 */
uint8_t senddatablock(struct transferStruct *dataInfo) {
    
    uint8_t byte_in,byte_out;
    uint8_t crc = 0;
    int bytecounter = 0;
    
    int datasize = dataInfo->datasize;

    //printf("senddatablock: starting\n");
    // send block size if requested by caller.
    send_byte(datasize % 256); 
    send_byte(datasize / 256);

    //printf("senddatablock:blocksize = %i\n",datasize);
        
    while(datasize>0) {         
        byte_out = *(dataInfo->data + bytecounter);
        send_byte(byte_out);
        crc ^= byte_out;
        bytecounter++;
        datasize--;
    }            
            
    byte_in = receive_byte(crc);
            
    //printf("senddatablock:Received MSX CRC: %x\n",mymsxbyte);
    if (byte_in == crc) {
        printf("mymsxbyte:CRC verified\n");
        dataInfo->rc = RC_SUCCESS;
    } else {
        dataInfo->rc = RC_CRCERROR;
        printf("senddatablock:CRC ERROR CRC: %x different than MSX CRC: %x\n",crc,dataInfo->rc);
    }
    
    send_byte(dataInfo->rc);

    //printf("senddatablock:exiting with rc = %x\n",dataInfo.rc);
    return dataInfo->rc;
}

/* recvdatablock
 ---------------
 Read a block of data from MSX and stores in the pointer passed to the function.
 Do not retry if it fails (this should be implemented somewhere else).
 Will read the block size from MSX (two bytes) to know size of transfer.
 
 Logic sequence is:
 1. read MSX status (expect SENDNEXT)
 2. read lsb for block size
 3. read msb for block size
 4. read (lsb+256*msb) bytes from MSX and store in buffer
 5. exchange crc with msx
 6. end function and return status
 
 Return code will contain the result of the oepration.
 */

struct transferStruct *recvdatablock() {

    int bytecounter,blocksize,lsb,msb = 0;
    uint8_t byte_in;
    uint8_t crc = 0;
    
    printf("recvdatablock:starting\n");
    // read block size
    lsb = receive_byte();
    msb = receive_byte();

    blocksize = lsb + 256 * msb;   
    
    printf("recvdatablock:blocksize = %i\n",blocksize);
    
    struct transferStruct *dataInfo = newtransferStruct(blocksize);

    printf("recvdatablock:blocksize = %i\n",blocksize);
    
    //while(dataInfo.datasize>bytecounter && byte_in>=0) {
    while(blocksize>0) {       
        byte_in = receive_byte();
        *(dataInfo->data + bytecounter) = byte_in;
        crc ^= byte_in;
        bytecounter++;
        blocksize--;
    }
    
    printf("senddatablock: received %s\n",dataInfo->data);
    byte_in = receive_byte();
        
    //printf("recvdatablock:Received MSX CRC: %x\n",byte_in);
    if (byte_in == crc) {
        printf("recvdatablock:CRC verified\n");
        dataInfo->rc = RC_SUCCESS;
    } else {
        dataInfo->rc = RC_CRCERROR;
        printf("recvdatablock:CRC ERROR CRC: %x different than MSX CRC: %x\n",crc,dataInfo->rc);
    }
    
    send_byte(dataInfo->rc);
    printf("recvdatablock:exiting with rc = %x\n",dataInfo->rc);
    return dataInfo;
}

uint8_t ptest(struct transferStruct *dataInfo) {
    printf("ptest:received parameters:%s\n",dataInfo->data);

    send_byte(RC_SUCCESS);

    int n,m,i,dsL,dsM,errors = 0;
    int countto = 8192;

    //if (args[0] == "\0" || atoi(args[0]) == 0) 
    //    countto = 255;
    //else
    //    countto = atoi(parms[0]);

    printf("Testing data send for %i bytes\n",countto);

    send_byte(countto % 256);
    send_byte(countto / 256);

    for (i=0;i < countto; i++) {
        dsL = receive_byte();
        dsM = receive_byte();
        m = dsL + 256 * dsM;

        if (m != n)
            errors += 1;

        n++;
        printf("%i",m);
    }

    printf("Receiving errors:%i\n",errors);
    send_byte(errors % 256);
    send_byte(errors / 256);

    printf("Testing data recv for %i bytes\n",countto);

    for (i=0;i < countto; i++) {
        printf("%i",i);
        send_byte(i % 256);
        send_byte(i / 256);
    }

    return RC_SUCCESS;
}


int main(int argc, char *argv[]){
    
    struct transferStruct *dataInfo;
    
    // bug in code is oversriting ENV vars below.
    // as a temp workaround I reserved some space here to allow dir entries
    //char dummy[2500];
    
    // there is a memory corruption / leak somewhere in the code
    // if these two variables are moved to other places, and dummy is removed,
    // some commands will crash. PCOPY won't work for sure.

    if (gpioInitialise() < 0)
    {
        fprintf(stderr, "pigpio initialisation failed\n");
        return 1;
    }
    
    init_spi_bitbang();
    gpioWrite(rdy,LOW);
    gpioWrite(rdy,HIGH);
    gpioWrite(miso,LOW);

    printf("GPIO Initialized\n");
    printf("Starting MSXPi Server Version %s Build %s\n",version,build);
    
    while(true)
    {
                
        printf("msxpi-server: waiting command\n");        
        dataInfo = recvdatablock();

        if (dataInfo->rc == RC_SUCCESS) {
            *(dataInfo->data + dataInfo->datasize) = '\0';
            printf("msxpi-server received command: %s\n",dataInfo->data);
            ptest(dataInfo);
        }


        // Release the memory allocated for dataInfo pointer strucutre inside the recvdatablock
        free(dataInfo->data);
        free(dataInfo);
    }
    
    //create_disk
    /* Stop DMA, release resources */
    printf("Terminating GPIO\n");
    // fprintf(flog,"Terminating GPIO\n");chunkptr
    gpioWrite(rdy,LOW);
    
    //system("/sbin/shutdown now &");
    //system("/usr/sbin/killall msxpi-server &");
    
    
    return 0;
}
