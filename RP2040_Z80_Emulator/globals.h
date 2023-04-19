#pragma once

//#define ILI9341

//*********************************************************************************************
//****                   3rd Party Board with ILI9341 LCD (experimental)                   ****
//*********************************************************************************************
#ifdef ILI9341
#include "Adafruit_GFX.h"
#include "Adafruit_ILI9341.h"

#define MISO  16
#define SS    17
#define SCK   18
#define MOSI  19

#define TFT_DC 20
#define TFT_CS 21
Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

//Define pins to use as virtual GPIO ports, -1 means not implemented
int PortA[8] = {  0,  1,  2,  3,  4,  5,  6,  7};   //Virtual GPIO Port A
int PortB[8] = {  8,  9, 10, 11, 12, 13, 14, 15};   //Virtual GPIO Port B

#define swA 22  //Breakpoint / Single Step push button


#else

//*********************************************************************************************
//****                         Raspberry Pi Pico (W) Board                                 ****
//*********************************************************************************************

//Default SPI0
#define MISO  16
#define SS    17
#define SCK   18
#define MOSI  19

//Define pins to use as virtual GPIO ports, -1 means not implemented
int PortA[8] = {  0,  1,  2,  3,  4,  5,  6,  7};   //Virtual GPIO Port A
int PortB[8] = {  8,  9, 10, 11, 12, 13, 14, 15};   //Virtual GPIO Port B

#define swA 22  //Breakpoint / Single Step push button


#endif

//Maker PI Pio board uses SPI1
//#define MISO  12
//#define SS    15
//#define SCK   10
//#define MOSI  11

//Waveshare Pico-ResTouch-LCD-3.5
//#define MISO  12
//#define SS    22
//#define SCK   10
//#define MOSI  11




//Virtual GPIO Port
const uint8_t GPP = 0;

//Virtual 8250 UART ports
const uint8_t UART_PORT = 0x80;
const uint8_t UART_LSR = UART_PORT + 5;

//Virtual disk controller ports
const uint8_t DCMD = 0x20;
const uint8_t DPARM = 0x30;

//Z80 Registers
uint8_t A = 0;
uint8_t Fl = 0;
uint8_t B = 0;
uint8_t C = 0;
uint8_t D = 0;
uint8_t E = 0;
uint8_t H = 0;
uint8_t L = 0;
uint16_t IX = 0;
uint16_t IY = 0;
uint16_t PC = 0;
uint16_t SP = 0;

//Z80 alternate registers
uint8_t Aa = 0;
uint8_t Fla = 0;
uint8_t Ba = 0;
uint8_t Ca = 0;
uint8_t Da = 0;
uint8_t Ea = 0;
uint8_t Ha = 0;
uint8_t La = 0;

//Z80 flags
bool Zf = false;  //Zero flag
bool Cf = false;  //Carry flag
bool Sf = false;  //Sign flag (MSBit of A Set)
bool Hf = false;  //Half Carry flag
bool Pf = false;  //Parity / Overflow flag
bool Nf = false;  //Add / Subtract flag

bool RUN = false;         //RUN flag
bool SingleStep = false;  //Single Step flag
bool intE = false;        //Interrupt enable
uint16_t BP = 0xffff;     //Breakpoint
uint8_t BPmode = 0;       //Breakpoint mode
bool bpOn = false;        //BP passed flag
uint8_t OC;               //Opcode store
uint8_t JR;               //Signed relative jump
uint8_t V8;               //8 Bit operand temp storge
uint16_t V16;             //16 Bit operand temp storge
uint16_t V16a;            //16 Bit operand temp storge
uint32_t V32;             //32 Bit operand temp storage used for 16 bit addition and subtraction
uint8_t v1;               //Temporary storage
uint8_t v2;               //Temporary storage
bool cfs;                 //Temp carry flag storage


uint8_t RAM[65536] = {};  //RAM
uint8_t pOut[256];        //Output port buffer
uint8_t pIn[256];         //Input port buffer
uint8_t rxBuf[1024];      //Serial receive buffer
uint16_t rxInPtr;         //Serial receive buffer input pointer
uint16_t rxOutPtr;        //Serial receive buffer output pointer
uint8_t txBuf[1024];      //Serial transmit buffer
uint16_t txInPtr;         //Serial transmit buffer input pointer
uint16_t txOutPtr;        //Serial transmit buffer output pointer

int vdrive;                        //Virtual drive number
char sdfile[50] = {};              //SD card filename
char sddir[50] = { "/download" };  //SD card path
bool sdfound = true;               //SD Card present flag
bool dled;                         //Disk activity flag


