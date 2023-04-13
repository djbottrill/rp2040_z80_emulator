#pragma once

//Define pins to use a virtual GPIO ports, -1 means not implemented
//GPIO for Virtual GPIO Port A
#define PortA0 0
#define PortA1 1
#define PortA2 2
#define PortA3 3
#define PortA4 -1
#define PortA5 -1
#define PortA6 -1
#define PortA7 -1

//GPIO for Virtual GPIO Port B
#define PortB0 -1
#define PortB1 -1
#define PortB2 -1
#define PortB3 -1
#define PortB4 -1
#define PortB5 -1
#define PortB6 -1
#define PortB7 -1

#define swA 12  //Breakpoint / Single Step push button
#define swB 13

const int _MISO = 4;
const int _CS   = 5;
const int _SCK  = 6;
const int _MOSI = 7;



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
