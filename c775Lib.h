/******************************************************************************
*
*  c775Lib.h  -  Driver library header file for readout of a C.A.E.N. multiple
*                Model 775 TDCs using a VxWorks 5.2 or later based single board
*                computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          March 2002
*
*/
#ifndef __C775LIB__
#define __C775LIB__

#define C775_MAX_CHANNELS   32
#define C775_MAX_WORDS_PER_EVENT  34

/* Define a Structure for access to TDC*/
typedef struct
{
  /* 0x1000 */ volatile unsigned short rev;
  /* 0x1002 */ volatile unsigned short geoAddr;
  /* 0x1004 */ volatile unsigned short cbltAddr;
  /* 0x1006 */ volatile unsigned short bitSet1;
  /* 0x1008 */ volatile unsigned short bitClear1;
  /* 0x100A */ volatile unsigned short intLevel;
  /* 0x100C */ volatile unsigned short intVector;
  /* 0x100E */ volatile unsigned short status1;
  /* 0x1010 */ volatile unsigned short control1;
  /* 0x1012 */ volatile unsigned short aderHigh;
  /* 0x1014 */ volatile unsigned short aderLow;
  /* 0x1016 */ volatile unsigned short ssReset;
  /* 0x1018          */ unsigned short blank2;
  /* 0x101A */ volatile unsigned short cbltControl;
  /* 0x101C          */ unsigned short blank3[2];
  /* 0x1020 */ volatile unsigned short evTrigger;
  /* 0x1022 */ volatile unsigned short status2;
  /* 0x1024 */ volatile unsigned short evCountL;
  /* 0x1026 */ volatile unsigned short evCountH;
  /* 0x1028 */ volatile unsigned short incrEvent;
  /* 0x102A */ volatile unsigned short incrOffset;
  /* 0x102C */ volatile unsigned short loadTest;
  /* 0x102E          */ unsigned short blank4;
  /* 0x1030 */ volatile unsigned short fclrWindow;
  /* 0x1032 */ volatile unsigned short bitSet2;
  /* 0x1034 */ volatile unsigned short bitClear2;
  /* 0x1036 */ volatile unsigned short wMemTestAddr;
  /* 0x1038 */ volatile unsigned short memTestWordH;
  /* 0x103A */ volatile unsigned short memTestWordL;
  /* 0x103C */ volatile unsigned short crateSelect;
  /* 0x103E */ volatile unsigned short testEvWrite;
  /* 0x1040 */ volatile unsigned short evCountReset;
  /* 0x1042          */ unsigned short blank5[15];
  /* 0x1060 */ volatile unsigned short fsr;
  /* 0x1062          */ unsigned short blank6;
  /* 0x1064 */ volatile unsigned short rTestAddr;
  /* 0x1066          */ unsigned short blank7;
  /* 0x1068 */ volatile unsigned short swComm;
  /* 0x106A */ volatile unsigned short slideConst;
  /* 0x106C          */ unsigned short blank8[2];
  /* 0x1070 */ volatile unsigned short AAD;
  /* 0x1072 */ volatile unsigned short BAD;
  /* 0x1074          */ unsigned short blank9[(0x1080 - 0x1074)>>1];
  /* 0x1080 */ volatile unsigned short threshold[C775_MAX_CHANNELS];
} c775_main;

typedef struct
{
  /* 0x8000          */ unsigned short blank0[0x26>>1];
  /* 0x8026 */ volatile unsigned short OUI_3;
  /* 0x8028          */ unsigned short blank1;
  /* 0x802A */ volatile unsigned short OUI_2;
  /* 0x802C          */ unsigned short blank2;
  /* 0x802E */ volatile unsigned short OUI_1;
  /* 0x8030          */ unsigned short blank3;
  /* 0x8032 */ volatile unsigned short version;
  /* 0x8034          */ unsigned short blank4;
  /* 0x8036 */ volatile unsigned short ID_3;
  /* 0x8038          */ unsigned short blank5;
  /* 0x803A */ volatile unsigned short ID_2;
  /* 0x803C          */ unsigned short blank6;
  /* 0x803E */ volatile unsigned short ID_1;
  /* 0x8040          */ unsigned short blank7[(0x804E - 0x8040)>>1];
  /* 0x804E */ volatile unsigned short revision;
  /* 0x8050          */ unsigned short blank8[(0x8F02 - 0x8050)>>1];
  /* 0x8F02 */ volatile unsigned short serial_msb;
  /* 0x8F04          */ unsigned short blank9;
  /* 0x8F06 */ volatile unsigned short serial_lsb;
}  c775_ROM;

/* Define a Structure for access to TDC*/
typedef struct
{
  /* 0x0000 */ volatile unsigned int data[512];
  /* 0x0800          */ unsigned int blank1[(0x1000 - 0x0800)>>2];
  /* 0x1000          */ c775_main main;
  /* 0x10C0          */ unsigned int blank2[(0x8000 - 0x10C0)>>2];
  /* 0x8000          */ c775_ROM  rom;
}  c775_regs;


#define C775_BOARD_ID   0x00000307

/* Define Address offset for 68K based A24/D32 VME addressing */
/* default VMEChip2 programming only supports A24/D16 */
#define C775_68K_A24D32_OFFSET   0xe0000000

/* Define default interrupt vector/level */
#define C775_INT_VEC      0xaa
#define C775_VME_INT_LEVEL   4

#define C775_MIN_FSR       140	/* nsec (High resoulution) */
#define C775_MAX_FSR      1200	/* nsec (Low resoulution) */

#define C775_ROM_OFFSET    0x8026

/* Register Bits */
#define C775_VME_BUS_ERROR 0x8
#define C775_SOFT_RESET    0x80
#define C775_DATA_RESET    0x4

#define C775_BUFFER_EMPTY  0x2
#define C775_BUFFER_FULL   0x4

/* status1 */
#define C775_DATA_READY    0x1
#define C775_BUSY          0x4
#define C775_EVRDY         0x100

/* control */
#define C775_BLK_END       0x04
#define C775_BERR_ENABLE   0x20
#define C775_ALIGN64       0x40

/* bitset2 */
#define C775_MEM_TEST            0x1
#define C775_OFFLINE             0x2
#define C775_OVER_RANGE          0x8
#define C775_LOW_THRESHOLD      0x10
#define C775_INVALID_SUP        0x20
#define C775_TEST_MODE          0x40
#define C775_SLIDE_ENABLE       0x80
#define C775_COMMON_STOP       0x400
#define C775_AUTO_INCR         0x800
#define C775_INC_HEADER       0x1000
#define C775_SLIDE_SUB_ENABLE 0x2000
#define C775_INCR_ALL_TRIG    0x4000


#define C775_DATA           0x00000000
#define C775_HEADER_DATA    0x02000000
#define C775_TRAILER_DATA   0x04000000
#define C775_INVALID_DATA   0x06000000


/* Register Masks */
#define C775_BITSET1_MASK   0x0098
#define C775_INTLEVEL_MASK  0x0007
#define C775_INTVECTOR_MASK 0x00ff
#define C775_STATUS1_MASK   0x01ff
#define C775_CONTROL1_MASK  0x0074
#define C775_STATUS2_MASK   0x00f6
#define C775_BITSET2_MASK   0x7fff
#define C775_EVTRIGGER_MASK 0x001f
#define C775_FSR_MASK       0x00ff

#define C775_DATA_ID_MASK    0x07000000
#define C775_WORDCOUNT_MASK  0x00003f00
#define C775_CHANNEL_MASK    0x003f0000
#define C775_CRATE_MASK      0x00ff0000
#define C775_EVENTCOUNT_MASK 0x00ffffff
#define C775_GEO_ADDR_MASK   0xf8000000
#define C775_TDC_DATA_MASK   0x00000fff

/* Function Prototypes */
STATUS c775Init(UINT32 addr, UINT32 addr_inc, int nadc, UINT16 crateID);
void c775Status(int id);
int c775PrintEvent(int id, int pflag);
int c775ReadEvent(int id, UINT32 * data);
int c775FlushEvent(int id, int fflag);
int c775ReadBlock(int id, volatile UINT32 * data, int nwrds);
STATUS c775IntConnect(VOIDFUNCPTR routine, int arg, UINT16 level,
		      UINT16 vector);
STATUS c775IntEnable(int id, UINT16 evCnt);
STATUS c775IntDisable(int iflag);
STATUS c775IntResume(void);
UINT16 c775Sparse(int id, int over, int under);
int c775Dready(int id);
int c775SetFSR(int id, UINT16 fsr);
INT16 c775BitSet2(int id, UINT16 val);
INT16 c775BitClear2(int id, UINT16 val);
void c775ClearThresh(int id);
void c775Gate(int id);
void c775IncrEventBlk(int id, int count);
void c775IncrEvent(int id);
void c775IncrWord(int id);
void c775Enable(int id);
void c775Disable(int id);
void c775CommonStop(int id);
void c775CommonStart(int id);
void c775Clear(int id);
void c775Reset(int id);

#endif /* __C775LIB__ */
