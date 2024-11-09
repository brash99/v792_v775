/******************************************************************************
*
*  c792Lib.h  -  Driver library header file for readout of a C.A.E.N. multiple
*                Model 792 QDCs using a VxWorks 5.2 or later based single board
*                computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          June 2000
*
*/
#ifndef __C792LIB__
#define __C792LIB__

#define C792_MAX_MODULES    20
#define C792_MAX_CHANNELS   32
#define C792_MAX_WORDS_PER_EVENT  34

/* Define a Structure for access to QDC*/
struct c792_struct {
  volatile unsigned int   data[512];
  unsigned int blank1[512];
  volatile unsigned short rev;
  volatile unsigned short geoAddr;
  volatile unsigned short cbltAddr;
  volatile unsigned short bitSet1;
  volatile unsigned short bitClear1;
  volatile unsigned short intLevel;
  volatile unsigned short intVector;
  volatile unsigned short status1;
  volatile unsigned short control1;
  volatile unsigned short aderHigh;
  volatile unsigned short aderLow;
  volatile unsigned short ssReset;
  unsigned short blank2;
  volatile unsigned short cbltControl;
  unsigned short blank3[2];
  volatile unsigned short evTrigger;
  volatile unsigned short status2;
  volatile unsigned short evCountL;
  volatile unsigned short evCountH;
  volatile unsigned short incrEvent;
  volatile unsigned short incrOffset;
  volatile unsigned short loadTest;
  unsigned short blank4;
  volatile unsigned short fclrWindow;
  volatile unsigned short bitSet2;
  volatile unsigned short bitClear2;
  volatile unsigned short wMemTestAddr;
  volatile unsigned short memTestWordH;
  volatile unsigned short memTestWordL;
  volatile unsigned short crateSelect;
  volatile unsigned short testEvWrite;
  volatile unsigned short evCountReset;
  unsigned short blank5[15];
  volatile unsigned short iped;
  unsigned short blank6;
  volatile unsigned short rTestAddr;
  unsigned short blank7;
  volatile unsigned short swComm;
  volatile unsigned short slideConst;
  unsigned short blank8[2];
  volatile unsigned short AAD;
  volatile unsigned short BAD;
  unsigned short blank9[6];
  volatile unsigned short threshold[C792_MAX_CHANNELS];
};

struct c792_ROM_struct {
  volatile unsigned short OUI_3;
  unsigned short blank1;
  volatile unsigned short OUI_2;
  unsigned short blank2;
  volatile unsigned short OUI_1;
  unsigned short blank3;
  volatile unsigned short version;
  unsigned short blank4;
  volatile unsigned short ID_3;
  unsigned short blank5;
  volatile unsigned short ID_2;
  unsigned short blank6;
  volatile unsigned short ID_1;
  unsigned short blank7[7];
  volatile unsigned short revision;
};


#define C792_BOARD_ID   0x00000318

/* Define Address offset for 68K based A24/D32 VME addressing */
/* default VMEChip2 programming only supports A24/D16 */
#define C792_68K_A24D32_OFFSET   0xe0000000

/* Define default interrupt vector/level */
#define C792_INT_VEC      0xaa
#define C792_VME_INT_LEVEL   4

#define C792_ROM_OFFSET    0x8026

/* Register Bits */
#define C792_VME_BUS_ERROR 0x8
#define C792_SOFT_RESET    0x80
#define C792_DATA_RESET    0x4

#define C792_BUFFER_EMPTY  0x2
#define C792_BUFFER_FULL   0x4

#define C792_DATA_READY    0x1
#define C792_BUSY          0x4

#define C792_BLK_END       0x04
#define C792_BERR_ENABLE   0x20
#define C792_ALIGN64       0x40

#define C792_MEM_TEST            0x1
#define C792_OFFLINE             0x2
#define C792_OVERFLOW_SUP        0x8
#define C792_UNDERFLOW_SUP      0x10
#define C792_TEST_MODE          0x40
#define C792_SLIDE_ENABLE       0x80
#define C792_AUTO_INCR         0x800
#define C792_INC_HEADER       0x1000
#define C792_SLIDE_SUB_ENABLE 0x2000
#define C792_INCR_ALL_TRIG    0x4000


#define C792_DATA           0x00000000
#define C792_HEADER_DATA    0x02000000
#define C792_TRAILER_DATA   0x04000000
#define C792_INVALID_DATA   0x06000000


/* Register Masks */
#define C792_BITSET1_MASK   0x0098
#define C792_INTLEVEL_MASK  0x0007
#define C792_INTVECTOR_MASK 0x00ff
#define C792_STATUS1_MASK   0x01ff
#define C792_CONTROL1_MASK  0x0034
#define C792_STATUS2_MASK   0x00f6
#define C792_BITSET2_MASK   0x7fff
#define C792_EVTRIGGER_MASK 0x001f

#define C792_DATA_ID_MASK    0x07000000
#define C792_WORDCOUNT_MASK  0x00003f00
#define C792_CHANNEL_MASK    0x003f0000
#define C792_CRATE_MASK      0x00ff0000
#define C792_EVENTCOUNT_MASK 0x00ffffff
#define C792_GEO_ADDR_MASK   0xf8000000
#define C792_ADC_DATA_MASK   0x00000fff

/* Function Prototypes */
STATUS c792Init (UINT32 addr, UINT32 addr_inc, int nadc, UINT16 crateID);
UINT32 c792ScanMask();
void   c792Status( int id, int reg, int sflag);
void   c792GStatus(int flag);
int    c792PrintEvent(int id, int pflag);
int    c792ReadEvent(int id, UINT32 *data);
int    c792FlushEvent(int id, int fflag);
int    c792ReadBlock(int id, volatile UINT32 *data, int nwrds);
STATUS c792IntConnect (VOIDFUNCPTR routine, int arg, UINT16 level, UINT16 vector);
STATUS c792IntEnable (int id, UINT16 evCnt);
STATUS c792IntDisable (int iflag);
STATUS c792IntResume (void);
UINT16 c792Sparse(int id, int over, int under);
unsigned int c792GDReady(unsigned int idmask, int nloop);
int    c792Dready(int id);
void   c792ClearThresh(int id);
short  c792SetThresh(int id, int chan, short val);
void   c792Gate(int id);
short  c792Control(int id, short val);
short  c792BitSet2(int id, short val);
void   c792BitClear2(int id, short val);
void   c792EnableBerr(int id);
void   c792DisableBerr(int id);
void   c792IncrEventBlk(int id, int count);
void   c792IncrEvent(int id);
void   c792IncrWord(int id);
void   c792Enable(int id);
void   c792Disable(int id);
void   c792Clear(int id);
void   c792Reset(int id);
void   c792EventCounterReset(int id);
int    c792SetGeoAddress(int id, int geo);

#endif /* __C792LIB__ */
