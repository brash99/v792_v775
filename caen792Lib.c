/******************************************************************************
*
*  caen792Lib.c  -  Driver library for readout of C.A.E.N. Model 792 QDC
*                   using a VxWorks 5.2 or later based Single Board computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          July 2000
*
*  Revision  1.0 - Initial Revision
*                    - Supports up to 20 CAEN Model 792s in a Crate
*                    - Programmed I/O reads
*                    - Interrupts from a Single 792
*
*  Revision  1.0.1 - Ported to Linux
*
*/

#ifdef VXWORKS
#include "vxWorks.h"
#include "logLib.h"
#include "taskLib.h"
#include "intLib.h"
#include "iv.h"
#include "semLib.h"
#include "vxLib.h"
#include "fppLib.h"
#else
#include <pthread.h>
#endif
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "jvme.h"

/* Include QDC definitions */
#include "c792Lib.h"


/* Include DMA Library definintions */
#ifdef VXWORKSPPC
#include "universeDma.h"
#elif defined(VXWORKS68K51)
#include "mvme_dma.c"
#endif


#ifdef VXWORKS
/* Define external Functions */
IMPORT  STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT  STATUS intDisconnect(int);
IMPORT  STATUS sysIntEnable(int);
IMPORT  STATUS sysIntDisable(int);
#endif

/* Mutex to guard c792 reads/writes - Linux only */
#ifdef VXWORKS
#define C792LOCK
#define C792UNLOCK
#else
pthread_mutex_t c792mutex = PTHREAD_MUTEX_INITIALIZER;
#define C792LOCK   if(pthread_mutex_lock(&c792mutex)<0) perror("pthread_mutex_lock");
#define C792UNLOCK if(pthread_mutex_unlock(&c792mutex)<0) perror("pthread_mutex_unlock");
#endif

/* Define Interrupts variables */
BOOL              c792IntRunning  = FALSE;                    /* running flag */
int               c792IntID       = -1;                       /* id number of QDC generating interrupts */
LOCAL VOIDFUNCPTR c792IntRoutine  = NULL;                     /* user interrupt service routine */
LOCAL int         c792IntArg      = 0;                        /* arg to user routine */
LOCAL int         c792IntEvCount  = 0;                        /* Number of Events to generate Interrupt */
LOCAL UINT32      c792IntLevel    = C792_VME_INT_LEVEL;       /* default VME interrupt level */
LOCAL UINT32      c792IntVec      = C792_INT_VEC;             /* default interrupt Vector */


/* Define global variables */
int Nc792 = 0;                                /* Number of QDCs in Crate */
volatile struct c792_struct *c792p[C792_MAX_MODULES];       /* pointers to QDC memory map */
volatile struct c792_struct *c792pl[C792_MAX_MODULES];      /* Support for 68K second memory map A24/D32 */
int c792IntCount = 0;                         /* Count of interrupts from QDC */
int c792EventCount[C792_MAX_MODULES];                       /* Count of Events taken by QDC (Event Count Register value) */
int c792EvtReadCnt[C792_MAX_MODULES];                       /* Count of events read from specified QDC */

unsigned long c792MemOffset = 0;               /* CPUs A24 or A32 address space offset */

#ifdef VXWORKS
FP_CONTEXT c792Fpr;

SEM_ID c792Sem;                               /* Semephore for Task syncronization */
#endif

/* Macros */
#define C792_EXEC_SOFT_RESET(id) {					\
    vmeWrite16(&c792p[id]->bitSet1, C792_SOFT_RESET);			\
    vmeWrite16(&c792p[id]->bitClear1, C792_SOFT_RESET);}

#define C792_EXEC_DATA_RESET(id) {					\
    vmeWrite16(&c792p[id]->bitSet2, C792_DATA_RESET);			\
    vmeWrite16(&c792p[id]->bitClear2, C792_DATA_RESET);}

#define C792_EXEC_READ_EVENT_COUNT(id) {				\
    volatile unsigned short s1, s2;					\
    s1 = vmeRead16(&c792p[id]->evCountL);				\
    s2 = vmeRead16(&c792p[id]->evCountH);				\
    c792EventCount[id] = (c792EventCount[id]&0xff000000) +		\
      (s2<<16) +							\
      (s1);}
#define C792_EXEC_SET_EVTREADCNT(id,val) {			       \
    if(c792EvtReadCnt[id] < 0)					       \
      c792EvtReadCnt[id] = val;					       \
    else							       \
      c792EvtReadCnt[id] = (c792EvtReadCnt[id]&0x7f000000) + val;}

#define C792_EXEC_CLR_EVENT_COUNT(id) {		\
    vmeWrite16(&c792p[id]->evCountReset, 1);	\
    c792EventCount[id] = 0;}
#define C792_EXEC_INCR_EVENT(id) {		\
    vmeWrite16(&c792p[id]->incrEvent, 1);	\
    c792EvtReadCnt[id]++;}
#define C792_EXEC_INCR_WORD(id) {		\
    vmeWrite16(&c792p[id]->incrOffset, 1);}
#define C792_EXEC_GATE(id) {			\
    vmeWrite16(&c792p[id]->swComm, 1);}


/*******************************************************************************
*
* c792Init - Initialize c792 Library.
*
*
* RETURNS: OK, or ERROR if the address is invalid or board is not present.
*/

STATUS
c792Init (UINT32 addr, UINT32 addr_inc, int nadc, UINT16 crateID)
{
  int ii, res, rdata, errFlag = 0;
  int boardID = 0;
  unsigned long laddr, lladdr;
  volatile struct c792_ROM_struct *rp;


  /* Check for valid address */
  if(addr==0) {
    printf("c792Init: ERROR: Must specify a Bus (VME-based A32/A24) address for QDC 0\n");
    return(ERROR);
  }else if(addr < 0x00ffffff) { /* A24 Addressing */
    if((addr_inc==0)||(nadc==0))
      nadc = 1; /* assume only one QDC to initialize */

    /* get the QDCs address */
#ifdef VXWORKS
    res = sysBusToLocalAdrs(0x39,(char *)addr,(char **)&laddr);
    if (res != 0) {
      printf("c792Init: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
      return(ERROR);
    }
#else
    res = vmeBusToLocalAdrs(0x39,(char *) (unsigned long)addr,(char **)&laddr);
    if (res != 0) {
      printf("c792Init: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",addr);
      return(ERROR);
    }
#endif
    c792MemOffset = laddr - addr;
  }else{ /* A32 Addressing */

#ifdef VXWORKS68K51
    printf("c792Init: ERROR: 68K Based CPU cannot support A32 addressing (use A24)\n");
    return(ERROR);
#endif

    if((addr_inc==0)||(nadc==0))
      nadc = 1; /* assume only one QDC to initialize */

    /* get the QDC address */
#ifdef VXWORKS
    res = sysBusToLocalAdrs(0x09,(char *)addr,(char **)&laddr);
    if (res != 0) {
      printf("c792Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",addr);
      return(ERROR);
    }
#else
    res = vmeBusToLocalAdrs(0x09,(char *)(unsigned long)addr,(char **)&laddr);
    if (res != 0) {
      printf("c792Init: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",addr);
      return(ERROR);
    }
#endif
    c792MemOffset = laddr - addr;
  }

  /* Put in Hack for 68K seperate address spaces for A24/D16 and A24/D32 */
  /* for PowerPC they are one and the same */
#ifdef VXWORKS68K51
  lladdr = C792_68K_A24D32_OFFSET + (laddr&0x00ffffff);
#else
  lladdr = laddr;
#endif

  Nc792 = 0;
  for (ii=0;ii<nadc;ii++) {
    c792p[ii] = (struct c792_struct *)(laddr + ii*addr_inc);
    c792pl[ii]  = (struct c792_struct *)(lladdr + ii*addr_inc);
    /* Check if Board exists at that address */
#ifdef VXWORKS
    res = vxMemProbe((char *) &(c792p[ii]->rev),0,2,(char *)&rdata);
#else
    res = vmeMemProbe((char *) &(c792p[ii]->rev),2,(char *)&rdata);
#endif
    if(res < 0) {
      printf("c792Init: ERROR: No addressable board at VME (local) addr=0x%08lx (0x%lx)\n",
	     (unsigned long) c792p[ii] - c792MemOffset,
	     (unsigned long) c792p[ii]);
      c792p[ii] = NULL;
      errFlag = 1;
      break;
    } else {
      /* Check if this is a Model 792 */
      rp = (struct c792_ROM_struct *)((unsigned long)c792p[ii] + C792_ROM_OFFSET);
      boardID = ((vmeRead16(&rp->ID_3)&(0xff))<<16) +
	((vmeRead16(&rp->ID_2)&(0xff))<<8) + (vmeRead16(&rp->ID_1)&(0xff));
      if(boardID != C792_BOARD_ID) {
	printf(" ERROR: Board ID does not match: %d \n",boardID);
	return(ERROR);
      }
    }
    Nc792++;
#ifdef VXWORKS
    printf("Initialized QDC ID %d at address 0x%08x \n",ii,(UINT32) c792p[ii]);
#else
    printf("Initialized QDC ID %d at VME (local) address 0x%08lx (0x%lx) \n",ii,
	   (unsigned long)(c792p[ii]) - c792MemOffset, (unsigned long) c792p[ii]);
#endif
  }

#ifdef VXWORKS
  /* Initialize/Create Semephore */
  if(c792Sem != 0) {
    semFlush(c792Sem);
    semDelete(c792Sem);
  }
  c792Sem = semBCreate(SEM_Q_PRIORITY,SEM_EMPTY);
  if(c792Sem <= 0) {
    printf("c792Init: ERROR: Unable to create Binary Semephore\n");
    return(ERROR);
  }
#endif

  /* Disable/Clear all QDCs */
  for(ii=0;ii<Nc792;ii++) {
    C792_EXEC_SOFT_RESET(ii);
    C792_EXEC_DATA_RESET(ii);
    vmeWrite16(&c792p[ii]->intLevel,0);        /* Disable Interrupts */
    vmeWrite16(&c792p[ii]->evTrigger,0);       /* Zero interrupt trigger count */
    vmeWrite16(&c792p[ii]->crateSelect,crateID);  /* Set Crate ID Register */
    vmeWrite16(&c792p[ii]->bitClear2,C792_INCR_ALL_TRIG); /* Increment event count only on
							    accepted gates */

    c792EventCount[ii] =  0;          /* Initialize the Event Count */
    c792EvtReadCnt[ii] = -1;          /* Initialize the Read Count */
  }
  /* Initialize Interrupt variables */
  c792IntID = -1;
  c792IntRunning = FALSE;
  c792IntLevel = 0;
  c792IntVec = 0;
  c792IntRoutine = NULL;
  c792IntArg = 0;
  c792IntEvCount = 0;

#ifdef VXWORKSPPC
  bzero((char *)&c792Fpr,sizeof(c792Fpr));
#endif

  if(errFlag > 0) {
    printf("c792Init: ERROR: Unable to initialize all QDC Modules\n");
    if(Nc792 > 0)
      printf("c7922Init: %d QDC(s) successfully initialized\n",Nc792);
    return(ERROR);
  } else {
    return(OK);
  }
}

UINT32
c792ScanMask()
{
  int iadc=0;
  UINT32 rval=0;

  for(iadc=0; iadc<Nc792; iadc++)
    rval |= 1<<iadc;

  return rval;
}

/*******************************************************************************
*
* c792Status - Gives Status info on specified QDC
*
*
* RETURNS: None
*/

void
c792Status( int id, int reg, int sflag)
{

  int DRdy=0, BufFull=0;
  int BlkEnd=0, Berr=0;
  UINT16 stat1, stat2, bit1, bit2, cntl1;
  UINT16 iLvl, iVec, evTrig;



  if((id<0) || (c792p[id] == NULL)) {
    printf("c792Status: ERROR : QDC id %d not initialized \n",id);
    return;
  }


  /* read various registers */
  C792LOCK;
  stat1 = vmeRead16(&c792p[id]->status1)&C792_STATUS1_MASK;
  stat2 = vmeRead16(&c792p[id]->status2)&C792_STATUS2_MASK;
  bit1 =  vmeRead16(&c792p[id]->bitSet1)&C792_BITSET1_MASK;
  bit2 =  vmeRead16(&c792p[id]->bitSet2)&C792_BITSET2_MASK;
  cntl1 = vmeRead16(&c792p[id]->control1)&C792_CONTROL1_MASK;
  C792_EXEC_READ_EVENT_COUNT(id);
  iLvl = vmeRead16(&c792p[id]->intLevel)&C792_INTLEVEL_MASK;
  iVec = vmeRead16(&c792p[id]->intVector)&C792_INTVECTOR_MASK;
  evTrig = vmeRead16(&c792p[id]->evTrigger)&C792_EVTRIGGER_MASK;
  C792UNLOCK;

  /* Get info from registers */
  if(stat1&C792_DATA_READY) DRdy = 1;
  if(stat2&C792_BUFFER_FULL) BufFull = 1;
  if(cntl1&C792_BERR_ENABLE) Berr = 1;
  if(cntl1&C792_BLK_END) BlkEnd = 1;

  /* print out status info */

#ifdef VXWORKS
  printf("STATUS for QDC id %d at base address 0x%x \n",id,(UINT32) c792p[id]);
#else
  printf("STATUS for QDC id %d at base VME (local) address 0x%08lx (0x%lx)\n",id,
	 (unsigned long)(c792p[id]) - c792MemOffset,
	 (unsigned long) c792p[id]);
#endif
  printf("---------------------------------------------- \n");

  if( (iLvl>0) && (evTrig>0)) {
    printf(" Interrupts Enabled - Every %d events\n",evTrig);
    printf(" VME Interrupt Level: %d   Vector: 0x%x \n",iLvl,iVec);
    printf(" Interrupt Count    : %d \n",c792IntCount);
  } else {
    printf(" Interrupts Disabled\n");
    printf(" Last Interrupt Count    : %d \n",c792IntCount);
  }
  printf("\n");

  printf("             --1--  --2--\n");
  if(BufFull && DRdy) {
    printf("  Status  = 0x%04x 0x%04x  (Buffer Full)\n",stat1,stat2);
  } else if(DRdy) {
    printf("  Status  = 0x%04x 0x%04x  (Data Ready)\n",stat1,stat2);
  }else{
    printf("  Status  = 0x%04x 0x%04x\n",stat1,stat2);
  }
  printf("  BitSet  = 0x%04x 0x%04x\n",bit1,bit2);

  if(Berr && BlkEnd) {
    printf("  Control = 0x%04x         (Bus Error,Block End Enabled)\n",cntl1);
  } else if(Berr) {
    printf("  Control = 0x%04x         (Bus Error Enabled)\n",cntl1);
  } else if(BlkEnd) {
    printf("  Control = 0x%04x         (Block End Enabled)\n",cntl1);
  } else {
    printf("  Control = 0x%04x\n",cntl1);
  }

  if(c792EventCount[id] == 0xffffff) {
    printf("  Event Count     = (No Events Taken)\n");
    printf("  Last Event Read = (No Events Read)\n");
  }else{
    printf("  Event Count     = %d\n",c792EventCount[id]);
    if(c792EvtReadCnt[id] == -1)
      printf("  Last Event Read = (No Events Read)\n");
    else
      printf("  Last Event Read = %d\n",c792EvtReadCnt[id]);
  }

}

/*******************************************************************************
*
* c792GStatus - Gives Summary Status info on all initialized QDCs
*
*
* RETURNS: None
*/

void
c792GStatus(int flag)
{
  struct c792_struct *r792;
  unsigned long *addr;
  int iadc;

  r792 = (struct c792_struct *)malloc(C792_MAX_MODULES*sizeof(struct c792_struct));
  if(r792 == NULL)
    {
      printf("%s: ERROR: Out of Memory\n", __func__);
      return;
    }

  addr = (unsigned long *)malloc(C792_MAX_MODULES*sizeof(unsigned long));
  if(addr == NULL)
    {
      printf("%s: ERROR: Out of Memory\n", __func__);
      return;
    }

  C792LOCK;
  for(iadc = 0; iadc < Nc792; iadc++)
    {
      addr[iadc] = (unsigned long)c792p[iadc] - c792MemOffset;

      r792[iadc].rev = vmeRead16(&c792p[iadc]->rev);
      r792[iadc].geoAddr = vmeRead16(&c792p[iadc]->geoAddr);
      r792[iadc].cbltAddr = vmeRead16(&c792p[iadc]->cbltAddr);
      r792[iadc].bitSet1 = vmeRead16(&c792p[iadc]->bitSet1);
      r792[iadc].status1 = vmeRead16(&c792p[iadc]->status1);
      r792[iadc].control1 = vmeRead16(&c792p[iadc]->control1);
      r792[iadc].cbltControl = vmeRead16(&c792p[iadc]->cbltControl);
      r792[iadc].evTrigger = vmeRead16(&c792p[iadc]->evTrigger);
      r792[iadc].status2 = vmeRead16(&c792p[iadc]->status2);
      r792[iadc].evCountL = vmeRead16(&c792p[iadc]->evCountL);
      r792[iadc].evCountH = vmeRead16(&c792p[iadc]->evCountH);
      r792[iadc].fclrWindow = vmeRead16(&c792p[iadc]->fclrWindow);
      r792[iadc].bitSet2 = vmeRead16(&c792p[iadc]->bitSet2);

    }
  C792UNLOCK;

  printf("\n");
  /* Parameters from Registers */
  printf("                    CAEN792 ADC Module Status\n\n");
  printf("            Firmware                                          Control\n");
  printf("  #  GEO    Revision     Address      CBLT/MCST Address       Termination\n");
  printf("--------------------------------------------------------------------------------\n");

  for(iadc = 0; iadc < Nc792; iadc++)
    {
      printf(" %2d  ",iadc);

      printf("%2d     ", r792[iadc].geoAddr & 0x1F);

      printf("0x%04x       ",r792[iadc].rev);

      printf("0x%08lx   ",addr[iadc]);

      printf("0x%08x - ", (r792[iadc].cbltAddr)<<24);

      printf("%s   ",
	     (r792[iadc].cbltControl&0x3)==0?"DISABLED":
	     (r792[iadc].cbltControl&0x3)==1?"LAST    ":
	     (r792[iadc].cbltControl&0x3)==2?"FIRST   ":
	     (r792[iadc].cbltControl&0x3)==3?"MIDDLE  ":
	     "        ");

      printf("%s",
	     (r792[iadc].status1 & 0xC0)==0x40?"ON":
	     (r792[iadc].status1 & 0xC0)==0x80?"OFF":
	     "MIXED");

      printf("\n");
    }

  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("                              Readout Configuration\n\n");
  printf("     Bus     FP     Block  Align  Fast Clear   Suppress   Auto  Empty  Trg\n");
  printf("  #  Errors  Reset  End    64     Window [us]  OF   Zero  Incr  Block  Counter\n");
  printf("--------------------------------------------------------------------------------\n");

  for(iadc = 0; iadc < Nc792; iadc++)
    {
      printf(" %2d  ",iadc);

      printf("%s     ",
	     (r792[iadc].control1 & 0x20)?"ON ":"OFF");

      printf("%s  ",
	     (r792[iadc].control1 & 0x80)?"RESET":"CLEAR");

      printf("%s    ",
	     (r792[iadc].control1 & 0x4)?"ALL":"EOB");

      printf("%s    ",
	     (r792[iadc].control1 & 0x40)?"ON ":"OFF");

      printf("%4.1f         ",
	     ((float)((r792[iadc].fclrWindow & 0x3ff))/32.0) + 7.0);

      printf("%s  ",
	     (r792[iadc].bitSet2 & 0x8)?"OFF":"ON ");

      printf("%s   ",
	     (r792[iadc].bitSet2 & 0x10)?"OFF":"ON ");

      printf("%s   ",
	     (r792[iadc].bitSet2 & 0x800)?"ON ":"OFF");

      printf("%s    ",
	     (r792[iadc].bitSet2 & 0x1000)?"ON ":"OFF");

      printf("%s",
	     (r792[iadc].bitSet2 & 0x4000)?"All":"Accepted");

      printf("\n");

    }
  printf("\n");

  printf("                                 Data Status\n\n");
  printf("     Output    Data     Block    Block     Busy      Event\n");
  printf("  #  Buffer    Ready    Level    Status    Status    Counter\n");
  printf("--------------------------------------------------------------------------------\n");
  for(iadc = 0; iadc < Nc792; iadc++)
    {
      printf(" %2d  ",iadc);

      printf("%s     ",
	     (r792[iadc].status2 & 0x6) == 0x2 ?"EMPTY":
	     (r792[iadc].status2 & 0x6) == 0x4 ?" FULL":
	     "AVAIL");

      printf("%s   ",
	     (r792[iadc].status2 & 0x1)?"READY":"-----");

      printf("%2d        ",
	     r792[iadc].evTrigger & 0x1F);

      printf("%s     ",
	     (r792[iadc].status2 & 0x10)?"READY":"-----");

      printf("%s      ",
	     (r792[iadc].status2 & 0x4)?"BUSY":"----");

      unsigned int evCount = ((unsigned int)(r792[iadc].evCountH & 0xFF) << 16) |
	(unsigned int)r792[iadc].evCountL;
      printf("%-8d   ", evCount);

      printf("\n");
    }
  printf("--------------------------------------------------------------------------------\n");

  printf("\n");
  printf("\n");

  if(r792)
    free(r792);
}

/*******************************************************************************
*
* c792PrintEvent - Print event from QDC to standard out.
*
*
* RETURNS: Number of Data words read from the QDC (including Header/Trailer).
*/

int
c792PrintEvent(int id, int pflag)
{

  int ii, nWords, evID;
  UINT32 header, trailer, dCnt;

  if((id<0) || (c792p[id] == NULL)) {
    printf("c792Printevent: ERROR : QDC id %d not initialized \n",id);
    return(-1);
  }

  /* Check if there is a valid event */

  C792LOCK;
  if(vmeRead16(&c792p[id]->status2)&C792_BUFFER_EMPTY) {
    printf("c792PrintEvent: Data Buffer is EMPTY!\n");
    C792UNLOCK;
    return(0);
  }
  if(vmeRead16(&c792p[id]->status1)&C792_DATA_READY) {
    dCnt = 0;
    /* Read Header - Get Word count */
    header = vmeRead32(&c792pl[id]->data[0]);
    if((header&C792_DATA_ID_MASK) != C792_HEADER_DATA) {
      printf("c792PrintEvent: ERROR: Invalid Header Word 0x%08x\n",header);
      C792UNLOCK;
      return(-1);
    }else{
      printf("  ADC DATA for Module %d\n",id);
      nWords = (header&C792_WORDCOUNT_MASK)>>8;
      dCnt++;
      printf("  Header: 0x%08x   nWords = %d ",header,nWords);
    }
    for(ii=0;ii<nWords;ii++) {
      if ((ii % 5) == 0) printf("\n    ");
      printf("  0x%08x",(UINT32) vmeRead32(&c792pl[id]->data[ii+1]));
    }
    printf("\n");
    dCnt += ii;

    trailer = vmeRead32(&c792pl[id]->data[dCnt]);
    if((trailer&C792_DATA_ID_MASK) != C792_TRAILER_DATA) {
      printf("c792PrintEvent: ERROR: Invalid Trailer Word 0x%08x\n",trailer);
      C792UNLOCK;
      return(-1);
    }else{
      evID = trailer&C792_EVENTCOUNT_MASK;
      dCnt++;
      printf("  Trailer: 0x%08x   Event Count = %d \n",trailer,evID);
    }
    C792_EXEC_SET_EVTREADCNT(id,evID);
    C792UNLOCK;
    return (dCnt);

  }else{
    printf("c792PrintEvent: Data Not ready for readout!\n");
    C792UNLOCK;
    return(0);
  }

  C792UNLOCK;

}


/*******************************************************************************
*
* c792ReadEvent - Read event from QDC to specified address.
*
*
*
* RETURNS: Number of Data words read from the QDC (including Header/Trailer).
*/

int
c792ReadEvent(int id, UINT32 *data)
{

  int ii, nWords, evID;
  UINT32 header, trailer, dCnt;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792ReadEvent: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(-1);
  }

  /* Check if there is a valid event */

  C792LOCK;
  if(vmeRead16(&c792p[id]->status2)&C792_BUFFER_EMPTY) {
    logMsg("c792ReadEvent: Data Buffer is EMPTY!\n",0,0,0,0,0,0);
    C792UNLOCK;
    return(0);
  }
  if(vmeRead16(&c792p[id]->status1)&C792_DATA_READY) {
    dCnt = 0;
    /* Read Header - Get Word count */
    header = vmeRead32(&c792pl[id]->data[dCnt]);
    if((header&C792_DATA_ID_MASK) != C792_HEADER_DATA) {
      logMsg("c792ReadEvent: ERROR: Invalid Header Word 0x%08x (0x%08x)\n",header,c792pl[id]->data[dCnt],0,0,0,0);
      C792UNLOCK;
      return(-1);
    }else{
      nWords = (header&C792_WORDCOUNT_MASK)>>8;
#ifndef VXWORKS
      // Swap endian-ness to be consistent with the data.
      header = LSWAP(header);
#endif
      data[dCnt] = header;
      dCnt++;
    }
    for(ii=0;ii<nWords;ii++) {
      data[ii+1] = c792pl[id]->data[ii+1];
    }
    dCnt += ii;

    trailer = vmeRead32(&c792pl[id]->data[dCnt]);
    if((trailer&C792_DATA_ID_MASK) != C792_TRAILER_DATA) {
      logMsg("c792ReadEvent: ERROR: Invalid Trailer Word 0x%08x\n",trailer,0,0,0,0,0);
      C792UNLOCK;
      return(-1);
    }else{
      evID = trailer&C792_EVENTCOUNT_MASK;
#ifndef VXWORKS
      // Swap endian-ness to be consistent with the data.
      trailer = LSWAP(trailer);
#endif
      data[dCnt] = trailer;
      dCnt++;
    }
    C792_EXEC_SET_EVTREADCNT(id,evID);
    C792UNLOCK;
    return (dCnt);

  }else{
    logMsg("c792ReadEvent: Data Not ready for readout!\n",0,0,0,0,0,0);
    C792UNLOCK;
    return(0);
  }

  C792UNLOCK;

}

/*******************************************************************************
*
* c792FlushEvent - Flush event/data from QDC.
*
*
* RETURNS: Number of Data words read from the QDC.
*/

int
c792FlushEvent(int id, int fflag)
{

  int evID;
  int done = 0;
  UINT32 tmpData, dCnt;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792FlushEvent: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(-1);
  }

  /* Check if there is a valid event */

  C792LOCK;
  if((c792p[id]->status2)&C792_BUFFER_EMPTY) {
    if(fflag > 0) logMsg("c792FlushEvent: Data Buffer is EMPTY!\n",0,0,0,0,0,0);
    C792UNLOCK;
    return(0);
  }

  /* Check if Data Ready Flag is on */
  if(vmeRead16(&c792p[id]->status1)&C792_DATA_READY) {
    dCnt = 0;

    while (!done) {
      tmpData = vmeRead32(&c792pl[id]->data[dCnt]);
      switch (tmpData&C792_DATA_ID_MASK) {
      case C792_HEADER_DATA:
	if(fflag > 0) logMsg("c792FlushEvent: Found Header 0x%08x\n",tmpData,0,0,0,0,0);
	break;
      case C792_DATA:
	break;
      case C792_TRAILER_DATA:
	if(fflag > 0) logMsg(" c792FlushEvent: Found Trailer 0x%08x\n",tmpData,0,0,0,0,0);
	evID = tmpData&C792_EVENTCOUNT_MASK;
	C792_EXEC_SET_EVTREADCNT(id,evID);
	done = 1;
	break;
      case C792_INVALID_DATA:
	if(fflag > 0) logMsg(" c792FlushEvent: Buffer Empty 0x%08x\n",tmpData,0,0,0,0,0);
	done = 1;
	break;
      default:
	if(fflag > 0) logMsg(" c792FlushEvent: Invalid Data 0x%08x\n",tmpData,0,0,0,0,0);
      }

      /* Print out Data */
      if(fflag > 1) {
	if ((dCnt % 5) == 0) printf("\n    ");
	printf("  0x%08x ",tmpData);
      }
      dCnt++;
    }
    if(fflag > 1) printf("\n");

    C792UNLOCK;
    return (dCnt);

  }else{
    if(fflag > 0) logMsg("c792FlushEvent: Data Not ready for readout!\n",0,0,0,0,0,0);
    C792UNLOCK;
    return(0);
  }

  C792UNLOCK;

}


/*******************************************************************************
*
* c792ReadBlock - Read Block of events from QDC to specified address.
*
* INPUTS:    id     - module id of QDC to access
*            data   - address of data destination
*            nwrds  - number of data words to transfer
*
* RETURNS: OK or ERROR on success of transfer.
*
* Note: User must call c792IncrEventBlk after a successful
*       call to c792ReadBlock to increment the number of events Read.
*         (e.g.   c792IncrEventBlk(0,15);
*/

int
c792ReadBlock(int id, volatile UINT32 *data, int nwrds)
{

  int retVal, xferCount=0;
  int dummy=0;
  volatile unsigned int *laddr;
  UINT32 evID, vmeAdr;
  UINT16 reg = 0, stat = 0;
  UINT32 trailer;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792ReadBlock: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(ERROR);
  }

  /* Check for 8 byte boundary for address - insert dummy word (Slot 0 FADC Dummy DATA)*/
  if((unsigned long) (data)&0x7)
    {
#ifdef VXWORKS
      *data = C792_INVALID_DATA;
#else
      *data = LSWAP(C792_INVALID_DATA);
#endif
      dummy = 1;
      laddr = (data + 1);
      xferCount = 1;
    }
  else
    {
      dummy = 0;
      laddr = data;
      xferCount = 0;
    }


  C792LOCK;
#ifdef VXWORKSPPC
  /* Don't bother checking if there is a valid event. Just blast data out of the
     FIFO Valid or Invalid
     Also assume that the Universe DMA programming is setup. */

  vmeAdr = (UINT32)(c792p[id]->data) - c792MemOffset;
  retVal = sysVmeDmaSend((UINT32)laddr, vmeAdr, (nwrds<<2), 0);
  if(retVal < 0) {
    logMsg("c792ReadBlock: ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
    C792UNLOCK;
    return(ERROR);
  }
  /* Wait until Done or Error */
  retVal = sysVmeDmaDone(1000,1);

#elif defined(VXWORKS68K51)

  /* 68K Block 32 transfer from FIFO using VME2Chip */
  retVal = mvme_dma((long)laddr, 1, (long)(c792pl[id]->data), 0, nwrds, 1);

#else
  /* Linux readout with jvme library */
  vmeAdr = (unsigned long)(c792p[id]->data) - c792MemOffset;
  retVal = vmeDmaSend((unsigned long)laddr, vmeAdr, (nwrds<<2));
  if(retVal < 0) {
    logMsg("c792ReadBlock: ERROR in DMA transfer Initialization 0x%x\n",retVal,0,0,0,0,0);
    C792UNLOCK;
    return(ERROR);
  }
  /* Wait until Done or Error */
  retVal = vmeDmaDone();

#endif

  if(retVal != 0) {
    /* Check to see if error was generated by QDC */
    reg = vmeRead16(&c792p[id]->bitSet1);
    stat = reg & C792_VME_BUS_ERROR;
    if((retVal>0) && (stat)) {
      vmeWrite16(&c792p[id]->bitClear1, C792_VME_BUS_ERROR);
      /*logMsg("c792ReadBlock: INFO: DMA terminated by QDC - Transfer OK\n",0,0,0,0,0,0); */
#ifdef VXWORKS
      xferCount = (nwrds - (retVal>>2) + dummy);  /* Number of Longwords transfered */
#else
      xferCount = (retVal>>2) + dummy;  /* Number of Longwords transfered */
#endif

      /* Work backwards until the EOB is found */
      int iword = xferCount -1, ieob = -1;
      while(iword >= 0)
	{
	  trailer = data[iword];
#ifndef VXWORKS
	  trailer = LSWAP(trailer);
#endif
	  if ((trailer&C792_DATA_ID_MASK) == C792_TRAILER_DATA) {
	    ieob = iword;
	    break;
	  }
	  iword--;
	}

      if(ieob == -1)
	{
	  logMsg("%s(%d): ERROR: Failed to find EOB (xferCount = %d)\n",
		 __func__, id, xferCount);
	  C792UNLOCK;
	  return(xferCount); // FIXME: return ERROR;
	}

      xferCount = iword + 1;
      if ((trailer&C792_DATA_ID_MASK) == C792_TRAILER_DATA) {
	evID = trailer&C792_EVENTCOUNT_MASK;
	C792_EXEC_SET_EVTREADCNT(id,evID);
	C792UNLOCK;
	return(xferCount); /* Return number of data words transfered */
      } else {
	logMsg("%s(%d): ERROR: Invalid Trailer data 0x%x\n",
	       __func__, id,trailer);
	C792UNLOCK;
	/* return(ERROR); */
	return(xferCount);
      }
    } else {
      logMsg("%s(%d): ERROR in DMA transfer retVal = 0x%x stat = %d reg = 0x%x\n",
	     __func__,id,retVal,stat, reg);
      C792UNLOCK;
      return(ERROR);
    }
  }

  C792UNLOCK;
  return(OK);

}


/*******************************************************************************
*
* c792Int - default interrupt handler
*
* This rountine handles the c792 QDC interrupt.  A user routine is
* called, if one was connected by c792IntConnect().
*
* RETURNS: N/A
*
*/

LOCAL void
c792Int (void)
{
  int ii=0;
  UINT32 nevt1=0;
  UINT32 nevt2=0;

  /* Disable interrupts */
#ifdef VXWORKS
  sysIntDisable(c792IntLevel);
#endif


#ifdef VXWORKSPPC
  fppSave(&c792Fpr);
#endif

  c792IntCount++;

#ifndef VXWORKS
  vmeBusLock();
#endif

  if (c792IntRoutine != NULL)  {     /* call user routine */
    (*c792IntRoutine) (c792IntArg);
  }else{
    if((c792IntID<0) || (c792p[c792IntID] == NULL)) {
      logMsg("c792Int: ERROR : QDC id %d not initialized \n",c792IntID,0,0,0,0,0);
      return;
    }
    /* Default action is to increment the Read pointer by
       the number of events in the Event Trigger register
       or until the Data buffer is empty. The later case would
       indicate a possible error. In either case the data is
       effectively thrown away */
    C792LOCK;
    nevt1 = vmeRead16(&c792p[c792IntID]->evTrigger)&C792_EVTRIGGER_MASK;
    C792UNLOCK;
    nevt2 = c792Dready(c792IntID);
    if(nevt2<nevt1) {
      logMsg("c792Int: ERROR: Event Trig Register(%d) < # Events Ready (%d)\n",
             nevt1,nevt2,0,0,0,0);
      c792Clear(c792IntID);
    } else {
      C792LOCK;
      for(ii=0;ii<nevt1;ii++) {
	C792_EXEC_INCR_EVENT(c792IntID);
      }
      C792UNLOCK;
    }

    /* logMsg("c792Int: Processed %d events\n",nevt,0,0,0,0,0); */
  }

#ifndef VXWORKS
  vmeBusUnlock();
#endif

#ifdef VXWORKSPPC
  fppRestore(&c792Fpr);
#endif

  /* Enable interrupts */
#ifdef VXWORKS
  sysIntEnable(c792IntLevel);
#endif

}


/*******************************************************************************
*
* c792IntConnect - connect a user routine to the c792 QDC interrupt
*
* This routine specifies the user interrupt routine to be called at each
* interrupt.
*
* RETURNS: OK, or ERROR if Interrupts are enabled
*/

STATUS
c792IntConnect (VOIDFUNCPTR routine, int arg, UINT16 level, UINT16 vector)
{

  if(c792IntRunning) {
    printf("c792IntConnect: ERROR : Interrupts already Initialized for QDC id %d\n",
	   c792IntID);
    return(ERROR);
  }

  c792IntRoutine = routine;
  c792IntArg = arg;

  /* Check for user defined VME interrupt level and vector */
  if(level == 0) {
    c792IntLevel = C792_VME_INT_LEVEL; /* use default */
  }else if (level > 7) {
    printf("c792IntConnect: ERROR: Invalid VME interrupt level (%d). Must be (1-7)\n",level);
    return(ERROR);
  } else {
    c792IntLevel = level;
  }

  if(vector == 0) {
    c792IntVec = C792_INT_VEC;  /* use default */
  }else if ((vector < 32)||(vector>255)) {
    printf("c792IntConnect: ERROR: Invalid interrupt vector (%d). Must be (32<vector<255)\n",vector);
    return(ERROR);
  }else{
    c792IntVec = vector;
  }

  /* Connect the ISR */
#ifdef VXWORKSPPC
  if((intDisconnect((int)INUM_TO_IVEC(c792IntVec)) != 0)) {
    printf("c792IntConnect: ERROR disconnecting Interrupt\n");
    return(ERROR);
  }
#endif
#ifdef VXWORKS
  if((intConnect(INUM_TO_IVEC(c792IntVec),c792Int,0)) != 0) {
    printf("c792IntConnect: ERROR in intConnect()\n");
    return(ERROR);
  }
#else
  if(vmeIntDisconnect(c792IntLevel) != 0)
    {
      printf("c792IntConnect: ERROR disconnecting Interrupt\n");
      return(ERROR);
    }
  if(vmeIntConnect(c792IntVec,c792IntLevel,c792Int,0) != 0)
    {
      printf("c792IntConnect: ERROR in intConnect()\n");
      return(ERROR);
    }
#endif

  return (OK);
}


/*******************************************************************************
*
* c792IntEnable - Enable interrupts from specified QDC
*
* Enables interrupts for a specified QDC.
*
* RETURNS OK or ERROR if QDC is not available or parameter is out of range
*/

STATUS
c792IntEnable (int id, UINT16 evCnt)
{

  if(c792IntRunning) {
    printf("c792IntEnable: ERROR : Interrupts already initialized for QDC id %d\n",
	   c792IntID);
    return(ERROR);
  }

  if((id<0) || (c792p[id] == NULL)) {
    printf("c792IntEnable: ERROR : QDC id %d not initialized \n",id);
    return(ERROR);
  }else{
    c792IntID = id;
  }

  /* check for event count out of range */
  if((evCnt<=0) || (evCnt>31)) {
    printf("c792IntEnable: ERROR: Event count %d for Interrupt is out of range (1-31)\n"
	   ,evCnt);
    return(ERROR);
  }

#ifdef VXWORKS
  sysIntEnable(c792IntLevel);   /* Enable VME interrupts */
#endif

  /* Zero Counter and set Running Flag */
  c792IntEvCount = evCnt;
  c792IntCount = 0;
  c792IntRunning = TRUE;
  /* Enable interrupts on QDC */
  C792LOCK;
  vmeWrite16(&c792p[c792IntID]->intVector, c792IntVec);
  vmeWrite16(&c792p[c792IntID]->intLevel, c792IntLevel);
  vmeWrite16(&c792p[c792IntID]->evTrigger, c792IntEvCount);
  C792UNLOCK;

  return(OK);
}


/*******************************************************************************
*
* c792IntDisable - disable the QDC interrupts
*
* RETURNS: OK, or ERROR if not initialized
*/

STATUS
c792IntDisable (int iflag)
{

  if((c792IntID<0) || (c792p[c792IntID] == NULL)) {
    logMsg("c792IntDisable: ERROR : QDC id %d not initialized \n",c792IntID,0,0,0,0,0);
    return(ERROR);
  }

#ifdef VXWORKS
  sysIntDisable(c792IntLevel);   /* Disable VME interrupts */
#endif
  C792LOCK;
  vmeWrite16(&c792p[c792IntID]->evTrigger, 0);

  /* Tell tasks that Interrupts have been disabled */
  if(iflag > 0)
    {
      c792IntRunning = FALSE;
      vmeWrite16(&c792p[c792IntID]->intLevel, 0);
      vmeWrite16(&c792p[c792IntID]->intVector, 0);
    }
#ifdef VXWORKS
  else
    {
      semGive(c792Sem);
    }
#endif
  C792UNLOCK;

  return (OK);
}

/*******************************************************************************
*
* c792IntResume - Re-enable interrupts from previously
*                 intitialized QDC
*
* RETURNS: OK, or ERROR if not initialized
*/

STATUS
c792IntResume (void)
{
  UINT16 evTrig = 0;

  if((c792IntID<0) || (c792p[c792IntID] == NULL)) {
    logMsg("c792IntResume: ERROR : QDC id %d not initialized \n",c792IntID,0,0,0,0,0);
    return(ERROR);
  }

  if ((c792IntRunning)) {
    C792LOCK;
    evTrig = vmeRead16(&c792p[c792IntID]->evTrigger)&C792_EVTRIGGER_MASK;
    if (evTrig == 0) {
#ifdef VXWORKS
      sysIntEnable(c792IntLevel);
#endif
      vmeWrite16(&c792p[c792IntID]->evTrigger, c792IntEvCount);
    } else {
      logMsg("c792IntResume: WARNING : Interrupts already enabled \n",0,0,0,0,0,0);
      C792UNLOCK;
      return(ERROR);
    }
    C792UNLOCK;
  } else {
      logMsg("c792IntResume: ERROR : Interrupts are not Enabled \n",0,0,0,0,0,0);
      return(ERROR);
  }

  return (OK);
}



/*******************************************************************************
*
* c792Sparse - Enable/Disable Overflow and Under threshold sparsification
*
*
* RETURNS: Bit Set 2 Register value.
*/

UINT16
c792Sparse(int id, int over, int under)
{
  UINT16 rval;

  if((id<0) || (c792p[id] == NULL)) {
    printf("c792Sparse: ERROR : QDC id %d not initialized \n",id);
    return(0xffff);
  }

  C792LOCK;
  if(!over) {  /* Set Overflow suppression */
    vmeWrite16(&c792p[id]->bitSet2, C792_OVERFLOW_SUP);
  }else{
    vmeWrite16(&c792p[id]->bitClear2, C792_OVERFLOW_SUP);
  }

  if(!under) {  /* Set Underflow suppression */
    vmeWrite16(&c792p[id]->bitSet2, C792_UNDERFLOW_SUP);
  }else{
    vmeWrite16(&c792p[id]->bitClear2, C792_UNDERFLOW_SUP);
  }


  rval = vmeRead16(&c792p[id]->bitSet2)&C792_BITSET2_MASK;
  C792UNLOCK;

  return(rval);
}


/*******************************************************************************
*
* c792Dready - Return status of Data Ready bit in QDC
*
*
* RETURNS: 0(No Data) or  # of events in FIFO (1-32) or ERROR.
*/

int
c792Dready(int id)
{

  int nevts = 0;
  UINT16 stat=0;


  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Dready: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return (ERROR);
  }

  C792LOCK;
  stat = vmeRead16(&c792p[id]->status1)&C792_DATA_READY;
  if(stat) {
    C792_EXEC_READ_EVENT_COUNT(id);
    C792UNLOCK;
    nevts = c792EventCount[id] - c792EvtReadCnt[id];
    if(nevts <= 0) {
      logMsg("c792Dready: ERROR : Bad Event Ready Count (nevts = %d)\n",
	     nevts,0,0,0,0,0);
      return(ERROR);
    }
  }
  C792UNLOCK;

  return(nevts);
}

unsigned int
c792GDReady(unsigned int idmask, int nloop)
{
  int iloop, id, stat=0;
  unsigned int dmask=0;

  C792LOCK;
  for(iloop = 0; iloop < nloop; iloop++)
    {
      for(id=0; id<Nc792; id++)
	{
	  if(idmask & (1<<id))
	    { /* id used */

	      if(!(dmask & (1<<id)))
		{ /* No data ready yet. Check it now. */
		  stat = vmeRead16(&c792p[id]->status1)&C792_DATA_READY;

		  if(stat)
		    dmask |= (1<<id);

		  if(dmask == idmask)
		    { /* Blockready mask matches user idmask */
		      C792UNLOCK;
		      return(dmask);
		    }
		}
	    }
	}
    }
  C792UNLOCK;

  return(dmask);
}

/*******************************************************************************
*
* c792ClearThresh  - Zero QDC thresholds for all channels
* c792Gate         - Issue Software Gate to QDC
* c792Csr2         - Program CSR2 register
* c792EnableBerr   - Enable Bus Error to finish a block transfer
* c792DisableBerr  - Disable Bus Error
* c792IncrEventBlk - Increment Event counter for Block reads
* c792IncrEvent    - Increment Read pointer to next event in the Buffer
* c792IncrWord     - Increment Read pointer to next word in the event
* c792Enable       - Bring QDC Online (Enable Gates)
* c792Disable      - Bring QDC Offline (Disable Gates)
* c792Clear        - Clear QDC
* c792Reset        - Clear/Reset QDC
*
*
* RETURNS: None.
*/

void
c792ClearThresh(int id)
{
  int ii;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792ClearThresh: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }

  C792LOCK;
  for (ii=0;ii< C792_MAX_CHANNELS; ii++) {
    vmeWrite16(&c792p[id]->threshold[ii], 0);
  }
  C792UNLOCK;
}

short
c792SetThresh(int id, int chan, short val)
{
  short rval;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792SetThresh: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(-1);
  }

  if((chan<0) || (chan>(C792_MAX_CHANNELS-1))) {
    logMsg("c792SetThresh: channel id %d - out of range (0-31) \n",chan,0,0,0,0,0);
    return (-1);
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->threshold[chan], val);
  rval = vmeRead16(&c792p[id]->threshold[chan]);
  C792UNLOCK;

  return (rval);
}


void
c792Gate(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Gate: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_GATE(id);
  C792UNLOCK;
}

short
c792Control(int id, short val)
{
  short rval;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Control: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(-1);
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->control1, val);
  rval = vmeRead16(&c792p[id]->control1);
  C792UNLOCK;

  return (rval);
}

short
c792BitSet2(int id, short val)
{
  short rval;

  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792BitSet2: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return(-1);
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->bitSet2, val);
  rval = vmeRead16(&c792p[id]->bitSet2);
  C792UNLOCK;

  return (rval);
}

void
c792BitClear2(int id, short val)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792BitClear2: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->bitClear2, val);
  C792UNLOCK;
}

void
c792EnableBerr(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("%s: ERROR : QDC id %d not initialized \n",__FUNCTION__,id,0,0,0,0);
    return;
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->control1,
	    vmeRead16(&c792p[id]->control1) |
	     C792_BERR_ENABLE | C792_BLK_END | C792_ALIGN64);
  C792UNLOCK;
}

void
c792DisableBerr(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("%s: ERROR : QDC id %d not initialized \n",__FUNCTION__,id,0,0,0,0);
    return;
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->control1,
	    vmeRead16(&c792p[id]->control1) & ~(C792_BERR_ENABLE | C792_BLK_END));
  C792UNLOCK;
}

void
c792IncrEventBlk(int id, int count)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792IncrEventBlk: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }

  if((count > 0) && (count <=32))
    c792EvtReadCnt[id] += count;
}

void
c792IncrEvent(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792IncrEvent: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_INCR_EVENT(id);
  C792UNLOCK;
}

void
c792IncrWord(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792IncrWord: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_INCR_WORD(id);
  C792UNLOCK;
}

void
c792Enable(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Enable: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  vmeWrite16(&c792p[id]->bitClear2, C792_OFFLINE);
  C792UNLOCK;
}

void
c792Disable(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Disable: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  vmeWrite16(&c792p[id]->bitSet2, C792_OFFLINE);
  C792UNLOCK;
}


void
c792Clear(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Clear: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_DATA_RESET(id);
  C792UNLOCK;
  c792EvtReadCnt[id] = -1;
  c792EventCount[id] =  0;

}

void
c792Reset(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Reset: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_DATA_RESET(id);
  C792_EXEC_SOFT_RESET(id);
  C792_EXEC_CLR_EVENT_COUNT(id);
  C792UNLOCK;
  c792EvtReadCnt[id] = -1;
  c792EventCount[id] =  0;
}

void
c792EventCounterReset(int id)
{
  if((id<0) || (c792p[id] == NULL)) {
    logMsg("c792Reset: ERROR : QDC id %d not initialized \n",id,0,0,0,0,0);
    return;
  }
  C792LOCK;
  C792_EXEC_CLR_EVENT_COUNT(id);
  C792UNLOCK;
  c792EvtReadCnt[id] = -1;
  c792EventCount[id] =  0;
}

int
c792SetGeoAddress(int id, int geo)
{
  if((id<0) || (c792p[id] == NULL)) {
    printf("%s: ERROR : QDC id %d not initialized \n",
	   __func__, id);
    return ERROR;
  }

  C792LOCK;
  vmeWrite16(&c792p[id]->geoAddr, geo);
  C792_EXEC_SOFT_RESET(id);
  C792UNLOCK;

  return OK;
}
