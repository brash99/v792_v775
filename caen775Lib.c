/******************************************************************************
*
*  caen775Lib.c  -  Driver library for readout of C.A.E.N. Model 775 TDC
*                   using a VxWorks 5.2 or later based Single Board computer.
*
*  Author: David Abbott
*          Jefferson Lab Data Acquisition Group
*          March 2002
*
*  Revision  1.0 - Initial Revision
*                    - Supports up to 20 CAEN Model 775s in a Crate
*                    - Programmed I/O reads
*                    - Interrupts from a Single 775
*
*/

#include <stdio.h>
#include <string.h>
#include <pthread.h>
#ifdef VXWORKS
#include "vxWorks.h"
#include "logLib.h"
#include "taskLib.h"
#include "intLib.h"
#include "iv.h"
#include "semLib.h"
#include "vxLib.h"
#endif
#include "jvme.h"

/* Include TDC definitions */
#include "c775Lib.h"

#ifdef VXWORKS
/* Define external Functions */
IMPORT STATUS sysBusToLocalAdrs(int, char *, char **);
IMPORT STATUS intDisconnect(int);
IMPORT STATUS sysIntEnable(int);
IMPORT STATUS sysIntDisable(int);
#endif

/* Mutex to guard c775 reads/writes */
pthread_mutex_t c775mutex = PTHREAD_MUTEX_INITIALIZER;
#define C775LOCK   if(pthread_mutex_lock(&c775mutex)<0) perror("pthread_mutex_lock");
#define C775UNLOCK if(pthread_mutex_unlock(&c775mutex)<0) perror("pthread_mutex_unlock");

/* Define Interrupts variables */
BOOL c775IntRunning = FALSE;	/* running flag */
int c775IntID = -1;		/* id number of TDC generating interrupts */
LOCAL VOIDFUNCPTR c775IntRoutine = NULL;	/* user interrupt service routine */
LOCAL int c775IntArg = 0;	/* arg to user routine */
LOCAL int c775IntEvCount = 0;	/* Number of Events to generate Interrupt */
LOCAL UINT32 c775IntLevel = C775_VME_INT_LEVEL;	/* default VME interrupt level */
LOCAL UINT32 c775IntVec = C775_INT_VEC;	/* default interrupt Vector */


/* Define global variables */
int Nc775 = 0;			/* Number of TDCs in Crate */
volatile c775_regs *c775p[20];	/* pointers to TDC memory map */
volatile c775_regs *c775pl[20];	/* Support for 68K second memory map A24/D32 */
int c775IntCount = 0;		/* Count of interrupts from TDC */
int c775EventCount[20];		/* Count of Events taken by TDC (Event Count Register value) */
int c775EvtReadCnt[20];		/* Count of events read from specified TDC */
unsigned int c775MemOffset = 0;	/* CPUs A24 or A32 address space offset */

#ifdef VXWORKS
SEM_ID c775Sem;			/* Semephore for Task syncronization */
#endif

/* Macros */
#define C775_EXEC_SOFT_RESET(id) {					\
    vmeWrite16(&c775p[id]->main.bitSet1, C775_SOFT_RESET);			\
    vmeWrite16(&c775p[id]->main.bitClear1, C775_SOFT_RESET);}

#define C775_EXEC_DATA_RESET(id) {					\
    vmeWrite16(&c775p[id]->main.bitSet2, C775_DATA_RESET);			\
    vmeWrite16(&c775p[id]->main.bitClear2, C775_DATA_RESET);}

#define C775_EXEC_READ_EVENT_COUNT(id) {				\
    volatile unsigned short s1, s2;					\
    s1 = vmeRead16(&c775p[id]->main.evCountL);				\
    s2 = vmeRead16(&c775p[id]->main.evCountH);				\
    c775EventCount[id] = (c775EventCount[id]&0xff000000) +		\
      (s2<<16) +							\
      (s1);}
#define C775_EXEC_SET_EVTREADCNT(id,val) {				\
    if(c775EvtReadCnt[id] < 0)						\
      c775EvtReadCnt[id] = val;						\
    else								\
      c775EvtReadCnt[id] = (c775EvtReadCnt[id]&0x7f000000) + val;}

#define C775_EXEC_CLR_EVENT_COUNT(id) {		\
    vmeWrite16(&c775p[id]->main.evCountReset, 1);	\
    c775EventCount[id] = 0;}
#define C775_EXEC_INCR_EVENT(id) {			\
    vmeWrite16(&c775p[id]->main.incrEvent, 1);		\
    c775EvtReadCnt[id]++;}
#define C775_EXEC_INCR_WORD(id) {		\
    vmeWrite16(&c775p[id]->main.incrOffset, 1);}
#define C775_EXEC_GATE(id) {			\
    vmeWrite16(&c775p[id]->main.swComm, 1);}


/*******************************************************************************
*
* c775Init - Initialize c775 Library.
*
*
* RETURNS: OK, or ERROR if the address is invalid or board is not present.
*/

STATUS
c775Init(UINT32 addr, UINT32 addr_inc, int ntdc, UINT16 crateID)
{
  int ii, res, rdata, errFlag = 0;
  int boardID = 0;
  unsigned long laddr, lladdr;
  volatile c775_ROM *rp;


  /* Check for valid address */
  if (addr == 0)
    {
      printf
	("c775Init: ERROR: Must specify a Bus (VME-based A32/A24) address for TDC 0\n");
      return (ERROR);
    }
  else if (addr < 0x00ffffff)
    {				/* A24 Addressing */
      if ((addr_inc == 0) || (ntdc == 0))
	ntdc = 1;		/* assume only one TDC to initialize */

      /* get the TDCs address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x39, (char *) addr, (char **) &laddr);
      if (res != 0)
	{
	  printf("c775Init: ERROR in sysBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 addr);
	  return (ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x39, (char *)(unsigned long) addr,
			      (char **)(unsigned long) &laddr);
      if (res != 0)
	{
	  printf("c775Init: ERROR in vmeBusToLocalAdrs(0x39,0x%x,&laddr) \n",
		 addr);
	  return (ERROR);
	}
      c775MemOffset = laddr - addr;
#endif
    }
  else
    {				/* A32 Addressing */

#ifdef VXWORKS68K51
      printf
	("c775Init: ERROR: 68K Based CPU cannot support A32 addressing (use A24)\n");
      return (ERROR);
#endif

      if ((addr_inc == 0) || (ntdc == 0))
	ntdc = 1;		/* assume only one TDC to initialize */

      /* get the TDC address */
#ifdef VXWORKS
      res = sysBusToLocalAdrs(0x09, (char *) addr, (char **) &laddr);
      if (res != 0)
	{
	  printf("c775Init: ERROR in sysBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 addr);
	  return (ERROR);
	}
#else
      res = vmeBusToLocalAdrs(0x09, (char *) (unsigned long)addr,
			      (char **) (unsigned long)&laddr);
      if (res != 0)
	{
	  printf("c775Init: ERROR in vmeBusToLocalAdrs(0x09,0x%x,&laddr) \n",
		 addr);
	  return (ERROR);
	}
#endif
      c775MemOffset = laddr - addr;
    }

  /* Put in Hack for 68K seperate address spaces for A24/D16 and A24/D32 */
  /* for PowerPC they are one and the same */
#ifdef VXWORKS68K51
  lladdr = C775_68K_A24D32_OFFSET + (laddr & 0x00ffffff);
#else
  lladdr = laddr;
#endif


  Nc775 = 0;
  for (ii = 0; ii < ntdc; ii++)
    {
      c775p[ii] = (c775_regs *) (laddr + ii * addr_inc);
      c775pl[ii] = (c775_regs *) (lladdr + ii * addr_inc);
      /* Check if Board exists at that address */
#ifdef VXWORKS
      res = vxMemProbe((char *) &(c775p[ii]->main.rev), 0, 2, (char *) &rdata);
#else
      res = vmeMemProbe((char *) &(c775p[ii]->main.rev), 2, (char *) &rdata);
#endif
      if (res < 0)
	{
	  printf("c775Init: ERROR: No addressable board at addr=0x%x\n",
		 (unsigned long) c775p[ii] - c775MemOffset);
	  c775p[ii] = NULL;
	  errFlag = 1;
	  break;
	}
      else
	{
	  /* Check if this is a Model 775 */
	  rp = (c775_ROM *) ((unsigned long) &c775p[ii]->rom);
	  boardID =
	    ((vmeRead16(&rp->ID_3) & (0xff)) << 16) +
	    ((vmeRead16(&rp->ID_2) & (0xff)) << 8) +
	    (vmeRead16(&rp->ID_1) & (0xff));
	  if ((boardID != C775_BOARD_ID) && (boardID != C775_BOARD_ID))
	    {
	      printf("c775Init: ERROR: Board ID does not match: %d \n",
		     boardID);
	      return (ERROR);
	    }
	}
      Nc775++;
#ifdef VXWORKS
      printf("Initialized TDC ID %d at address 0x%08x \n", ii,
	     (UINT32) c775p[ii]);
#else
      printf("Initialized TDC ID %d at VME (LOCAL) address 0x%x (0x%x)\n", ii,
	     (unsigned long) c775p[ii] - c775MemOffset, (unsigned long) c775p[ii]);
#endif
    }

#ifdef VXWORKS
  /* Initialize/Create Semephore */
  if (c775Sem != 0)
    {
      semFlush(c775Sem);
      semDelete(c775Sem);
    }
  c775Sem = semBCreate(SEM_Q_PRIORITY, SEM_EMPTY);
  if (c775Sem <= 0)
    {
      printf("c775Init: ERROR: Unable to create Binary Semephore\n");
      return (ERROR);
    }
#endif

  /* Disable/Clear all TDCs */
  for (ii = 0; ii < Nc775; ii++)
    {
      C775_EXEC_SOFT_RESET(ii);
      C775_EXEC_DATA_RESET(ii);
      /* Disable Interrupts */
      vmeWrite16(&c775p[ii]->main.intLevel, 0);
      /* Zero interrupt trigger count */
      vmeWrite16(&c775p[ii]->main.evTrigger, 0);
      /* Set Crate ID Register */
      vmeWrite16(&c775p[ii]->main.crateSelect, crateID);
      /* Increment event count only on accepted gates */
      vmeWrite16(&c775p[ii]->main.bitClear2, C775_INCR_ALL_TRIG);
      /* Turn off suppression of header and EOB if no accepted channels */
      vmeWrite16(&c775p[ii]->main.bitClear2, C775_INC_HEADER);

      c775EventCount[ii] = 0;	/* Initialize the Event Count */
      c775EvtReadCnt[ii] = -1;	/* Initialize the Read Count */

      c775SetFSR(ii, C775_MIN_FSR);	/* Set Full Scale Range for TDC */

      c775Sparse(ii, 0, 0);	/* Disable Overflow/Underflow suppression */
    }
  /* Initialize Interrupt variables */
  c775IntID = -1;
  c775IntRunning = FALSE;
  c775IntLevel = 0;
  c775IntVec = 0;
  c775IntRoutine = NULL;
  c775IntArg = 0;
  c775IntEvCount = 0;


  if (errFlag > 0)
    {
      printf("c775Init: ERROR: Unable to initialize all TDC Modules\n");
      if (Nc775 > 0)
	printf("c7752Init: %d TDC(s) successfully initialized\n", Nc775);
      return (ERROR);
    }
  else
    {
      return (OK);
    }
}

/*******************************************************************************
*
* c775Status - Gives Status info on specified TDC
*
*
* RETURNS: None
*/

void
c775Status(int id)
{

  int DRdy = 0, BufFull = 0;
  UINT16 stat1, stat2, bit1, bit2, cntl1, rev;
  UINT16 iLvl, iVec, evTrig;
  UINT16 fsr;

  if ((id < 0) || (c775p[id] == NULL))
    {
      printf("c775Status: ERROR : TDC id %d not initialized \n", id);
      return;
    }


  /* read various registers */
  C775LOCK;
  rev = vmeRead16(&c775p[id]->main.rev);
  stat1 = vmeRead16(&c775p[id]->main.status1) & C775_STATUS1_MASK;
  stat2 = vmeRead16(&c775p[id]->main.status2) & C775_STATUS2_MASK;
  bit1 = vmeRead16(&c775p[id]->main.bitSet1) & C775_BITSET1_MASK;
  bit2 = vmeRead16(&c775p[id]->main.bitSet2) & C775_BITSET2_MASK;
  cntl1 = vmeRead16(&c775p[id]->main.control1) & C775_CONTROL1_MASK;
  fsr = 4 * (290 - (vmeRead16(&c775p[id]->main.fsr) & C775_FSR_MASK));
  C775_EXEC_READ_EVENT_COUNT(id);
  if (stat1 & C775_DATA_READY)
    DRdy = 1;
  if (stat2 & C775_BUFFER_FULL)
    BufFull = 1;

  iLvl = vmeRead16(&c775p[id]->main.intLevel) & C775_INTLEVEL_MASK;
  iVec = vmeRead16(&c775p[id]->main.intVector) & C775_INTVECTOR_MASK;
  evTrig = vmeRead16(&c775p[id]->main.evTrigger) & C775_EVTRIGGER_MASK;
  C775UNLOCK;

  /* print out status info */

#ifdef VXWORKS
  printf("STATUS for TDC id %d at base address 0x%x \n", id,
	 (UINT32) c775p[id]);
#else
  printf("STATUS for TDC id %d at VME (LOCAL) base address 0x%x (0x%x) \n", id,
	 (unsigned long) c775p[id] - c775MemOffset, (unsigned long) c775p[id]);
#endif
  printf("--------------------------------------------------------------------------------\n");
  printf(" Firmware Revision = %d.%d\n", rev >> 8, rev & 0xff);

  if ((iLvl > 0) && (evTrig > 0))
    {
      printf(" Interrupts Enabled - Every %d events\n", evTrig);
      printf(" VME Interrupt Level: %d   Vector: 0x%x \n", iLvl, iVec);
      printf(" Interrupt Count    : %d \n", c775IntCount);
    }
  else
    {
      printf(" Interrupts Disabled\n");
      printf(" Last Interrupt Count    : %d \n", c775IntCount);
    }
  printf("\n");

  printf("             --1--                  --2--\n");
  printf("  Status:    0x%04x                 0x%04x\n", stat1, stat2);
  printf("    Data Ready = %d      Buffer Empty = %d\n",
	 (stat1 & C775_DATA_READY)?1:0, (stat2 & C775_BUFFER_EMPTY)?1:0);
  printf("          Busy = %d       Buffer Full = %d\n",
	 (stat1 & C775_BUSY)?1:0, (stat2 & C775_BUFFER_FULL)?1:0);
  printf("       EvReady = %d\n",
	 (stat1 & C775_EVRDY)?1:0);
  printf("\n");

  printf("  BitSet:    0x%04x                 0x%04x\n", bit1, bit2);
  printf("          BERR = %d        Over Range = %d\n",
	 (bit1 & C775_VME_BUS_ERROR)?1:0, (bit2 & C775_OVER_RANGE)?1:0);
  printf("      Sel Addr = %d     Low Threshold = %d\n",
	 (bit1 & 0x10)?1:0, (bit2 & C775_LOW_THRESHOLD)?1:0);
  printf("    Soft Reset = %d       Common Stop = %d\n",
	 (bit1 & 0x80)?1:0, (bit2 & C775_COMMON_STOP)?1:0);
  printf("                         Slide Enable = %d\n",
	 (bit2 & C775_SLIDE_ENABLE)?1:0);
  printf("                            Auto Incr = %d\n",
	 (bit2 & C775_AUTO_INCR)?1:0);
  printf("                           Empty Prog = %d\n",
	 (bit2 & C775_INC_HEADER)?1:0);
  printf("                     Slide Sub Enable = %d\n",
	 (bit2 & C775_SLIDE_SUB_ENABLE)?1:0);
  printf("                              All Trg = %d\n",
	 (bit2 & C775_INCR_ALL_TRIG)?1:0);
  printf("\n");

  printf("  Control:   0x%04x\n", cntl1);
  printf("       Blk End = %d\n", cntl1 & C775_BLK_END);
  printf("   BERR Enable = %d\n", cntl1 & C775_BERR_ENABLE);
  printf("      Align 64 = %d\n", cntl1 & C775_ALIGN64);
  printf("\n");

  printf("  FSR     = %d nsec\n", fsr);
  if (c775EventCount[id] == 0xffffff)
    {
      printf("  Event Count     = (No Events Taken)\n");
      printf("  Last Event Read = (No Events Read)\n");
    }
  else
    {
      printf("  Event Count     = %d\n", c775EventCount[id]);
      if (c775EvtReadCnt[id] == -1)
	printf("  Last Event Read = (No Events Read)\n");
      else
	printf("  Last Event Read = %d\n", c775EvtReadCnt[id]);
    }

  printf("--------------------------------------------------------------------------------\n");
  printf("\n\n");

}

/*******************************************************************************
*
* c775PrintEvent - Print event from TDC to standard out.
*
*
* RETURNS: Number of Data words read from the TDC (including Header/Trailer).
*/

int
c775PrintEvent(int id, int pflag)
{

  int ii, nWords, evID;
  UINT32 header, trailer, dCnt;

  if ((id < 0) || (c775p[id] == NULL))
    {
      printf("c775ClearThresh: ERROR : TDC id %d not initialized \n", id);
      return (-1);
    }

  /* Check if there is a valid event */

  C775LOCK;
  if (vmeRead16(&c775p[id]->main.status2) & C775_BUFFER_EMPTY)
    {
      printf("c775PrintEvent: Data Buffer is EMPTY!\n");
      C775UNLOCK;
      return (0);
    }
  if (vmeRead16(&c775p[id]->main.status1) & C775_DATA_READY)
    {
      dCnt = 0;
      /* Read Header - Get Word count */
      header = vmeRead32(&c775pl[id]->data[0]);
      if ((header & C775_DATA_ID_MASK) != C775_HEADER_DATA)
	{
	  printf("c775PrintEvent: ERROR: Invalid Header Word 0x%08x\n",
		 header);
	  C775UNLOCK;
	  return (-1);
	}
      else
	{
	  printf("  TDC DATA for Module %d\n", id);
	  nWords = (header & C775_WORDCOUNT_MASK) >> 8;
	  dCnt++;
	  printf("  Header: 0x%08x   nWords = %d ", header, nWords);
	}
      for (ii = 0; ii < nWords; ii++)
	{
	  if ((ii % 5) == 0)
	    printf("\n    ");
	  printf("  0x%08x", (UINT32) vmeRead32(&c775pl[id]->data[ii + 1]));
	}
      printf("\n");
      dCnt += ii;

      trailer = vmeRead32(&c775pl[id]->data[dCnt]);
      if ((trailer & C775_DATA_ID_MASK) != C775_TRAILER_DATA)
	{
	  printf("c775PrintEvent: ERROR: Invalid Trailer Word 0x%08x\n",
		 trailer);
	  C775UNLOCK;
	  return (-1);
	}
      else
	{
	  evID = trailer & C775_EVENTCOUNT_MASK;
	  dCnt++;
	  printf("  Trailer: 0x%08x   Event Count = %d \n", trailer, evID);
	}
      C775_EXEC_SET_EVTREADCNT(id, evID);
      C775UNLOCK;
      return (dCnt);

    }
  else
    {
      printf("c775PrintEvent: Data Not ready for readout!\n");
      C775UNLOCK;
      return (0);
    }
}


/*******************************************************************************
*
* c775ReadEvent - Read event from TDC to specified address.
*
*
*
* RETURNS: Number of Data words read from the TDC (including Header/Trailer).
*/

int
c775ReadEvent(int id, UINT32 * data)
{

  int ii, nWords, evID;
  UINT32 header, trailer, dCnt;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775ReadEvent: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (-1);
    }

  /* Check if there is a valid event */

  C775LOCK;
  if (vmeRead16(&c775p[id]->main.status2) & C775_BUFFER_EMPTY)
    {
      logMsg("c775ReadEvent: Data Buffer is EMPTY!\n", 0, 0, 0, 0, 0, 0);
      C775UNLOCK;
      return (0);
    }
  if (vmeRead16(&c775p[id]->main.status1) & C775_DATA_READY)
    {
      dCnt = 0;
      /* Read Header - Get Word count */
      header = c775pl[id]->data[dCnt];
#ifndef VXWORKS
      header = LSWAP(header);
#endif
      if ((header & C775_DATA_ID_MASK) != C775_HEADER_DATA)
	{
	  logMsg("c775ReadEvent: ERROR: Invalid Header Word 0x%08x\n", header,
		 0, 0, 0, 0, 0);
	  C775UNLOCK;
	  return (-1);
	}
      else
	{
	  nWords = (header & C775_WORDCOUNT_MASK) >> 8;
#ifndef VXWORKS
	  header = LSWAP(header);
#endif
	  data[dCnt] = header;
	  dCnt++;
	}
      for (ii = 0; ii < nWords; ii++)
	{
	  data[ii + 1] = c775pl[id]->data[ii + 1];
	}
      dCnt += ii;

      trailer = c775pl[id]->data[dCnt];
#ifndef VXWORKS
      trailer = LSWAP(trailer);
#endif
      if ((trailer & C775_DATA_ID_MASK) != C775_TRAILER_DATA)
	{
	  logMsg("c775ReadEvent: ERROR: Invalid Trailer Word 0x%08x\n",
		 trailer, 0, 0, 0, 0, 0);
	  C775UNLOCK;
	  return (-1);
	}
      else
	{
	  evID = trailer & C775_EVENTCOUNT_MASK;
#ifndef VXWORKS
	  trailer = LSWAP(trailer);
#endif
	  data[dCnt] = trailer;
	  dCnt++;
	}
      C775_EXEC_SET_EVTREADCNT(id, evID);
      C775UNLOCK;
      return (dCnt);

    }
  else
    {
      logMsg("c775ReadEvent: Data Not ready for readout!\n", 0, 0, 0, 0, 0,
	     0);
      C775UNLOCK;
      return (0);
    }
}

/*******************************************************************************
*
* c775FlushEvent - Flush event/data from TDC.
*
*
* RETURNS: Number of Data words read from the TDC.
*/

int
c775FlushEvent(int id, int fflag)
{

  int evID;
  int done = 0;
  UINT32 tmpData, dCnt;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775FlushEvent: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (-1);
    }

  /* Check if there is a valid event */

  C775LOCK;
  if (vmeRead16(&c775p[id]->main.status2) & C775_BUFFER_EMPTY)
    {
      if (fflag > 0)
	logMsg("c775FlushEvent: Data Buffer is EMPTY!\n", 0, 0, 0, 0, 0, 0);
      C775UNLOCK;
      return (0);
    }

  /* Check if Data Ready Flag is on */
  if (vmeRead16(&c775p[id]->main.status1) & C775_DATA_READY)
    {
      dCnt = 0;

      while (!done)
	{
	  tmpData = c775pl[id]->data[dCnt];
#ifndef VXWORKS
	  tmpData = LSWAP(tmpData);
#endif
	  switch (tmpData & C775_DATA_ID_MASK)
	    {
	    case C775_HEADER_DATA:
	      if (fflag > 0)
		logMsg("c775FlushEvent: Found Header 0x%08x\n", tmpData, 0, 0,
		       0, 0, 0);
	      break;
	    case C775_DATA:
	      break;
	    case C775_TRAILER_DATA:
	      if (fflag > 0)
		logMsg(" c775FlushEvent: Found Trailer 0x%08x\n", tmpData, 0,
		       0, 0, 0, 0);
	      evID = tmpData & C775_EVENTCOUNT_MASK;
	      C775_EXEC_SET_EVTREADCNT(id, evID);
	      done = 1;
	      break;
	    case C775_INVALID_DATA:
	      if (fflag > 0)
		logMsg(" c775FlushEvent: Buffer Empty 0x%08x\n", tmpData, 0,
		       0, 0, 0, 0);
	      done = 1;
	      break;
	    default:
	      if (fflag > 0)
		logMsg(" c775FlushEvent: Invalid Data 0x%08x\n", tmpData, 0,
		       0, 0, 0, 0);
	    }

	  /* Print out Data */
	  if (fflag > 1)
	    {
	      if ((dCnt % 5) == 0)
		printf("\n    ");
	      printf("  0x%08x ", tmpData);
	    }
	  dCnt++;
	}
      if (fflag > 1)
	printf("\n");
      C775UNLOCK;
      return (dCnt);

    }
  else
    {
      if (fflag > 0)
	logMsg("c775FlushEvent: Data Not ready for readout!\n", 0, 0, 0, 0, 0,
	       0);
      C775UNLOCK;
      return (0);
    }
}


/*******************************************************************************
*
* c775ReadBlock - Read Block of events from TDC to specified address.
*
* INPUTS:    id     - module id of TDC to access
*            data   - address of data destination
*            nwrds  - number of data words to transfer
*
* RETURNS: OK or ERROR on success of transfer.
*
* Note: User must call c775IncrEventBlk after a successful
*       call to c775ReadBlock to increment the number of events Read.
*         (e.g.   c775IncrEventBlk(0,15);
*/

int
c775ReadBlock(int id, volatile UINT32 * data, int nwrds)
{

  int retVal, xferCount;
  UINT32 vmeAdr, trailer, evID;
  UINT16 stat = 0;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775ReadBlock: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (-1);
    }

  C775LOCK;
#ifdef VXWORKSPPC
  /* Don't bother checking if there is a valid event. Just blast data out of the
     FIFO Valid or Invalid
     Also assume that the Universe DMA programming is setup. */

  retVal =
    sysVmeDmaSend((UINT32) data, (UINT32) (c775pl[id]->data), (nwrds << 2),
		  0);
  if (retVal < 0)
    {
      logMsg("c775ReadBlock: ERROR in DMA transfer Initialization 0x%x\n",
	     retVal, 0, 0, 0, 0, 0);
      return (retVal);
    }
  /* Wait until Done or Error */
  retVal = sysVmeDmaDone(1000, 1);

#elif defined(VXWORKS68K51)

  /* 68K Block 32 transfer from FIFO using VME2Chip */
  retVal = mvme_dma((long) data, 1, (long) (c775pl[id]->data), 0, nwrds, 1);

#else
  /* Linux readout with jvme library */
  vmeAdr = (unsigned long) c775p[id]->data - c775MemOffset;
  retVal = vmeDmaSend((unsigned long)data, vmeAdr, (nwrds << 2));
  if (retVal < 0)
    {
      logMsg("c775ReadBlock: ERROR in DMA transfer Initialization 0x%x\n",
	     retVal, 0, 0, 0, 0, 0);
      C775UNLOCK;
      return (ERROR);
    }
  /* Wait until Done or Error */
  retVal = vmeDmaDone();

#endif

  if (retVal != 0)
    {
      /* Check to see if error was generated by TDC */
      stat = vmeRead16(&c775p[id]->main.bitSet1) & C775_VME_BUS_ERROR;
      if ((retVal > 0) && (stat))
	{
	  vmeWrite16(&c775p[id]->main.bitClear1, C775_VME_BUS_ERROR);
/*       logMsg("c775ReadBlock: INFO: DMA terminated by TDC(BUS Error) - Transfer OK\n",0,0,0,0,0,0); */
#ifdef VXWORKS
	  xferCount = (nwrds - (retVal >> 2));	/* Number of Longwords transfered */
#else
	  xferCount = (retVal >> 2);	/* Number of Longwords transfered */
#endif
	  trailer = data[xferCount - 1];
#ifndef VXWORKS
	  trailer = LSWAP(trailer);
#endif
	  if ((trailer & C775_DATA_ID_MASK) == C775_TRAILER_DATA)
	    {
	      evID = trailer & C775_EVENTCOUNT_MASK;
	      C775_EXEC_SET_EVTREADCNT(id, evID);
	      C775UNLOCK;
	      return (xferCount);	/* Return number of data words transfered */
	    }
	  else
	    {
	      trailer = data[xferCount - 2];
#ifndef VXWORKS
	      trailer = LSWAP(trailer);
#endif
	      if ((trailer & C775_DATA_ID_MASK) == C775_TRAILER_DATA)
		{
		  evID = trailer & C775_EVENTCOUNT_MASK;
		  C775_EXEC_SET_EVTREADCNT(id, evID);
		  C775UNLOCK;
		  return (xferCount - 1);	/* Return number of data words transfered */
		}
	      else
		{
		  logMsg("c775ReadBlock: ERROR: Invalid Trailer data 0x%x\n",
			 trailer, 0, 0, 0, 0, 0);
		  C775UNLOCK;
		  return (xferCount);
		}
	    }
	}
      else
	{
	  logMsg("c775ReadBlock: ERROR in DMA transfer 0x%x\n", retVal, 0, 0,
		 0, 0, 0);
	  C775UNLOCK;
	  return (retVal);
	}
    }

  C775UNLOCK;
  return (OK);

}


/*******************************************************************************
*
* c775Int - default interrupt handler
*
* This rountine handles the c775 TDC interrupt.  A user routine is
* called, if one was connected by c775IntConnect().
*
* RETURNS: N/A
*
*/
//FIXME SKIPPED
LOCAL void
c775Int(void)
{
  int ii = 0;
  UINT32 nevt = 0;

  /* Disable interrupts */
#ifdef VXWORKS
  sysIntDisable(c775IntLevel);
#endif

  c775IntCount++;

#ifndef VXWORKS
  vmeBusLock();
#endif

  if (c775IntRoutine != NULL)
    {				/* call user routine */
      (*c775IntRoutine) (c775IntArg);
    }
  else
    {
      if ((c775IntID < 0) || (c775p[c775IntID] == NULL))
	{
	  logMsg("c775Int: ERROR : TDC id %d not initialized \n", c775IntID,
		 0, 0, 0, 0, 0);
	  return;
	}
      /* Default action is to increment the Read pointer by
         the number of events in the Event Trigger register
         or until the Data buffer is empty. The later case would
         indicate a possible error. In either case the data is
         effectively thrown away */
      C775LOCK;
      nevt = vmeRead16(&c775p[c775IntID]->main.evTrigger) & C775_EVTRIGGER_MASK;
      C775UNLOCK;
      while ((ii < nevt) && (c775Dready(c775IntID) > 0))
	{
	  C775LOCK;
	  C775_EXEC_INCR_EVENT(c775IntID);
	  C775UNLOCK;
	  ii++;
	}
      if (ii < nevt)
	logMsg
	  ("c775Int: WARN : TDC %d - Events dumped (%d) != Events Triggered (%d)\n",
	   c775IntID, ii, nevt, 0, 0, 0);
      logMsg("c775Int: Processed %d events\n", nevt, 0, 0, 0, 0, 0);

    }

  /* Enable interrupts */
#ifdef VXWORKS
  sysIntEnable(c775IntLevel);
#else
  vmeBusUnlock();
#endif


}


/*******************************************************************************
*
* c775IntConnect - connect a user routine to the c775 TDC interrupt
*
* This routine specifies the user interrupt routine to be called at each
* interrupt.
*
* RETURNS: OK, or ERROR if Interrupts are enabled
*/
//FIXME SKIPPED

STATUS
c775IntConnect(VOIDFUNCPTR routine, int arg, UINT16 level, UINT16 vector)
{

  if (c775IntRunning)
    {
      printf
	("c775IntConnect: ERROR : Interrupts already Initialized for TDC id %d\n",
	 c775IntID);
      return (ERROR);
    }

  c775IntRoutine = routine;
  c775IntArg = arg;

  /* Check for user defined VME interrupt level and vector */
  if (level == 0)
    {
      c775IntLevel = C775_VME_INT_LEVEL;	/* use default */
    }
  else if (level > 7)
    {
      printf
	("c775IntConnect: ERROR: Invalid VME interrupt level (%d). Must be (1-7)\n",
	 level);
      return (ERROR);
    }
  else
    {
      c775IntLevel = level;
    }

  if (vector == 0)
    {
      c775IntVec = C775_INT_VEC;	/* use default */
    }
  else if ((vector < 32) || (vector > 255))
    {
      printf
	("c775IntConnect: ERROR: Invalid interrupt vector (%d). Must be (32<vector<255)\n",
	 vector);
      return (ERROR);
    }
  else
    {
      c775IntVec = vector;
    }

  /* Connect the ISR */
#ifdef VXWORKSPPC
  if ((intDisconnect((int) INUM_TO_IVEC(c775IntVec)) != 0))
    {
      printf("c775IntConnect: ERROR disconnecting Interrupt\n");
      return (ERROR);
    }
#endif
#ifdef VXWORKS
  if ((intConnect(INUM_TO_IVEC(c775IntVec), c775Int, 0)) != 0)
    {
      printf("c775IntConnect: ERROR in intConnect()\n");
      return (ERROR);
    }
#else
  if (vmeIntDisconnect(c775IntLevel) != 0)
    {
      printf("c775IntConnect: ERROR disconnecting Interrupt\n");
      return (ERROR);
    }
  if (vmeIntConnect(c775IntVec, c775IntLevel, c775Int, 0) != 0)
    {
      printf("c775IntConnect: ERROR in intConnect()\n");
      return (ERROR);
    }
#endif

  return (OK);
}


/*******************************************************************************
*
* c775IntEnable - Enable interrupts from specified TDC
*
* Enables interrupts for a specified TDC.
*
* RETURNS OK or ERROR if TDC is not available or parameter is out of range
*/

STATUS
c775IntEnable(int id, UINT16 evCnt)
{

  if (c775IntRunning)
    {
      printf
	("c775IntEnable: ERROR : Interrupts already initialized for TDC id %d\n",
	 c775IntID);
      return (ERROR);
    }

  if ((id < 0) || (c775p[id] == NULL))
    {
      printf("c775IntEnable: ERROR : TDC id %d not initialized \n", id);
      return (ERROR);
    }
  else
    {
      c775IntID = id;
    }

  /* check for event count out of range */
  if ((evCnt <= 0) || (evCnt > 31))
    {
      printf
	("c775IntEnable: ERROR: Event count %d for Interrupt is out of range (1-31)\n",
	 evCnt);
      return (ERROR);
    }

#ifdef VXWORKS
  sysIntEnable(c775IntLevel);	/* Enable VME interrupts */
#endif

  /* Zero Counter and set Running Flag */
  c775IntEvCount = evCnt;
  c775IntCount = 0;
  c775IntRunning = TRUE;
  /* Enable interrupts on TDC */
  C775LOCK;
  vmeWrite16(&c775p[c775IntID]->main.intVector, c775IntVec);
  vmeWrite16(&c775p[c775IntID]->main.intLevel, c775IntLevel);
  vmeWrite16(&c775p[c775IntID]->main.evTrigger, c775IntEvCount);
  C775UNLOCK;

  return (OK);
}


/*******************************************************************************
*
* c775IntDisable - disable the TDC interrupts
*
* RETURNS: OK, or ERROR if not initialized
*/

STATUS
c775IntDisable(int iflag)
{

  if ((c775IntID < 0) || (c775p[c775IntID] == NULL))
    {
      logMsg("c775IntDisable: ERROR : TDC id %d not initialized \n",
	     c775IntID, 0, 0, 0, 0, 0);
      return (ERROR);
    }

#ifdef VXWORKS
  sysIntDisable(c775IntLevel);	/* Disable VME interrupts */
#endif
  C775LOCK;
  vmeWrite16(&c775p[c775IntID]->main.evTrigger, 0);

  /* Tell tasks that Interrupts have been disabled */
  if (iflag > 0)
    {
      c775IntRunning = FALSE;
      vmeWrite16(&c775p[c775IntID]->main.intLevel, 0);
      vmeWrite16(&c775p[c775IntID]->main.intVector, 0);
    }
#ifdef VXWORKS
  else
    {
      semGive(c775Sem);
    }
#endif

  C775UNLOCK;
  return (OK);
}

/*******************************************************************************
*
* c775IntResume - Re-enable interrupts from previously
*                 intitialized TDC
*
* RETURNS: OK, or ERROR if not initialized
*/

STATUS
c775IntResume(void)
{
  UINT16 evTrig = 0;

  if ((c775IntID < 0) || (c775p[c775IntID] == NULL))
    {
      logMsg("c775IntResume: ERROR : TDC id %d not initialized \n", c775IntID,
	     0, 0, 0, 0, 0);
      return (ERROR);
    }

  C775LOCK;
  if ((c775IntRunning))
    {
      evTrig = vmeRead16(&c775p[c775IntID]->main.evTrigger) & C775_EVTRIGGER_MASK;
      if (evTrig == 0)
	{
#ifdef VXWORKS
	  sysIntEnable(c775IntLevel);
#endif
	  vmeWrite16(&c775p[c775IntID]->main.evTrigger, c775IntEvCount);
	}
      else
	{
	  logMsg("c775IntResume: WARNING : Interrupts already enabled \n", 0,
		 0, 0, 0, 0, 0);
	  C775UNLOCK;
	  return (ERROR);
	}
    }
  else
    {
      logMsg("c775IntResume: ERROR : Interrupts are not Enabled \n", 0, 0, 0,
	     0, 0, 0);
      C775UNLOCK;
      return (ERROR);
    }

  C775UNLOCK;
  return (OK);
}



/*******************************************************************************
*
* c775Sparse - Enable/Disable Overflow and Under threshold sparsification
*
*
* RETURNS: Bit Set 2 Register value.
*/

UINT16
c775Sparse(int id, int over, int under)
{
  UINT16 rval;

  if ((id < 0) || (c775p[id] == NULL))
    {
      printf("c775Sparse: ERROR : TDC id %d not initialized \n", id);
      return (0xffff);
    }

  C775LOCK;
  if (!over)
    {				/* Set Overflow suppression */
      vmeWrite16(&c775p[id]->main.bitSet2, C775_OVER_RANGE);
    }
  else
    {
      vmeWrite16(&c775p[id]->main.bitClear2, C775_OVER_RANGE);
    }

  if (!under)
    {				/* Set Underflow suppression */
      vmeWrite16(&c775p[id]->main.bitSet2, C775_LOW_THRESHOLD);
    }
  else
    {
      vmeWrite16(&c775p[id]->main.bitClear2, C775_LOW_THRESHOLD);
    }
  rval = vmeRead16(&c775p[id]->main.bitSet2) & C775_BITSET2_MASK;

  C775UNLOCK;
  return (rval);
}


/*******************************************************************************
*
* c775Dready - Return status of Data Ready bit in TDC
*
*
* RETURNS: 0(No Data) or  # of events in FIFO (1-32) or ERROR.
*/

int
c775Dready(int id)
{

  int nevts = 0;
  UINT16 stat = 0;


  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Dready: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return (ERROR);
    }

  C775LOCK;
  stat = vmeRead16(&c775p[id]->main.status1) & C775_DATA_READY;
  if (stat)
    {
      C775_EXEC_READ_EVENT_COUNT(id);
      nevts = c775EventCount[id] - c775EvtReadCnt[id];
      if (nevts <= 0)
	{
	  logMsg("c775Dready: ERROR : Bad Event Ready Count (nevts = %d)\n",
		 nevts, 0, 0, 0, 0, 0);
	  C775UNLOCK;
	  return (ERROR);
	}
    }

  C775UNLOCK;
  return (nevts);
}


/*******************************************************************************
*
* c775SetFSR - Set and/or Return TDC full scale range programming
*
*      Register value:    0xff (255) ->  35 ps/count  (  140ns FSR )
*       range between:    0x1e ( 30) -> 300 ps/count  ( 1200ns FSR )
*
*        FSR = 4*(290-reg)
*
*        reg = 290 - FSR/4
*
*      Note: Passing 0 for fsr will return the contents of the register
*            but not set it.
*
* RETURNS: Full scale range of the TDC in nanoseconds or ERROR.
*/

int
c775SetFSR(int id, UINT16 fsr)
{

  int rfsr = 0;
  UINT16 reg;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775SetFSR: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return (ERROR);
    }

  C775LOCK;
  if (fsr == 0)
    {
      reg = vmeRead16(&c775p[id]->main.fsr) & C775_FSR_MASK;
      rfsr = (int) (290 - reg) * 4;
    }
  else if ((fsr < C775_MIN_FSR) || (fsr > C775_MAX_FSR))
    {
      logMsg("c775SetFSR: ERROR: FSR (%d ns) out of range (140<=FSR<=1200)\n",
	     fsr, 0, 0, 0, 0, 0);
      C775UNLOCK;
      return (ERROR);
    }
  else
    {
      reg = (UINT16) (290 - (fsr >> 2));
      vmeWrite16(&c775p[id]->main.fsr, reg);
      reg = vmeRead16(&c775p[id]->main.fsr) & C775_FSR_MASK;
      rfsr = (int) (290 - reg) * 4;
    }

  C775UNLOCK;
  return (rfsr);

}

/******************************************************************************
 *
 *
 * c775BitSet2      - Program Bit Set 2 register
 * c775BitClear2    - Program Bit clear 2 register
 *
 */

INT16
c775BitSet2(int id, UINT16 val)
{
  INT16 rval;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775BitSet2: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return (ERROR);
    }

  C775LOCK;
  if (val)
    vmeWrite16(&c775p[id]->main.bitSet2, val);
  rval = vmeRead16(&c775p[id]->main.bitSet2) & C775_BITSET2_MASK;

  C775UNLOCK;
  return (rval);
}

INT16
c775BitClear2(int id, UINT16 val)
{
  INT16 rval;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775BitClear2: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return (ERROR);
    }

  C775LOCK;
  if (val)
    vmeWrite16(&c775p[id]->main.bitClear2, val);
  rval = vmeRead16(&c775p[id]->main.bitSet2) & C775_BITSET2_MASK;

  C775UNLOCK;
  return (rval);
}


/*******************************************************************************
*
* c775ClearThresh  - Zero TDC thresholds for all channels
* c775Gate         - Issue Software Gate to TDC
* c775EnableBerr   - Enable Bus Error termination of block reads
* c775DisableBerr  - Disable Bus Error termination of block reads
* c775IncrEventBlk - Increment Event counter for Block reads
* c775IncrEvent    - Increment Read pointer to next event in the Buffer
* c775IncrWord     - Increment Read pointer to next word in the event
* c775Enable       - Bring TDC Online (Enable Gates)
* c775Disable      - Bring TDC Offline (Disable Gates)
* c775CommonStop   - Program for Common Stop
* c775CommonStart  - Program for Common Start (Default)
* c775Clear        - Clear TDC
* c775Reset        - Clear/Reset TDC
*
*
* RETURNS: None.
*/

void
c775ClearThresh(int id)
{
  int ii;

  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775ClearThresh: ERROR : TDC id %d not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  C775LOCK;
  for (ii = 0; ii < C775_MAX_CHANNELS; ii++)
    {
      vmeWrite16(&c775p[id]->main.threshold[ii], 0);
    }
  C775UNLOCK;
}

void
c775Gate(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Gate: ERROR : TDC id %d not initialized \n", id, 0, 0, 0, 0,
	     0);
      return;
    }
  C775LOCK;
  C775_EXEC_GATE(id);
  C775UNLOCK;
}

void
c775EnableBerr(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775EnableBerr: ERROR : QDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }

  C775LOCK;
  vmeWrite16(&c775p[id]->main.control1, C775_BERR_ENABLE);	/*  | C775_BLK_END); */
  C775UNLOCK;
}

void
c775DisableBerr(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775DisableBerr: ERROR : QDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }

  C775LOCK;
  vmeWrite16(&c775p[id]->main.control1,
	    vmeRead16(&c775p[id]->main.control1)
	     & ~(C775_BERR_ENABLE | C775_BLK_END));
  C775UNLOCK;
}

void
c775IncrEventBlk(int id, int count)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775IncrEventBlk: ERROR : TDC id %d not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }

  if ((count > 0) && (count <= 32))
    c775EvtReadCnt[id] += count;
}

void
c775IncrEvent(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775IncrEvent: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }
  C775LOCK;
  C775_EXEC_INCR_EVENT(id);
  C775UNLOCK;
}

void
c775IncrWord(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775IncrWord: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }
  C775LOCK;
  C775_EXEC_INCR_WORD(id);
  C775UNLOCK;
}

void
c775Enable(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Enable: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return;
    }
  C775LOCK;
  vmeWrite16(&c775p[id]->main.bitClear2, C775_OFFLINE);
  C775UNLOCK;
}

void
c775Disable(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Disable: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return;
    }
  C775LOCK;
  vmeWrite16(&c775p[id]->main.bitSet2, C775_OFFLINE);
  C775UNLOCK;
}

void
c775CommonStop(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775CommonStop: ERROR : TDC id %d not initialized \n", id, 0, 0,
	     0, 0, 0);
      return;
    }
  C775LOCK;
  vmeWrite16(&c775p[id]->main.bitSet2, C775_COMMON_STOP);
  C775UNLOCK;
}

void
c775CommonStart(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775CommonStart: ERROR : TDC id %d not initialized \n", id, 0,
	     0, 0, 0, 0);
      return;
    }
  C775LOCK;
  vmeWrite16(&c775p[id]->main.bitClear2, C775_COMMON_STOP);
  C775UNLOCK;
}


void
c775Clear(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Clear: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return;
    }
  C775LOCK;
  C775_EXEC_DATA_RESET(id);
  C775UNLOCK;
  c775EvtReadCnt[id] = -1;
  c775EventCount[id] = 0;

}

void
c775Reset(int id)
{
  if ((id < 0) || (c775p[id] == NULL))
    {
      logMsg("c775Reset: ERROR : TDC id %d not initialized \n", id, 0, 0, 0,
	     0, 0);
      return;
    }
  C775LOCK;
  C775_EXEC_DATA_RESET(id);
  C775_EXEC_SOFT_RESET(id);
  C775UNLOCK;
  c775EvtReadCnt[id] = -1;
  c775EventCount[id] = 0;
}
