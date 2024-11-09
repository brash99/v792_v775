#define ROL_NAME__ "VME1"
#define MAX_EVENT_LENGTH 9216
#define MAX_EVENT_POOL   500
#define VME
/* POLLING_MODE */
#define POLLING___
#define POLLING_MODE
#define INIT_NAME c792_list__init
#define INIT_NAME_POLL c792_list__poll
#include <rol.h>
#include <VME_source.h>
#define TIR_ADDR 0x0ed0
#define ADC_ID 0
#define MAX_ADC_DATA 34
unsigned long laddr;
extern int bigendian_out;
static void __download()
{
    daLogMsg("INFO","Readout list compiled %s", DAYTIME);
#ifdef POLLING___
   rol->poll = 1;
#endif
    *(rol->async_roc) = 0; /* Normal ROC */
  {  /* begin user */
unsigned long res;
  bigendian_out = 1;
  tirInit(TIR_ADDR);
    daLogMsg("INFO","User Download Executed");

  }  /* end user */
} /*end download */     

static void __prestart()
{
CTRIGINIT;
    *(rol->nevents) = 0;
  {  /* begin user */
    daLogMsg("INFO","Entering User Prestart");

    VME_INIT;
    CTRIGRSS(VME,1,usrtrig,usrtrig_done);
    CRTTYPE(1,VME,1);
  c792Sparse(ADC_ID,0,0);
  c792Clear(ADC_ID);
  c792Status(ADC_ID,0,0);
    daLogMsg("INFO","User Prestart Executed");

  }  /* end user */
    if (__the_event__) WRITE_EVENT_;
    *(rol->nevents) = 0;
    rol->recNb = 0;
} /*end prestart */     

static void __end()
{
  {  /* begin user */
  CDODISABLE(VME,1,0);
  c792Status(ADC_ID,0,0);
    daLogMsg("INFO","User End Executed");

  }  /* end user */
    if (__the_event__) WRITE_EVENT_;
} /* end end block */

static void __pause()
{
  {  /* begin user */
  CDODISABLE(VME,1,0);
    daLogMsg("INFO","User Pause Executed");

  }  /* end user */
    if (__the_event__) WRITE_EVENT_;
} /*end pause */
static void __go()
{

  {  /* begin user */
    daLogMsg("INFO","Entering User Go");

  CDOENABLE(VME,1,0);
  }  /* end user */
    if (__the_event__) WRITE_EVENT_;
}

void usrtrig(unsigned int EVTYPE,unsigned int EVSOURCE)
{
    int EVENT_LENGTH;
  {  /* begin user */
unsigned long stat, csr;
 rol->dabufp = (long *) 0;
    CEOPEN(EVTYPE, BT_UI4, blockLevel);
{/* inline c-code */
 
 int nwrds=0;
 /* Check if an Event is available */
 stat = c792Dready(ADC_ID);
 if(stat > 0) {
   nwrds = c792ReadEvent(ADC_ID,rol->dabufp);
   if(nwrds<=0) {
      logMsg("ERROR: ADC Read Failed - Status 0x%x\n",nwrds,0,0,0,0,0);
      *rol->dabufp++ = 0xda000bad;
      c792Clear(ADC_ID);
   } else {
      rol->dabufp += nwrds;
   }
 }else{
   logMsg("ERROR: NO data in ADC  datascan = 0x%x\n",stat,0,0,0,0,0);
   c792Clear(ADC_ID);
 }

 
 }/*end inline c-code */
    CECLOSE;
  }  /* end user */
} /*end trigger */

void usrtrig_done()
{
  {  /* begin user */
  }  /* end user */
} /*end done */

void __done()
{
poolEmpty = 0; /* global Done, Buffers have been freed */
  {  /* begin user */
  CDOACK(VME,1,0);
  }  /* end user */
} /*end done */

static void __status()
{
  {  /* begin user */
  }  /* end user */
} /* end status */

