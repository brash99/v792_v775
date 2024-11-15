/*************************************************************************
 *
 *  c792_linux_list.c - Library of routines for the user to write for
 *                      readout and buffering of events using
 *                      a Linux VME controller.
 *
 */

/* Event Buffer definitions */
#define MAX_EVENT_POOL     400
#define MAX_EVENT_LENGTH   1024*10      /* Size in Bytes */

/* Define Interrupt source and address */
#define TIR_SOURCE
#define TIR_ADDR 0x0ed0
//#define TIR_ADDR 0x800000
#define TIR_MODE TIR_EXT_POLL

#define ADC_ID 0
#define MAX_ADC_DATA 34 

#define TDC_ID 0
#define MAX_TDC_DATA 34

#include "linuxvme_list.c"
#include "c792Lib.h"
#include "c775Lib.h"


/* function prototype */
void rocTrigger(int arg);

/* function prototype */
void rocCleanup()
{

/* Add things here to clean up ROC at some point, maybe */

}

void
rocDownload()
{
  int dmaMode;

  /* Setup Address and data modes for DMA transfers
   *   
   *  vmeDmaConfig(addrType, dataType, sstMode);
   *
   *  addrType = 0 (A16)    1 (A24)    2 (A32)
   *  dataType = 0 (D16)    1 (D32)    2 (BLK32) 3 (MBLK) 4 (2eVME) 5 (2eSST)
   *  sstMode  = 0 (SST160) 1 (SST267) 2 (SST320)
   */
  vmeDmaConfig(1,3,0); 

  ////c775Init(0x08A10000,0,1,0);//this is taken from c775_linux_list.c TONY
  c775Init(0xa10000,0,1,0);
  ////c792Init(0x08A20000,0,1,0);//0x08A20000 is user address specified on jumpers
  c792Init(0x110000,0,1,0); // we think that the above address means A24?

  printf("rocDownload: User Download Executed\n");

}

void
rocPrestart()
{
  unsigned short iflag;
  int stat;

  /* Program/Init VME Modules Here */
  /* Setup ADCs (no sparsification, enable berr for block reads) */
  c792Sparse(ADC_ID,0,0);
  c792Clear(ADC_ID);
  c792DisableBerr(ADC_ID); // Disable berr - multiblock read
/*  c792EnableBerr(ADC_ID); /\* for 32bit block transfer *\/ */

  c792Status(ADC_ID,0,0);
  
/* Program/Init VME Modules Here */
  /* Setup TDCs (no sparcification, enable berr for block reads) */
  c775Clear(TDC_ID);
  c775DisableBerr(TDC_ID); // Disable berr - multiblock read
/*   c775EnableBerr(TDC_ID); /\* for 32bit block transfer *\/ */
  c775CommonStop(TDC_ID);
  //c775CommonStart(TDC_ID);

  c775Status(TDC_ID);
  
  printf("rocPrestart: User Prestart Executed\n");

}

void
rocGo()
{
  printf("rocGo: Go!!!");
  /* Enable modules, if needed, here */
  /* c775IntEnable(TDC_ID,0); */
  c775Status(TDC_ID);
  c792Status(ADC_ID,0,0); //TONY ADDED THIS
  printf("rocGo: After status!!!");
  /* Interrupts/Polling enabled after conclusion of rocGo() */
}

void
rocEnd()
{


  //  int status, count;  
  c775Status(TDC_ID);
  c792Status(ADC_ID,0,0); //TONY ADDED THIS
  // c775Disable(TDC_ID); //Commented out, Brash: June 1, 2023

  printf("rocEnd: Ended after %d events\n",tirGetIntCount());
  
}

void
rocTrigger(int arg)
{

  int ii, status, dma, count;
  int nwords;
  unsigned int datascan, tirval, vme_addr;
  unsigned long long length,size;
  int itimeout=0;

/* /\*   tirIntOutput(2); *\/ */

  printf("Event Count: %d\n",tirGetIntCount());

  *dma_dabufp++ = LSWAP(tirGetIntCount()); /* Insert Event Number */

  /* Check if an Event is available */

  while(itimeout<1000)
    {
      itimeout++;
      status = c792Dready(ADC_ID);
      if(status>0) break;
    }
  if(status > 0)
    {
      if(tirGetIntCount() %100==0)
	{
	  printf("itimeout = %d\n",itimeout);
	  c792PrintEvent(ADC_ID,0);
	}
      else
	{
 	  nwords = c792ReadEvent(ADC_ID,dma_dabufp);
          /* or use c792ReadBlock, if BERR was enabled */
/*	  nwords = c792ReadBlock(ADC_ID,dma_dabufp,MAX_ADC_DATA); */
	  if(nwords<=0)
	    {
	      logMsg("ERROR: ADC Read Failed - Status 0x%x\n",nwords,0,0,0,0,0);
	      *dma_dabufp++ = 0xda000bad;
	      c792Clear(ADC_ID);
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      logMsg("ERROR: NO data in ADC  datascan = 0x%x, itimeout=%d\n",status,itimeout,0,0,0,0);
      c792Clear(ADC_ID);
    }

  while(itimeout<1000)
    {
      itimeout++;
      status = c775Dready(TDC_ID);
      if(status>0) break;
    }
  if(status > 0)
    {
      if(tirGetIntCount() %100==0)
	{
	  printf("itimeout = %d\n",itimeout);
	  c775PrintEvent(TDC_ID,0);
	}
      else
	{
	  nwords = c775ReadEvent(TDC_ID,dma_dabufp);
	  /* or use c775ReadBlock, if BERR was enabled */
/* 	  nwords = c775ReadBlock(TDC_ID,dma_dabufp,MAX_TDC_DATA); */
	  if(nwords<=0)
	    {
	      logMsg("ERROR: TDC Read Failed - Status 0x%x\n",nwords,0,0,0,0,0);
	      *dma_dabufp++ = 0xda000bad;
	      c775Clear(TDC_ID);
	    }
	  else
	    {
	      dma_dabufp += nwords;
	    }
	}
    }
  else
    {
      //      logMsg("ERROR: NO data in TDC  datascan = 0x%x, itimeout=%d\n",status,itimeout,0,0,0,0);
      c775Clear(TDC_ID);
    }
  *dma_dabufp++ = LSWAP(0xdaebd00d); /* Event EOB */ //TONY - made no change

/*   tirIntOutput(0); */

}
