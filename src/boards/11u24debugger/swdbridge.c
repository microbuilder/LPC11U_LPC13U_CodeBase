/****************************************************************************
 *   $Id:: swdbridge.c 6085 2011-01-05 23:55:06Z usb00423                   $
 *   Project: LPC11U00 SWD to USB bridge
 *
 *   Description:
 *     Source code file for SWD to USB bridge firmware
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.
****************************************************************************/
#include "projectconfig.h"

#if defined CFG_BRD_LPC11U24_DEBUGGER

#include "boards/board.h"
#include "core/usb/usbd.h"
#include "swdbridge.h"
#include "swd.h"

//volatile tInReport  InReport;
//volatile tOutReport OutReport;

#define SWDBRIDGE_QUEUE_DEPTH (8)

volatile int        InReport_readidx=0, InReport_writeidx=0;
volatile tInReport  InReports[SWDBRIDGE_QUEUE_DEPTH];
volatile int        OutReport_writeidx=0, OutReport_readidx=0;
volatile tOutReport OutReports[SWDBRIDGE_QUEUE_DEPTH];

volatile int LED_on = 0;
volatile int LED_off_counter = 0;

/**************************************************************************/
/*!
    @brief  Get HID Input Report (InReport)
*/
/**************************************************************************/
//void swdbridgeGetInReport (void)
bool usb_hid_generic_report_request_isr(USB_HID_GenericReportIn_t *in_report)
{
  if(InReport_readidx != InReport_writeidx)
  {
    *in_report = InReports[InReport_readidx];
    InReport_readidx = (InReport_readidx+1) % SWDBRIDGE_QUEUE_DEPTH;

    LOG("In report" CFG_PRINTF_NEWLINE
        "\tStatus   = %d - %s" CFG_PRINTF_NEWLINE
        "\tCount    = %d" CFG_PRINTF_NEWLINE
        "\tSequence = %d",
        in_report->Status, REPORT_STATUS_STR(in_report->Status), in_report->Count, in_report->Sequence);
    LOG_TAB(1);
    LOG_ARR(in_report->ReadData, 15, "%08X");

  }
  else
  {
    in_report->Status = SWDBRIDGE_OR_STATUS_NOTHING;
  }

  if(LED_off_counter)
  {
    LED_off_counter--;
  }
  else
  {
    if(LED_on)
      boardLED(CFG_LED_OFF);
    else
      boardLED(CFG_LED_ON);
  }

  return true;
}

/**************************************************************************/
/*!
    @brief  Set HID Output Report (OutReport)
*/
/**************************************************************************/
void usb_hid_generic_recv_isr(USB_HID_GenericReportOut_t  *out_report)
{
        if(out_report->Opcode != SWDBRIDGE_OR_OPCODE_NOTHING)
        {
                OutReports[OutReport_writeidx] = *out_report;
                OutReport_writeidx = (OutReport_writeidx+1) % SWDBRIDGE_QUEUE_DEPTH;
        }
}

volatile uint32_t APSelect = 0, TAR = 0xFFFFFFFF;

uint8_t ReadCore(uint8_t regno, uint32_t *data);

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadDP( uint8_t address, uint32_t *data)
{
  uint8_t status;

  do
  {
    status = swdRead(0, address, (unsigned long *)data);
  } while(status == SWDBRIDGE_OR_STATUS_WAIT);

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t WriteDP( uint8_t address, uint32_t data)
{
  uint8_t status;

  do
  {
    status = swdWrite(0, address, data);
  } while(status == SWDBRIDGE_OR_STATUS_WAIT);

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t SetAPSelect(uint8_t address)
{
  uint32_t new_APSelect = (APSelect&(~0xF0)) | (address&0xF0);
  uint8_t status = SWDBRIDGE_OR_STATUS_ACK;

  if(new_APSelect != APSelect)
  {
    status = WriteDP(8, new_APSelect);
    if(status == SWDBRIDGE_OR_STATUS_ACK)
    {
      APSelect = new_APSelect;
    }
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadAP( uint8_t address, uint32_t *data)
{
  if(address == 0x04 && TAR != 0xFFFFFFFF)
  {
    return TAR;
  }

  uint8_t status = SetAPSelect(address);

  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    do
    {
      status = swdRead(1, address&0x0F, (unsigned long *)data);
    } while(status == SWDBRIDGE_OR_STATUS_WAIT);
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadAP2( uint8_t address, uint32_t *data)
{
  uint8_t status;

  status = ReadAP(address, data);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    status = ReadAP(address, data);
  }
  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t WriteAP( uint8_t address, uint32_t data)
{
  uint8_t status = SetAPSelect(address);

  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    do
    {
      status = swdWrite(1, address&0x0F, data);
    } while(status == SWDBRIDGE_OR_STATUS_WAIT);
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t WriteMemory(uint32_t address, uint32_t data)
{
  uint8_t status;

  status = WriteAP( 4, address);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    status = WriteAP( 0x0C, data);
    swdFlush();
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t WriteMemoryArray(uint32_t address, const uint32_t *data, uint32_t count)
{
  uint8_t status;
  int i;

  status = WriteAP( 4, address);
  i = 0;
  while(status == SWDBRIDGE_OR_STATUS_ACK && i < count)
  {
    status = WriteAP( 0x0C, data[i]);
    i++;
  }
  swdFlush();

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadMemory(uint32_t address, uint32_t *data)
{
  uint8_t status;

  status = WriteAP( 4, address);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    status = ReadAP2( 0x0C, data);
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadMemoryArray(uint32_t address, uint32_t *data, uint32_t count)
{
  uint8_t status;
  int i;

  status = WriteAP( 4, address);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
  status = ReadAP( 0x0C, &data[0]); // dummy read
  i = 0;
  while(status == SWDBRIDGE_OR_STATUS_ACK && i < count)
  {
    status = ReadAP( 0x0C, &data[i]);
    i++;
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Run()
{
  uint8_t status = WriteAP(4, 0xE000EDF0);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    uint32_t data;

    status = ReadAP2(0x10, &data);
    if(data & (1<<17) && status == SWDBRIDGE_OR_STATUS_ACK)
    {
      data &= 0xFFFF; data |= 0xA05F << 16;
      data &= ~7;
      data |= 1;
      status = WriteAP(0x10, data);
      swdFlush();
    }
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Step()
{
  uint8_t status = WriteAP(4, 0xE000EDF0);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    uint32_t data;

    status = ReadAP2(0x10, &data);
    if(data & (1<<17) && status == SWDBRIDGE_OR_STATUS_ACK)
    {
      data &= 0xFFFF; data |= 0xA05F << 16;
      data &= ~7;
      data |= 1 | 4;
      status = WriteAP(0x10, data);
      swdFlush();
    }
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Stop()
{
  uint8_t status = WriteAP(4, 0xE000EDF0);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
  {
    uint32_t data;

    status = ReadAP2(0x10, &data);
    if(!(data & (1<<17)) && status == SWDBRIDGE_OR_STATUS_ACK)
    {
      data &= 0xFFFF; data |= 0xA05F << 16;
      data &= ~7;
      data |= 3;
      status = WriteAP(0x10, data);
      swdFlush();
    }
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Reset()
{
  uint32_t data, wps;
  uint8_t status = Stop();
  volatile int i;

  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

  // Turn on reset vector + exception catch
  status = WriteMemory(0xE000EDFC, (1<<24) | (1<<10) | (1<<8) | (1<<4) | (1<<0));
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

  // ReadCore(SWDBRIDGE_CORE_PC, &data);
  // Reset the chip
  status = WriteMemory(0xE000ED0C, (0x5FA << 16) | 4);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;


  //      ReadCore(SWDBRIDGE_CORE_PC, &data);

  // Enable Data Watchpoint & Trace module
  status = ReadMemory(0xE000EDFC, &data);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
  status = WriteMemory(0xE000EDFC, data | 1<<24);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
  status = ReadMemory(0xE000EDFC, &data);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

  // Read and figure out how many watchpoints are supported
  status = ReadMemory(0xE0001000, &wps);
  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
  wps = (wps>>28) & 0xF;
  if(wps >= 2)
  {
          // Set up 2 watchpoints to stop when flash/rom select register written
          for(i=0;i<wps;i++)
          {
                  uint32_t wpbase = 0xE0001000 + 0x20 + (i * 0x20);
                  if(i==0)
                  {
                          status = WriteMemory(wpbase, 0x40048000);
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                          status = WriteMemory(wpbase + 4, 0);
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                          status = WriteMemory(wpbase + 8, 6); // break on write
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                  } else if(i==1)
                  {
                          status = WriteMemory(wpbase, 0x400FC040);
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                          status = WriteMemory(wpbase + 4, 0);
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                          status = WriteMemory(wpbase + 8, 6); // break on write
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                  } else
                  {
                          status = WriteMemory(wpbase + 8, 0); // disabled
                          if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                  }
          }

  //              ReadCore(SWDBRIDGE_CORE_PC, &data);
          // Run boot rom until it stops at the watchpoint
          status = Run();
          if(status != SWDBRIDGE_OR_STATUS_ACK && status != 7)
                  return status;

          // Wait until MCU has halted
          // potentially could hang here if no bootrom or wrong watchpoint addresses
          // need to add timeout XXXXXX
          for(i=0;i<50000;i++);

          do
          {
                  status = ReadAP(0x10, &data);
          } while(status == 7);
          if(status != SWDBRIDGE_OR_STATUS_ACK)
                  return status;

          do
          {
                  do
                  {
                          status = ReadAP(0x10, &data);
                  } while(status == 7);
                  if(status != SWDBRIDGE_OR_STATUS_ACK)
                          return status;
          } while(!(data & (1 << 17)));

  //              ReadCore(SWDBRIDGE_CORE_PC, &data);
          // clear all watchpoints, leave core halted
          for(i=0;i<wps;i++)
          {
                  uint32_t wpbase = 0xE0001000 + 0x20 + (i * 0x20);
                  status = WriteMemory(wpbase + 8, 0); // disabled
                  if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
          }

          // step one instruction
          Step();
  //              ReadCore(SWDBRIDGE_CORE_PC, &data);

  //       Run(); // XXXXXX
  //              ReadCore(SWDBRIDGE_CORE_PC, &data);
  //              ReadCore(SWDBRIDGE_CORE_PC, &data);
  } else
  { // There are NO watchpoints in this part. We will have to do something else?
  }

  return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t ReadCore(uint8_t regno, uint32_t *data)
{
        uint8_t status = WriteAP( 0x4, 0xE000EDF0); // set up TAR
        if(status == SWDBRIDGE_OR_STATUS_ACK)
        {
                status = WriteAP( 0x14, regno);
                if(status == SWDBRIDGE_OR_STATUS_ACK)
                        status = ReadAP2( 0x18, data);
        }
        return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t WriteCore(uint8_t regno, uint32_t data)
{
        uint8_t status = WriteAP( 0x4, 0xE000EDF0); // set up TAR
        if(status == SWDBRIDGE_OR_STATUS_ACK)
        {
                status = WriteAP( 0x18, data);
                if(status == SWDBRIDGE_OR_STATUS_ACK)
                        status = WriteAP( 0x14, ((uint32_t)regno) | 1<<16);
        }
        return status;
}

uint32_t m_IAPRAMStart;
uint32_t m_IAPCode;
uint32_t m_IAPBuffer;
uint32_t m_IAPCommand;
uint32_t m_IAPResult;
uint32_t m_IAPStack;

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t SetupInvoke()
{
    uint32_t CallBreak, MoveMove;
    uint8_t status;

    m_IAPRAMStart = 0x10000000;
    m_IAPCode = m_IAPRAMStart;
    m_IAPBuffer = m_IAPCode + 16;
    m_IAPCommand = m_IAPBuffer + 512;
    m_IAPResult = m_IAPCommand + 5 * 4;
    m_IAPStack = m_IAPCommand + 64;

    status = Stop();
    if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

        // Set up stack pointer- end of 1K memory block, -32
        status = WriteCore(SWDBRIDGE_CORE_SP, m_IAPStack + 256);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

    // mov r0, r4
    MoveMove = (0x4600 | (SWDBRIDGE_CORE_R4 << 3) | (SWDBRIDGE_CORE_R0));
    // mov r1, r5
    MoveMove |= (0x4600 | (SWDBRIDGE_CORE_R5 << 3) | (SWDBRIDGE_CORE_R1)) << 16;
//    MoveMove = (0xBF00 << 16) | 0xBF00;

    // Set up call (BLX) and breakpoint (BKPT) instructions
    CallBreak = (0x4780) | ((SWDBRIDGE_CORE_R8) << 3); // BLX R8
    CallBreak |= (0xBE00) << 16; // BKPT #0
//    CallBreak = ((0xBE00) << 16) | 0xBF00; // BKPT #0

    // Write BLX R2 / BKPT instructions into code memory
    status = WriteMemory(m_IAPCode, MoveMove);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
        status = WriteMemory(m_IAPCode+4, CallBreak);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
    // Write IAP Entry into R8
        status = WriteCore(SWDBRIDGE_CORE_R8, 0x1fff1ff1 | 1);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
    // args- command, result
        status = WriteCore(SWDBRIDGE_CORE_R4, m_IAPCommand);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
    status = WriteCore(SWDBRIDGE_CORE_R5, m_IAPResult);
        return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Invoke(const uint32_t *Command, uint32_t *Result, uint32_t Timeout)
{
        uint32_t DebugStatus; // data
        uint8_t status;

        status = WriteCore(SWDBRIDGE_CORE_PC, m_IAPCode | 1);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

        status = WriteMemoryArray(m_IAPCommand, Command, 5);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

        status = WriteMemory(m_IAPResult, 123); // zero return code
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

//        ReadCore(SWDBRIDGE_CORE_PC, &data);
//        ReadMemoryArray(0x10000000, Result, 2);
        status = Run();
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

        //DateTime startTime = DateTime.Now;
    //                TimeSpan ts = new TimeSpan();
    status = WriteAP(0x4, 0xE000EDF0); // Write halt control status to TAR
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

    status = ReadAP(0x10, &DebugStatus);
        if(status != SWDBRIDGE_OR_STATUS_ACK) return status;

        do
        {
            status = ReadAP(0x10, &DebugStatus);
                if(status != SWDBRIDGE_OR_STATUS_ACK) return status;
                if(DebugStatus & (1<<19)) // core is locked up
                {
                        Stop();
                        return SWDBRIDGE_OR_STATUS_FAULT;
                }
        } while(!(DebugStatus & (1 << 17)));
//        ReadCore(SWDBRIDGE_CORE_PC, &data);

        status = ReadMemoryArray(m_IAPResult, Result, 5);
        return status;
}

/**************************************************************************/
/*!

*/
/**************************************************************************/
uint8_t Connect(uint32_t *CoreID)
{
  uint32_t data;
  uint8_t status;

  swdEnable();
  status = ReadDP(0, &data);

  if ( status != SWD_ACK || (data & 1) != 1 || ((data >> 1) & 0x7FF) != 0x23B)
    return SWDBRIDGE_OR_STATUS_FAULT;

  if(CoreID)
    *CoreID = data;

  // Reset sticky error bits
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = WriteDP(0, 0x1E);

  // Turn on SWD debug port
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = ReadDP(0x4, &data);
  data |= (1 << 28) | (1 << 30); // turn on debug interface
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = WriteDP(0x4, data);

  // Read AP IDR- Mandatory or the AP will not work
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = ReadAP2(0xFC, &data);

  // Configure memory accesses for auto-increment, 32-bit mode
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = ReadAP2(0, &data);
  data &= ~0x7; data |= 0x2; // Set 32-bit access size
  data &= ~0x30; data |= 0x10; // increment single
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    status = WriteAP(0, data);
  if(status == SWDBRIDGE_OR_STATUS_ACK)
    swdFlush();
  return status;
}

volatile int8_t test_status;

/**************************************************************************/
/*!

*/
/**************************************************************************/
int swdbridgeInit (void)
{
  uint32_t i;
  uint32_t status = SWDBRIDGE_OR_STATUS_ACK;
  uint32_t ir, or; //data

  LOG_STR("//--------------------------//");
  LOG_STR("// SWD start");
  LOG_STR("//--------------------------//" CFG_PRINTF_NEWLINE);

  /* Make sure the SWD pins are setup properly */
  LPC_GPIO->DIR[SWD_PORT] |= SWD_SWDIO_BIT_MSK | SWD_SWCLK_BIT_MSK;

  /* Enable OE on the level shifter */
  LPC_GPIO->DIR[SWD_PORT] |= (1 << 25);
  LPC_GPIO->SET[SWD_PORT] = (1 << 25);

  /* Set reset high on the target MCU */
  LPC_GPIO->DIR[SWD_PORT] |= (1 << 29);
  LPC_GPIO->SET[SWD_PORT] = (1 << 29);

  while (1) /* Loop forever */
  {
    if (OutReport_readidx != OutReport_writeidx)
    {
      LOG("Process out report, readidx = %d, writeidx = %d", OutReport_readidx, OutReport_writeidx);

      or = OutReport_readidx;
      ir = InReport_writeidx;

      LED_off_counter = 10;

      InReports[ir].Status = SWDBRIDGE_OR_STATUS_NOTHING;

      LOG("Out report" CFG_PRINTF_NEWLINE
          "\tOpcode   = %d - %s" CFG_PRINTF_NEWLINE
          "\tAddress  = %d" CFG_PRINTF_NEWLINE
          "\tCount    = %d" CFG_PRINTF_NEWLINE
          "\tSequence = %d",
          OutReports[or].Opcode, SWD_OPCODE_STR[OutReports[or].Opcode], OutReports[or].Address, OutReports[or].Count, OutReports[or].Sequence);
      LOG_TAB(1);
      LOG_ARR(OutReports[or].WriteData, 15, "%08X");

      switch (OutReports[or].Opcode)
      {
        case SWDBRIDGE_OR_OPCODE_CONNECT:
          status = Connect((uint32_t *) &InReports[ir].ReadData[0]);
          InReports[ir].Status = status;
          InReports[ir].Count = 1;
        break;
        case SWDBRIDGE_OR_OPCODE_READDP:
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = ReadDP(OutReports[or].Address, (uint32_t *) &InReports[ir].ReadData[i]);
            if (status == SWDBRIDGE_OR_STATUS_ACK)
            {
              if (OutReports[or].Address == 8) // Cache read from select register
                APSelect = InReports[ir].ReadData[i];
            } else
              break;
          }
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_WRITEDP:
          for (i = 0; i < OutReports[or].Count; i++)
          {
            do
            {
              status = WriteDP(OutReports[or].Address, OutReports[or].WriteData[i]);
            } while (status == SWDBRIDGE_OR_STATUS_WAIT);
            if (status == SWDBRIDGE_OR_STATUS_ACK)
            {
              if (OutReports[or].Address == 8) // Cache write to select register
                APSelect = OutReports[or].WriteData[i];
            } else
              break;
          }
          swdFlush();
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_READAP:
          status = SWDBRIDGE_OR_STATUS_ACK;
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = ReadAP(OutReports[or].Address, (uint32_t *) &InReports[ir].ReadData[i]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        break;
        case SWDBRIDGE_OR_OPCODE_WRITEAP:
          status = SWDBRIDGE_OR_STATUS_ACK;
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = WriteAP(OutReports[or].Address, OutReports[or].WriteData[i]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          swdFlush();
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_FLUSH:
          swdFlush();
          InReports[ir].Count = 1;
          InReports[ir].Status = SWDBRIDGE_OR_STATUS_ACK;
        break;
        case SWDBRIDGE_OR_OPCODE_LED:
          LED_on = OutReports[or].WriteData[0] ? 1 : 0;
          InReports[ir].Count = 1;
          InReports[ir].Status = SWDBRIDGE_OR_STATUS_ACK;
        break;
        case SWDBRIDGE_OR_OPCODE_READMEM:
          status = SWDBRIDGE_OR_STATUS_ACK;
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = ReadMemory(OutReports[or].WriteData[i], (uint32_t *) &InReports[ir].ReadData[i]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        break;
        case SWDBRIDGE_OR_OPCODE_WRITEMEM:
          status = SWDBRIDGE_OR_STATUS_ACK;
          for (i = 0; i < OutReports[or].Count; i += 2)
          {
            status = WriteMemory(OutReports[or].WriteData[i], OutReports[or].WriteData[i + 1]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          swdFlush();
          InReports[ir].Count = i / 2;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_RUN:
          status = Run();
          InReports[ir].Count = 1;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_STOP:
          status = Stop();
          InReports[ir].Count = 1;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_RESET:
          status = Reset();
          InReports[ir].Count = 1;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_DISABLEWDT:
          /* Obsolete! */
          // status = DisableWDT();
          status = SWDBRIDGE_OR_STATUS_ACK;
          InReports[ir].Count = 1;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_READCORE:
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = ReadCore(OutReports[or].Address + i, (uint32_t *) &InReports[ir].ReadData[i]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_WRITECORE:
          for (i = 0; i < OutReports[or].Count; i++)
          {
            status = WriteCore(OutReports[or].Address + i, OutReports[ir].WriteData[i]);
            if (status != SWDBRIDGE_OR_STATUS_ACK)
              break;
          }
          swdFlush();
          InReports[ir].Count = i;
          InReports[ir].Status = status;
        break;
        case SWDBRIDGE_OR_OPCODE_SETUPINVOKE:
          InReports[ir].Status = SetupInvoke();
          InReports[ir].Count = 1;
        break;
        case SWDBRIDGE_OR_OPCODE_INVOKE:
          InReports[ir].Status = Invoke((uint32_t *) &OutReports[or].WriteData[0], (uint32_t *) &InReports[ir].ReadData[0], OutReports[or].WriteData[5]);
          InReports[ir].Count = 5; // maximum number of words returned is 5
        break;

      }
      OutReport_readidx = (OutReport_readidx + 1) % SWDBRIDGE_QUEUE_DEPTH;
      if (InReports[ir].Status != SWDBRIDGE_OR_STATUS_NOTHING)
      {
        InReports[ir].Sequence = OutReports[or].Sequence;
        InReport_writeidx = (InReport_writeidx + 1) % SWDBRIDGE_QUEUE_DEPTH;
      }
    }
  }
}

#endif

