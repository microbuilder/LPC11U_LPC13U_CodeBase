#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <math.h>

#include "projectconfig.h"
#include "core/systick/systick.h"
#include "boards/board.h"
#include "messages.h"

#ifdef CFG_INTERFACE
  #include "cli/cli.h"
#endif

#ifdef CFG_USB
  #include "core/usb/usbd.h"
  #ifdef CFG_USB_HID_GENERIC
    /* Buffer to hold incoming HID data */
    static USB_HID_GenericReport_t hid_out_report;
    static bool is_received_report = false;
  #endif
#endif

#ifdef CFG_CHIBI
  #include "drivers/rf/chibi/chb.h"
  #include "drivers/rf/chibi/chb_drvr.h"
  static chb_rx_data_t rx_data;
#endif

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

#include "drivers/sensors/accelerometers/lsm303accel.h"
#include "drivers/sensors/magnetometers/lsm303mag.h"
#include "drivers/sensors/pressure/bmp085.h"

#ifdef CFG_CHIBI
/**************************************************************************/
/*!
    Converts the ED (Energy Detection) value to dBm using the following
    formula: dBm = RSSI_BASE_VAL + 1.03 * ED

    For more information see section 6.5 of the AT86RF212 datasheet
*/
/**************************************************************************/
int edToDBM(uint32_t ed)
{
  #if CFG_CHIBI_MODE == 0 || CFG_CHIBI_MODE == 1 || CFG_CHIBI_MODE == 2
    // Calculate for OQPSK (RSSI Base Value = -100)
    int dbm = (103 * ed - 10000);
  #else
    // Calculate for BPSK (RSSI Base Value = -98)
    int dbm = (103 * ed - 9800);
  #endif

  return dbm / 100;
}

void sendMessage(void)
{
  uint8_t msgbuf[10] = { 0, 1, 2, 3, 4, 5, 6, 7, 8, 9 };
  if(msgSend(0xFFFF, MSG_MESSAGETYPE_NONE, msgbuf, 10))
  {
    printf("Message TX failure%s", CFG_PRINTF_NEWLINE);
  }
}

void checkForMessages(void)
{
  chb_pcb_t *pcb = chb_get_pcb();

  while (pcb->data_rcv)
  {
    // Enable LED to indicate message reception
    boardLED(CFG_LED_ON);
    // get the length of the data
    rx_data.len = chb_read(&rx_data);
    // make sure the length is nonzero
    if (rx_data.len)
    {
      int dbm = edToDBM(pcb->ed);
      // printf("Message received from node %02X: %s, len=%d, dBm=%d.%s", rx_data.src_addr, rx_data.data, rx_data.len, dbm, CFG_PRINTF_NEWLINE);
      printf("Message received from node 0x%04X (len=%d, dBm=%d):%s", rx_data.src_addr, rx_data.len, dbm, CFG_PRINTF_NEWLINE);
      printf("  Message ID:   0x%04X%s", *(uint16_t*)&rx_data.data[0], CFG_PRINTF_NEWLINE);
      printf("  Message Type: 0x%02X%s", *(uint8_t*)&rx_data.data[2], CFG_PRINTF_NEWLINE);
      printf("  Timestamp:    %d%s", *(uint32_t*)&rx_data.data[3], CFG_PRINTF_NEWLINE);
      printf("  Payload:      %d bytes%s", *(uint8_t*)&rx_data.data[8], CFG_PRINTF_NEWLINE);
      if (rx_data.data[8])
      {
        uint8_t i;
        printf("%s", CFG_PRINTF_NEWLINE);
        for (i = 0; i < rx_data.data[8]; i++)
        {
          printf("0x%02X ", *(uint8_t*)&rx_data.data[9+i]);
        }
      }
      printf("%s", CFG_PRINTF_NEWLINE);
    }
    // Disable LED
    boardLED(CFG_LED_OFF);
  }
}
#endif

void blinkyError(void)
{
  while(1)
  {
    boardLED(CFG_LED_ON);
    systickDelay(100);
    boardLED(CFG_LED_OFF);
    systickDelay(100);
  }
}

int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  /* Configure common and board-level peripherals */
  boardInit();

  while (1)
  {
    currentSecond = systickGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      lastSecond = currentSecond;

      /* Display any incoming HID data if HID GENERIC is enabled */
      #ifdef CFG_USB_HID_GENERIC
        if(usb_isConfigured())
        {
          if(is_received_report)
          {
            uint32_t i;
            for (i=0; i< sizeof(USB_HID_GenericReport_t); i++)
            {
              printf("%02x ", hid_out_report.report[i]);
            }
            printf(CFG_PRINTF_NEWLINE);
            is_received_report = false;
          }
        }
      #endif

      /* Send a message over the air */
      #ifdef CFG_CHIBI
        sendMessage();
      #endif

      /* Do blinky if we're not sending data wirelessly */
      #if !defined(CFG_CHIBI)
        boardLED(lastSecond % 2);
      #endif
    }

    /* Poll for CLI input if CFG_INTERFACE is enabled */
    #ifdef CFG_INTERFACE
      cliPoll();
    #endif

    /* Check for incoming wireless messages */
    #ifdef CFG_CHIBI
      checkForMessages();
    #endif
  }

  return 0;
}

#ifdef CFG_USB_HID_GENERIC
void usb_hid_generic_recv_isr(USB_HID_GenericReport_t *out_report)
{
  /* Copy out_report to a buffer in case new data comes in */
  memcpy(&hid_out_report, out_report, sizeof(USB_HID_GenericReport_t));
  is_received_report = true;
}
#endif
