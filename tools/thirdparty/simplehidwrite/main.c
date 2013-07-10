#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "projectconfig.h"
#include "sysinit.h"
#include "core/systick/systick.h"
#include "boards/board.h"

#ifdef CFG_USB
  #include "core/usb/usbd.h"
#endif

#if defined(__CODE_RED)
  #include <cr_section_macros.h>
  #include <NXP/crp.h>
  __CRP const unsigned int CRP_WORD = CRP_NO_CRP ;
#endif

int main(void)
{
  uint32_t currentSecond, lastSecond;
  currentSecond = lastSecond = 0;

  /* Configure board-level peripherals */
  boardInit();

  while (1)
  {
    currentSecond = systickGetSecondsActive();
    if (currentSecond != lastSecond)
    {
      /* Toggle LED once per second */
      lastSecond = currentSecond;
      boardLED(lastSecond % 2);

      USB_HID_GenericReport_t in_report = { .report = {currentSecond % 100} };
      usb_hid_generic_send(&in_report);
    }
  }

  return 0;
}

#ifdef CFG_USB_HID_GENERIC
void usb_hid_generic_recv_isr(USB_HID_GenericReport_t *out_report)
{
  uint32_t i;
  for (i=0; i< sizeof(USB_HID_GenericReport_t); i++)
  {
    printf("%02x ", out_report->report[i]);
  }
  printf(CFG_PRINTF_NEWLINE);
}
#endif
