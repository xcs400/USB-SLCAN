/*
 * board/lpc17xx_devkit/shared/board_shared.c
 * Copyright (C) 2019 xent
 * Project is distributed under the terms of the GNU General Public License v3.0
 */

#include "board_shared.h"
#include "version.h"
#include <dpm/memory/m24.h>
#include <halm/core/cortex/systick.h>
#include <halm/generic/work_queue.h>
#include <halm/platform/lpc/can.h>
#include <halm/platform/lpc/clocking.h>
#include <halm/platform/lpc/i2c.h>
#include <halm/platform/lpc/gptimer.h>
#include <halm/platform/lpc/usb_device.h>
#include <halm/platform/lpc/wdt.h>
#include <halm/usb/cdc_acm.h>
#include <halm/usb/usb.h>
#include <halm/usb/usb_string.h>
#include <string.h>
/*----------------------------------------------------------------------------*/
/* LPC17xx has 32 priority levels */

#define PRI_CHRONO  5
#define PRI_CAN     4
/* PRI_GPDMA 3 */
#define PRI_USB     2
#define PRI_I2C     1
#define PRI_TIMER   1
/* PRI_WQ_LP 0 */
/*----------------------------------------------------------------------------*/
DECLARE_WQ_IRQ(WQ_LP, SPI_ISR)
/*----------------------------------------------------------------------------*/
static void customStringHeader(const void *, enum UsbLangId,
    struct UsbDescriptor *, void *);
static void customStringWrapper(const void *, enum UsbLangId,
    struct UsbDescriptor *, void *);
/*----------------------------------------------------------------------------*/
static void customStringHeader(const void *, enum UsbLangId,
    struct UsbDescriptor *header, void *payload)
{
  usbStringHeader(header, payload, LANGID_ENGLISH_US);
}
/*----------------------------------------------------------------------------*/
static void customStringWrapper(const void *argument, enum UsbLangId,
    struct UsbDescriptor *header, void *payload)
{
  usbStringWrap(header, payload, argument);
}
/*----------------------------------------------------------------------------*/
bool boardSetupClock(void)
{
  static const struct ExternalOscConfig extOscConfig = {
      .frequency = 12000000
  };
  static const struct PllConfig sysPllConfig = {
      .divisor = 3,
      .multiplier = 25,
      .source = CLOCK_EXTERNAL
  };
  static const struct PllConfig usbPllConfig = {
      .divisor = 4,
      .multiplier = 16,
      .source = CLOCK_EXTERNAL
  };

  if (clockEnable(ExternalOsc, &extOscConfig) != E_OK)
    return false;
  while (!clockReady(ExternalOsc));

  if (clockEnable(SystemPll, &sysPllConfig) != E_OK)
    return false;
  while (!clockReady(SystemPll));

  if (clockEnable(UsbPll, &usbPllConfig) != E_OK)
    return false;
  while (!clockReady(UsbPll));

  clockEnable(UsbClock, &(struct GenericClockConfig){CLOCK_USB_PLL});
  clockEnable(MainClock, &(struct GenericClockConfig){CLOCK_PLL});

  return true;
}
/*----------------------------------------------------------------------------*/
void boardSetupDefaultWQ(void)
{
  static const struct WorkQueueConfig wqConfig = {
      .size = 3
  };

  WQ_DEFAULT = init(WorkQueue, &wqConfig);
}
/*----------------------------------------------------------------------------*/
void boardSetupLowPriorityWQ(void)
{
  static const struct WorkQueueIrqConfig wqConfig = {
      .size = 1,
      .irq = SPI_IRQ,
      .priority = 0
  };

  WQ_LP = init(WorkQueueIrq, &wqConfig);
}
/*----------------------------------------------------------------------------*/
struct Interface *boardMakeCan(void)
{
  static const struct CanConfig canConfig = {
      .rate = 10000,
      .rxBuffers = 32,
      /* TX buffer count should be at least SERIALIZED_QUEUE_SIZE */
      .txBuffers = 32,
      .rx = PIN(0, 0),
      .tx = PIN(0, 1),
      .priority = PRI_CAN,
      .channel = 0
  };

  return init(Can, &canConfig);
}
/*----------------------------------------------------------------------------*/
struct Timer *boardMakeChronoTimer(void)
{
  static const struct GpTimerConfig chronoTimerConfig = {
      .frequency = 1000000,
      .priority = PRI_CHRONO,
      .channel = 0
  };

  return init(GpTimer, &chronoTimerConfig);
}
/*----------------------------------------------------------------------------*/
struct Timer *boardMakeEventTimer(void)
{
  return init(SysTick, &(struct SysTickConfig){PRI_TIMER});
}
/*----------------------------------------------------------------------------*/
struct Timer *boardMakeMemoryTimer(void)
{
  static const struct GpTimerConfig eepromTimerConfig = {
      .frequency = 1000000,
      .priority = PRI_I2C,
      .channel = 1
  };

  return init(GpTimer, &eepromTimerConfig);
}
/*----------------------------------------------------------------------------*/
struct Interface *boardMakeI2C(void)
{
  static const struct I2CConfig i2cConfig = {
      .rate = 400000,
      .scl = PIN(0, 11),
      .sda = PIN(0, 10),
      .priority = PRI_I2C,
      .channel = 2
  };

  return init(I2C, &i2cConfig);
}
/*----------------------------------------------------------------------------*/
struct Interface *boardMakeSerial(struct Usb *usb)
{
  /* CDC */
  const struct CdcAcmConfig config = {
      .device = usb,
      .rxBuffers = 4,
      .txBuffers = 8,

      .endpoints = {
          .interrupt = 0x81,
          .rx = 0x02,
          .tx = 0x82
      }
  };

  return init(CdcAcm, &config);
}
/*----------------------------------------------------------------------------*/
struct Usb *boardMakeUsb(void)
{
  static const struct UsbDeviceConfig usbConfig = {
      .dm = PIN(0, 30),
      .dp = PIN(0, 29),
      .connect = PIN(2, 9),
      .vbus = PIN(1, 30),
      .vid = 0x2404,
      .pid = 0x03EB,
      .priority = PRI_USB,
      .channel = 0
  };

  return init(UsbDevice, &usbConfig);
}
/*----------------------------------------------------------------------------*/
void boardMakeUsbStrings(struct Usb *usb, const char *number)
{
  const IrqState state = irqSave();

  /* USB Strings */
  usbDevStringAppend(usb, usbStringBuild(customStringHeader, nullptr,
      USB_STRING_HEADER, 0));

  if (strlen(getUsbVendorString()) > 0)
  {
    usbDevStringAppend(usb, usbStringBuild(customStringWrapper,
        getUsbVendorString(), USB_STRING_VENDOR, 0));
  }
  if (strlen(getUsbProductString()) > 0)
  {
    usbDevStringAppend(usb, usbStringBuild(customStringWrapper,
        getUsbProductString(), USB_STRING_PRODUCT, 0));
  }
  if (number != nullptr && strlen(number) > 0)
  {
    usbDevStringAppend(usb, usbStringBuild(customStringWrapper,
        number, USB_STRING_SERIAL, 0));
  }

  irqRestore(state);
}
/*----------------------------------------------------------------------------*/
struct Watchdog *boardMakeWatchdog(void)
{
  static const struct WdtConfig wdtConfig = {
      .period = 1000
  };

  return init(Wdt, &wdtConfig);
}
/*----------------------------------------------------------------------------*/
bool boardSetupMemoryPackage(struct MemoryPackage *package)
{
  struct Interface * const i2c = boardMakeI2C();
  struct Timer * const timer = boardMakeMemoryTimer();

  if (i2c == nullptr || timer == nullptr)
    return false;

  const struct M24Config memoryConfig = {
      .bus = i2c,
      .timer = timer,
      .address = 0x50,
      .chipSize = 65536,
      .pageSize = 32,
      .rate = 0,
      .blocks = 1
  };
  struct Interface * const memory = init(M24, &memoryConfig);

  if (memory != nullptr)
  {
    m24SetUpdateWorkQueue(memory, WQ_LP);

    package->i2c = i2c;
    package->timer = timer;
    package->memory = memory;
    return true;
  }
  else
    return false;
}
