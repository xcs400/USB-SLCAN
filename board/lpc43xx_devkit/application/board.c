/*
 * board/lpc43xx_devkit/application/board.c
 * Copyright (C) 2023 xent
 * Project is distributed under the terms of the GNU General Public License v3.0
 */

#include "board.h"
#include "dfu_defs.h"
#include "led_indicator.h"
#include "proxy_port.h"
#include <halm/core/cortex/nvic.h>
#include <halm/delay.h>
#include <halm/platform/lpc/backup_domain.h>
#include <halm/usb/usb.h>
#include <halm/watchdog.h>
/*----------------------------------------------------------------------------*/
#define EVENT_RATE    50
#define MAX_BLINKS    16

#define MEMORY_OFFSET 0
/*----------------------------------------------------------------------------*/
static void onConfigLoaded(void *, bool);
static void panic(struct Pin, struct Watchdog *);
/*----------------------------------------------------------------------------*/
static const struct LedIndicatorConfig errorLedConfig = {
    .pin = BOARD_LED_ERROR,
    .limit = MAX_BLINKS,
    .inversion = false
};

static const struct LedIndicatorConfig portLedConfig = {
    .pin = BOARD_LED_BUSY,
    .limit = MAX_BLINKS,
    .inversion = true
};
/*----------------------------------------------------------------------------*/
static void onConfigLoaded(void *argument, bool)
{
  struct Board * const board = argument;

  makeSerialNumber(board->number, board->config.serial);
  boardMakeUsbStrings(board->usb, board->number);

  if (board->config.initial[0] != -1)
  {
    proxyPortChangeMode(&board->hub->ports[0], SLCAN_MODE_ACTIVE,
        slcanRatePresetToValue((unsigned int)board->config.initial[0]));
  }

  usbDevSetConnected(board->usb, true);
}
/*----------------------------------------------------------------------------*/
static void panic(struct Pin led, struct Watchdog *watchdog)
{
  while (1)
  {
    if (watchdog != nullptr)
      watchdogReload(watchdog);

    pinToggle(led);
    mdelay(500);
  }
}
/*----------------------------------------------------------------------------*/
void appBoardInit(struct Board *board)
{
  struct Pin led = pinInit(BOARD_LED_ERROR);
  pinOutput(led, false);

  if (!boardSetupClock())
    panic(led, nullptr);

#ifdef ENABLE_WDT
  board->watchdog = boardMakeWatchdog();
  if (board->watchdog == nullptr)
    panic(led, nullptr);
#else
  board->watchdog = nullptr;
#endif

  /* Initialize Work Queues */

  boardSetupDefaultWQ();
  boardSetupLowPriorityWQ();
  if (WQ_DEFAULT == nullptr || WQ_LP == nullptr)
    panic(led, board->watchdog);

  /* Timers */

  board->chronoTimer = boardMakeChronoTimer();
  if (board->chronoTimer == nullptr)
    panic(led, board->watchdog);

  board->eventTimer = boardMakeEventTimer();
  if (board->eventTimer == nullptr)
    panic(led, board->watchdog);

  /* CAN */

  board->can = boardMakeCan();
  if (board->can == nullptr)
    panic(led, board->watchdog);

  /* USB */

  board->usb = boardMakeUsb();
  if (board->usb == nullptr)
    panic(led, board->watchdog);
  board->serial = boardMakeSerial(board->usb);
  if (board->serial == nullptr)
    panic(led, board->watchdog);

  /* I2C and parameter storage, start Low-Priority Work Queue */

  if (!boardSetupMemoryPackage(&board->memoryPackage))
    panic(led, board->watchdog);

  settingsInit(&board->configContext, board->memoryPackage.memory,
      MEMORY_OFFSET, &board->config, sizeof(board->config));
  settingsLoadDefault(&board->config);

  /* Indication */

  board->error = init(LedIndicator, &errorLedConfig);
  if (board->error == nullptr)
    panic(led, board->watchdog);
  board->status = init(LedIndicator, &portLedConfig);
  if (board->status == nullptr)
    panic(led, board->watchdog);

  /* Create port hub and initialize all ports */

  board->hub = makeProxyHub(1);
  if (board->hub == nullptr)
    panic(led, board->watchdog);

  const struct ProxyPortConfig proxyPortConfig = {
      .can = board->can,
      .serial = board->serial,
      .chrono = board->chronoTimer,
      .error = board->error,
      .status = board->status,
      .settings = &board->configContext,
      .number = SLCAN_PORT_1
  };
  if (!proxyPortInit(&board->hub->ports[0], &proxyPortConfig))
    panic(led, board->watchdog);
}
/*----------------------------------------------------------------------------*/
int appBoardStart(struct Board *board)
{
  timerSetOverflow(board->eventTimer,
      timerGetFrequency(board->eventTimer) / EVENT_RATE);

  timerEnable(board->chronoTimer);
  timerEnable(board->eventTimer);

  wqStart(WQ_LP);
  settingsLoad(&board->configContext, onConfigLoaded, board);

  wqStart(WQ_DEFAULT);
  return 0;
}
/*----------------------------------------------------------------------------*/
void resetToBootloader(void)
{
  *(uint32_t *)backupDomainAddress() = DFU_START_REQUEST;
  nvicResetCore();
}
