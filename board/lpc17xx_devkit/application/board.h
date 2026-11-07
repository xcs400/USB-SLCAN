/*
 * board/lpc17xx_devkit/application/board.h
 * Copyright (C) 2019 xent
 * Project is distributed under the terms of the GNU General Public License v3.0
 */

#ifndef BOARD_LPC17XX_DEVKIT_APPLICATION_BOARD_H_
#define BOARD_LPC17XX_DEVKIT_APPLICATION_BOARD_H_
/*----------------------------------------------------------------------------*/
#include "board_shared.h"
#include "settings_project.h"
/*----------------------------------------------------------------------------*/
struct ProxyHub;

struct Board
{
  struct Interface *can;
  struct Interface *serial;
  struct Timer *chronoTimer;
  struct Timer *eventTimer;
  struct Watchdog *watchdog;

  struct Indicator *error;
  struct Indicator *status;
  struct ProxyHub *hub;

  struct MemoryPackage memoryPackage;
  struct SettingsContext configContext;
  struct Settings config;
};
/*----------------------------------------------------------------------------*/
void appBoardInit(struct Board *);
int appBoardStart(struct Board *);
/*----------------------------------------------------------------------------*/
#endif /* BOARD_LPC17XX_DEVKIT_APPLICATION_BOARD_H_ */
