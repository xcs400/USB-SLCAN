/*
 * board/lpc17xx_devkit/application/main.c
 * Copyright (C) 2018 xent
 * Project is distributed under the terms of the GNU General Public License v3.0
 */

#include "board.h"
#include <halm/delay.h>
#include <halm/pin.h>
#include <xcore/interface.h>
#include <stdlib.h>
/*----------------------------------------------------------------------------*/
int main(void)
{
  struct Board * const board = malloc(sizeof(struct Board));

  appBoardInit(board);

  struct Pin led = pinInit(PIN(1, 18));
  pinOutput(led, false);
  for (unsigned int i = 0; i < 200; ++i)
  {
    pinToggle(led);
    mdelay(50);
  }

  static const char startupMessage[] = "USB-CAN MBED UART0 ready\r\n";
  ifWrite(board->serial, startupMessage, sizeof(startupMessage) - 1);

  return appBoardStart(board);
}
