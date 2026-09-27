/*
 * board/blackboard/application/main.c
 * Copyright (C) 2026 xent
 * Project is distributed under the terms of the GNU General Public License v3.0
 */

#include "board.h"
#include <stdlib.h>
/*----------------------------------------------------------------------------*/
int main(void)
{
  struct Board * const board = malloc(sizeof(struct Board));

  appBoardInit(board);
  return appBoardStart(board);
}
