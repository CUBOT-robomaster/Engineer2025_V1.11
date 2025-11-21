#ifndef __INTERBOARD_H
#define __INTERBOARD_H

#include "motor.h"
#include "dr16.h"


void board_conmmunity();
void BoardCallBack(CAN_Object canx, CAN_RxBuffer rxBuffer);

extern int8_t speed_sence;
extern int8_t move_x_ok;


#endif