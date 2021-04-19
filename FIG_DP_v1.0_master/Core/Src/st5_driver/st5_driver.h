#ifndef __ST5_DRIVER_H__
#define __ST5_DRIVER_H__

#include <stdint.h>
#include <stdbool.h>

typedef struct _ST5_DRIVER{
  int32_t current_cnt;
  int32_t input_cnt;
  int32_t speed;
}ST5_DRIVER;

extern ST5_DRIVER st5;
extern bool st5_eternal_rotate;

extern void ST5_Init(uint16_t freq);
extern void ST5_Loop();
extern void ST5_getPWM(float pwmDuty[10]);
#endif
