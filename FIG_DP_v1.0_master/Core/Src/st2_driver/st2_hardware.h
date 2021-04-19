#ifndef __ST2_HARDWARE_H__
#define __ST2_hARDWARE_H__

// NK245-01AT
#define TABLE_LEN 10

static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.2},
  {200, 0.16},
  {400, 0.2},
  {450, 0.24},
  {550, 0.28},
  {600, 0.32},
  {780, 0.36},
  {1000, 0.38},
  {1500, 0.4},
  {2000, 0.42},
};

#endif
