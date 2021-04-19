#ifndef __ST5_HARDWARE_H__
#define __ST5_HARDWARE_H__


// A140K-M599-G5 (2020 09)

// A10K-S545-G5 (2020 12 Rotary)

// A2K-S544W (Rated : 1.65v / Break : 0.44v(1.8kgf.cm))
// rated voltage = 0.06875 (1.65V)
#define TABLE_LEN 9
static float motor_table[TABLE_LEN][2] = {
  // {SPEED, DUTY}
  {0, 0.2},
  {50, 0.12},
  {250, 0.14},
  {500, 0.16},
  {800, 0.18},
  {1000, 0.25},
  {1300, 0.24},
  {1500, 0.26},
  {2000, 0.28}
};

#endif
