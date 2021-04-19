#include "st5_driver.h"
#include "st5_hardware.h"



// ======================== internal values & functions ========================

// pentagon 4phase full step exitation

#define FULL_STEP 10  // FULL_STEP MODE
//#define HALF_STEP 20 // HALF_STEP MODE

#ifdef FULL_STEP

  #define STEP_5_VALUE_COL 10

  static uint8_t step_num = FULL_STEP;

  static uint8_t step5Value[FULL_STEP][STEP_5_VALUE_COL] =
  {
    // FULL legnth version : 24 24 24 0 0
    {1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
    {1, 1, 0, 0, 0, 0, 0, 0, 1, 0},
    {0, 1, 0, 0, 0, 0, 0, 0, 1, 1},
    {0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
    {0, 0, 1, 0, 0, 1, 0, 0, 0, 1},
    {0, 0, 1, 1, 0, 1, 0, 0, 0, 0},
    {0, 0, 0, 1, 0, 1, 1, 0, 0, 0},
    {0, 0, 0, 1, 1, 0, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, 0, 1, 1, 0, 0},
    {1, 0, 0, 0, 1, 0, 0, 1, 0, 0}
    
  };
#endif

#ifdef HALF_STEP
  
  #define STEP_5_VALUE_COL 20
  
  static uint8_t step_num = HALF_STEP;

  static uint8_t step5Value[HALF_STEP][STEP_5_VALUE_COL] =
  {
    // HALF legnth version
     {1, 0, 0, 0, 0, 0, 0, 1, 1, 0},
     {1, 0, 0, 0, 0, 0, 0, 0, 1, 0},
     {1, 1, 0, 0, 0, 0, 0, 0, 1, 0},
     {0, 1, 0, 0, 0, 0, 0, 0, 1, 0},
     {0, 1, 0, 0, 0, 0, 0, 0, 1, 1},
     {0, 1, 0, 0, 0, 0, 0, 0, 0, 1},
     {0, 1, 1, 0, 0, 0, 0, 0, 0, 1},
     {0, 0, 1, 0, 0, 0, 0, 0, 0, 1},
     {0, 0, 1, 0, 0, 1, 0, 0, 0, 1},
     {0, 0, 1, 0, 0, 1, 0, 0, 0, 0},
     {0, 0, 1, 1, 0, 1, 0, 0, 0, 0},
     {0, 0, 0, 1, 0, 1, 0, 0, 0, 0},
     {0, 0, 0, 1, 0, 1, 1, 0, 0, 0},
     {0, 0, 0, 1, 0, 0, 1, 0, 0, 0},
     {0, 0, 0, 1, 1, 0, 1, 0, 0, 0},
     {0, 0, 0, 0, 1, 0, 1, 0, 0, 0},
     {0, 0, 0, 0, 1, 0, 1, 1, 0, 0},
     {0, 0, 0, 0, 1, 0, 0, 1, 0, 0},
     {1, 0, 0, 0, 1, 0, 0, 1, 0, 0},
     {1, 0, 0, 0, 0, 0, 0, 1, 0, 0}
  };
#endif

static uint8_t st5_step = 0;
static float st5_actual_speed;
static float st5_currentCnt_float;
static float st5_duty_cycle;
static float st5_period;

static void ST5_goNext(bool dir){

  if(dir){
    st5_step += 1;
  }
  else{
    st5_step += step_num - 1;
  }
  st5_step %= step_num;
}

static void ST5_interPolate(int32_t speed){

  uint8_t i;
  float x1, x2;
  float y1, y2;

  if(speed < 0) speed = -speed;

  for(i = 0; i < TABLE_LEN; i++){
    if(speed < motor_table[i][0]) break;
  }

  // Interpolate
  if(i == 0){
    st5_duty_cycle = motor_table[0][1];
    return;
  }else if(i >= TABLE_LEN - 1){
    st5_duty_cycle = motor_table[TABLE_LEN - 1][1];
    return;
  }

  x1 = motor_table[i-1][0];
  x2 = motor_table[i][0];
  y1 = motor_table[i-1][1];
  y2 = motor_table[i][1];

  st5_duty_cycle = (y2 - y1) / (x2 - x1) * (speed - x1) + y1;
}

// ======================== internal values & functions ========================



// ======================== external values & functions ========================

ST5_DRIVER st5;
bool st5_eternal_rotate;

void ST5_Init(uint16_t frequency){

  st5_period = 1.0f / (float)frequency;
  st5_currentCnt_float = 0.0f;
}

void ST5_Loop(){ // Should be called periodically

  static int32_t cnt_prev;

  // velocity = distance / time
  st5_actual_speed = ((float)st5.input_cnt - st5_currentCnt_float) / st5_period;

  // user-input speed validation
  if(st5.speed < 0){
    st5.speed = -st5.speed;
  }
  if(motor_table[TABLE_LEN - 1][0] < st5.speed){
    st5.speed = (int32_t)motor_table[TABLE_LEN - 1][0];
  }

  // actual speed validation
  if(st5_actual_speed > st5.speed){
    st5_actual_speed = st5.speed;
  }
  else if(st5_actual_speed < -st5.speed){
    st5_actual_speed = -st5.speed;
  }

  st5_currentCnt_float += st5_actual_speed * st5_period;
  cnt_prev = st5.current_cnt;
  st5.current_cnt = (int32_t)st5_currentCnt_float;

  if(cnt_prev != st5.current_cnt){
    ST5_goNext(st5_actual_speed >= 0.0f);
    if(st5_eternal_rotate){
      // counter value up/down depending on a direction.
      if(st5.input_cnt > 0){
        st5_currentCnt_float -= 1;
      }
      else{
        st5_currentCnt_float += 1;
      }
    }
  }
}

void ST5_getPWM(float pwmDuty[]){

  uint8_t* pinValue = step5Value[st5_step];

  ST5_interPolate((int32_t)st5_actual_speed); // Calculate proper duty

  pwmDuty[0] = pinValue[0] ? st5_duty_cycle : 0; // VOHGA
  pwmDuty[1] = pinValue[1] ? st5_duty_cycle : 0; // VOHGB
  pwmDuty[2] = pinValue[2] ? st5_duty_cycle : 0; // VOHGC
  pwmDuty[3] = pinValue[3] ? st5_duty_cycle : 0; // VOHGD
  pwmDuty[4] = pinValue[4] ? st5_duty_cycle : 0; // VOHGE
  pwmDuty[5] = pinValue[5] ? 1 : 0; // VOLA
  pwmDuty[6] = pinValue[6] ? 1 : 0; // VOLB
  pwmDuty[7] = pinValue[7] ? 1 : 0; // VOLC
  pwmDuty[8] = pinValue[8] ? 1 : 0; // VOLD
  pwmDuty[9] = pinValue[9] ? 1 : 0; // VOLE
}

// ======================== external values & functions ========================
