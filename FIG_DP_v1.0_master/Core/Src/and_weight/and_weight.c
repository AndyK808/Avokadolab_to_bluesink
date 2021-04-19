#include "and_weight.h"

#define ASCII_CR 0x0D
#define ASCII_LF 0x0A

uint8_t andCommandBuffer[AND_COMMAND_SIZE];
uint8_t andResponseBuffer[AND_RESPONSE_SIZE];

// 9.3.1. Operation command
// RW CRLF
bool require_data(){
  andCommandBuffer[0] = 'R';
  andCommandBuffer[1] = 'W';
  andCommandBuffer[2] = ASCII_CR;
  andCommandBuffer[3] = ASCII_LF;
}

// MZT CRLF
bool scale_to_zero(uint8_t andCommandBuffer[5]){
  andCommandBuffer[0] = 'M';
  andCommandBuffer[1] = 'Z';
  andCommandBuffer[2] = 'T';
  andCommandBuffer[3] = ASCII_CR;
  andCommandBuffer[4] = ASCII_LF;
}

// CT CRLF
bool clear_tare(){
}

// MG CRLF
bool change_to_gross_weight(){
}

// MN CRLF
bool change_to_net_weight(){
}

// 9.3.2. Calibration command
// CALZERO CRLF
bool calibration_zero(){
}

// CALSPAN CRLF
bool calibration_span(){
}

// 9.3.3. Function command
bool set_f_function_setting(){
}

bool get_f_function_setting(){
}

bool set_cf_function_setting(){
}

bool get_cf_function_setting(){
}

// 9.3.4. Rest command
bool get_software_version(){
}

// 9.3.5. Initialization command
// EER CRLF (2400bps, 8bit, Parity none, stop 1bit, CRLF, stream mode)
bool reset_serial_setting(){
}

// INC CRLF
bool reset_all_setting(){
}

// INF CRLF
bool reset_function(){
}



// S T , G S , + 0 0 1 2 3 4 5 k g CR(\r) lF(\n)
// S T , G S , + 0 1 2 3 4 . 5 () g CR(\r) lF(\n)
// U S , G S , + 0 1 2 3 . 4 5 () g CR(\r) lF(\n)
AND_DATA_FORMAT andData;
uint8_t andRxBuffer[AND_RX_BUFFSIZE];

bool parse_andData(){
  static uint8_t temp[8];
  static AND_DATA_FORMAT andData_temp;
  
  uint8_t i, j;
  bool found_terminators = false;
  
  int32_t weight_high, weight_low;
  bool is_neg, is_decimal;
  
  char* x = (char*)&temp;
  
  // 1. Find frame between CR(0x0D) LF(0x0A)
  for(i = 0; i < AND_RX_BUFFSIZE; i++){
    if(andRxBuffer[i] == ASCII_CR){
      if(andRxBuffer[mod_buff(i + 1)] == ASCII_LF){
        found_terminators = true;
        break;
      }
    }
  }
  if(!found_terminators){
    return false;
  }
  
  // 2. Check data frame briefly
  i = mod_buff(i - 16);
  if(andRxBuffer[mod_buff(i + 2)] != ',' || andRxBuffer[mod_buff(i + 5)] != ','){
    return false;
  }
  
  // 3. Check Data Header1
  temp[0] = andRxBuffer[i];
  temp[1] = andRxBuffer[mod_buff(i + 1)];
  if(!strncmp(x, "OL", 2)){
    temp[2] = andRxBuffer[mod_buff(i + 6)];
    if(!strncmp(x, "OL+", 3)){
      andData_temp.header1 = AND_OVERLOAD;
    }
    else if(!strncmp(x, "OL-", 3)){
      andData_temp.header1 = AND_UNDERLOAD;
    }
    else{
      return false;
    }
  }
  else if(!strncmp(x, "ST", 2)){
    andData_temp.header1 = AND_STABLE;
  }
  else if(!strncmp(x, "US", 2)){
    andData_temp.header1 = AND_UNSTABLE;
  }
  else if(!strncmp(x, "HD", 2)){
    andData_temp.header1 = AND_HOLD;
  }
  else{
    return false;
  }
  
  // 4. Check Data Header2
  temp[0] = andRxBuffer[mod_buff(i + 3)];
  temp[1] = andRxBuffer[mod_buff(i + 4)];
  if(!strncmp(x, "NT", 2)){
    andData_temp.header2 = AND_NET_WEIGHT;
  }
  else if(!strncmp(x, "GS", 2)){
    andData_temp.header2 = AND_GROSS_WEIGHT;
  }
  else if(!strncmp(x, "TR", 2)){
    andData_temp.header2 = AND_TARE;
  }
  else{
    return false;
  }
  
  // 5. Check Weight Data
  is_neg = false;
  is_decimal = false;
  for(j = 0; j < 8; j++){
    temp[j] = andRxBuffer[mod_buff(i + 6 + j)];
  }
  switch(temp[0]){
    case '+' : break;
    case '-' : is_neg = true; break;
    case ' ' : break;
    case '.' : is_decimal = true; break;
    default : return false;
  }
  
  weight_high = 0;
  weight_low = 0;
  for(j = 1; j < 8; j++){
    if('0' <= temp[j] && temp[j] <= '9'){
      if(is_decimal){
        weight_low = weight_low * 10 + (temp[j] - '0');
      }
      else{
        weight_high = weight_high * 10 + (temp[j] - '0');
      }
    }
    else if(temp[j] == '.'){
      is_decimal = true;
    }
//    else{
//      return false;
//    }
  }
  
  // decimal point
  andData_temp.data = (float) weight_low;
  while(andData_temp.data > 1.0f){
    andData_temp.data /= 10.0f;
  }
  // integer
  andData_temp.data += (float) weight_high;
  // sign
  if(is_neg){
    andData_temp.data *= -1.0f;
  }
  
  // 6. Check Units
  temp[0] = andRxBuffer[mod_buff(i + 14)];
  temp[1] = andRxBuffer[mod_buff(i + 15)];
  if(!strncmp(x, " g", 2)){
    andData_temp.unit = AND_UINT_g;
  }
  else if(!strncmp(x, "kg", 2)){
    andData_temp.unit = AND_UNIT_Kg;
  }
  else if(!strncmp(x, " t", 2)){
    andData_temp.unit = AND_UINT_t;
  }
  else{
    return false;
  }
  
  // 7. Copy data & erase frame from buffer
  for(j = 0; j < 18; j++){
    andRxBuffer[mod_buff(i + j)] = 0x00;
  }
  andData_temp.length = 18;
  memcpy(&andData, &andData_temp, sizeof(AND_DATA_FORMAT));
  
  return true;
}
