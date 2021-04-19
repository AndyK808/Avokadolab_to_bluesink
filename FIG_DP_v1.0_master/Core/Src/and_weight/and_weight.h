#ifndef __AND_WEIGHT_H__
#define __AND_WEIGHT_H__

#include <stdint.h>
#include <stdbool.h>


#define AND_COMMAND_SIZE 18
#define AND_RESPONSE_SIZE 18
extern uint8_t andCommandBuffer[AND_COMMAND_SIZE];
extern uint8_t andResponseBuffer[AND_RESPONSE_SIZE];
extern bool require_data();


typedef enum _AND_HEADER1{
 AND_OVERLOAD,
 AND_UNDERLOAD,
 AND_STABLE,
 AND_UNSTABLE,
 AND_HOLD
}AND_HEADER1;

typedef enum _AND_HEADER2{
 AND_NET_WEIGHT,
 AND_GROSS_WEIGHT,
 AND_TARE
}AND_HEADER2;

typedef enum _AND_UNIT{
  AND_UINT_g,
  AND_UNIT_Kg,
  AND_UINT_t,
}AND_UNIT;

typedef enum _AND_TERMINATOR{
  AND_CRLF
}AND_TERMINATOR;

typedef struct _AND_DATA_FORMAT{
  AND_HEADER1 header1;
  AND_HEADER2 header2;
  float data;
  AND_UNIT unit;
//  AND_TERMINATOR terminator;
  uint8_t length;       // 18
}AND_DATA_FORMAT;

#define AND_RX_BUFFSIZE 100
#define mod_buff(i) (((i) + AND_RX_BUFFSIZE) % AND_RX_BUFFSIZE)

extern AND_DATA_FORMAT andData;
extern uint8_t andRxBuffer[AND_RX_BUFFSIZE];
extern bool parse_andData();


#endif
