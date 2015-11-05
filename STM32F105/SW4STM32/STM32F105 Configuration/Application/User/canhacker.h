#ifndef _CANHACKER_H
#define _CANHACKER_H

#include <stdint.h>
#include "cmsis_os.h"

#define UART_RX_BUFFER_SIZE 512
#define UART_TX_BUFFER_SIZE 512

#define HW_VER        0x10      // hardware version
#define SW_VER        0x10      // software version
#define SW_VER_MAJOR  0x01    // software major version
#define SW_VER_MINOR  0x07    // software minor version
#define SERIAL        "0001"    // device serial number
#define CANHACKER_CR 13
#define CANHACKER_ERROR 7
#define SET_BITRATE     'S' // set CAN bit rate
#define SET_BTR         's' // set CAN bit rate via
#define OPEN_CAN_CHAN   'O' // open CAN channel
#define CLOSE_CAN_CHAN  'C' // close CAN channel
#define SEND_11BIT_ID   't' // send CAN message with 11bit ID
#define SEND_29BIT_ID   'T' // send CAN message with 29bit ID
#define SEND_R11BIT_ID  'r' // send CAN remote message with 11bit ID
#define SEND_R29BIT_ID  'R' // send CAN remote message with 29bit ID
#define READ_STATUS     'F' // read status flag byte
#define SET_ACR         'M' // set Acceptance Code Register
#define SET_AMR         'm' // set Acceptance Mask Register
#define GET_VERSION     'V' // get hardware and software version
#define GET_SW_VERSION  'v' // get software version only
#define GET_SERIAL      'N' // get device serial number
#define TIME_STAMP      'Z' // toggle time stamp setting
#define READ_ECR        'E' // read Error Capture Register
#define READ_ALCR       'A' // read Arbritation Lost Capture Register
#define CANHACKER_READ_REG        'G'   // read register conten from SJA1000
#define CANHACKER_WRITE_REG       'W'   // write register content to SJA1000
#define LISTEN_ONLY     'L' // switch to listen only mode

#define TIME_STAMP_TICK 1000    // microseconds

#define CMD_BUFFER_LENGTH  30


void exec_usb_cmd (uint8_t * cmd_buf, osMessageQId uartTxQueueHandle, osMessageQId canTxQueueHandle);

#endif
