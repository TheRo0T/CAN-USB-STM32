#include "canhacker.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <ctype.h>
#include <stdlib.h>
#include "cmsis_os.h"

extern void txUart();
extern void txCan();

void transmit_CAN(osMessageQId canTxQueueHandle, CanTxMsgTypeDef *txMsg);
uint8_t ascii2byte (uint8_t val);
void transmit_UART(osMessageQId uartTxQueueHandle, uint8_t *txLine);
void transmit_error(osMessageQId uartTxQueueHandle);
extern void transmitErrorMessage(char *message);
uint8_t nibble2ascii(uint8_t byte);
void debugPrint(osMessageQId uartTxQueueHandle, uint8_t *txLine);

void execTransmit11bit(osMessageQId uartTxQueueHandle, osMessageQId canTxQueueHandle, uint8_t *str) {

    const int idOffset = 1;
    const int dataOffset = 5;
    const int dlcOffset = 4;


    uint8_t cmd_len = strlen((char *)str); // get command length

    if ((cmd_len < 5) || (cmd_len > 21)) {   // check valid cmd length
        transmitErrorMessage("Error: unexpected command length");
        return;
    }

    uint8_t *cmd_buf_pntr2 = &(*str);    // point to start of received string
    cmd_buf_pntr2++;     // skip command identifier
    // check if all chars are valid hex chars
    while (*cmd_buf_pntr2) {
        if (!isxdigit (*cmd_buf_pntr2)) {
            transmitErrorMessage("Error: unexpected command data character");
            return;
        }
        ++cmd_buf_pntr2;
    }

    CanTxMsgTypeDef CAN_tx_msg;

    CAN_tx_msg.RTR = CAN_RTR_DATA;   // no remote transmission request
    CAN_tx_msg.IDE = CAN_ID_STD;
    // store ID
    CAN_tx_msg.StdId = ascii2byte(str[idOffset]) << 8 | ascii2byte(str[idOffset+1]) << 4 | ascii2byte(str[idOffset+2]);

    // store data length
    //CAN_tx_msg.DLC = ascii2byte(str[dlcOffset]);
    CAN_tx_msg.DLC = str[dlcOffset] & 0x0F;
    // check number of data bytes supplied against data lenght byte
    if (CAN_tx_msg.DLC != ((cmd_len - 5) / 2)) {
        char message[80];
        sprintf(message, "Invalid DLC: %d and %d", (int)CAN_tx_msg.DLC, (int)((cmd_len - 5) / 2));
        transmitErrorMessage(message);
        return;
    }

    // check for valid length
    if (CAN_tx_msg.DLC > 8) {
        transmitErrorMessage("DLC > 8");
        return;
    }

    int i;
    for (i = 0; i < CAN_tx_msg.DLC; i++) {
        CAN_tx_msg.Data[i] = (ascii2byte(str[dataOffset+i*2]) << 4) + ascii2byte(str[dataOffset+i*2+1]);
    }

    // if transmit buffer was empty send message
    transmit_CAN(canTxQueueHandle, &CAN_tx_msg);
}

void exec_usb_cmd (uint8_t *cmd_buf, osMessageQId uartTxQueueHandle, osMessageQId canTxQueueHandle)
{
    //debugPrint(uartTxQueueHandle, cmd_buf);

    char firstChar = *cmd_buf;

    switch (firstChar) {
        // get serial number
        case GET_SERIAL: {
            uint8_t str[7];
            str[0] = GET_SERIAL;
            memcpy(str+1, SERIAL, strlen(SERIAL));
            str[5] = CANHACKER_CR;
            str[6] = '\0';
            transmit_UART(uartTxQueueHandle, str);
            return;
        }

        // get hard- and software version
        case GET_VERSION: {
            uint8_t str[7];
            str[0] = GET_VERSION;
            str[1] = nibble2ascii(HW_VER >> 4);
            str[2] = nibble2ascii(HW_VER);
            str[3] = nibble2ascii(SW_VER >> 4);
            str[4] = nibble2ascii(SW_VER);
            str[5] = CANHACKER_CR;
            str[6] = '\0';
            transmit_UART(uartTxQueueHandle, str);
            return;
        }

        // get only software version
        case GET_SW_VERSION: {
            uint8_t str[7];
            str[0] = GET_SW_VERSION;
            str[1] = nibble2ascii(SW_VER_MAJOR >> 4);
            str[2] = nibble2ascii(SW_VER_MAJOR);
            str[3] = nibble2ascii(SW_VER_MINOR >> 4);
            str[4] = nibble2ascii(SW_VER_MINOR);
            str[5] = CANHACKER_CR;
            str[6] = '\0';
            transmit_UART(uartTxQueueHandle, str);
            return;
        }

        case SEND_11BIT_ID:
            execTransmit11bit(uartTxQueueHandle, canTxQueueHandle, cmd_buf);
            return;

            // end with error on unknown commands
        default:
            return transmit_error(uartTxQueueHandle);
    }               // end switch

    // we should never reach this return
    return transmit_error(uartTxQueueHandle);
}

void transmit_error(osMessageQId uartTxQueueHandle) {
    uint8_t str[2] = {CANHACKER_ERROR, '\0'};
    transmit_UART(uartTxQueueHandle, str);
}

void debugPrint(osMessageQId uartTxQueueHandle, uint8_t *txLine)
{
    int len = strlen((char *)txLine);
    char *str = malloc(len + 3);
    memcpy(str, txLine, len + 3);
    str[len] = '\r';
    str[len+1] = '\n';
    str[len+2] = 0;
    osMessagePut (uartTxQueueHandle, (uint32_t)str, osWaitForever);
    txUart();

    return;
}

void transmit_UART(osMessageQId uartTxQueueHandle, uint8_t *txLine)
{
    int len = strlen((char *)txLine);
    char *str = malloc(len+1);
    memcpy(str, txLine, len+1);
    osMessagePut (uartTxQueueHandle, (uint32_t)str, osWaitForever);
    txUart();

    return;
}

void transmit_CAN(osMessageQId canTxQueueHandle, CanTxMsgTypeDef *txMsg)
{
    CanTxMsgTypeDef *txMsgQ = malloc(sizeof(CanTxMsgTypeDef));
    memcpy(txMsgQ, txMsg, sizeof(CanTxMsgTypeDef));

    osMessagePut (canTxQueueHandle, (uint32_t)txMsgQ, osWaitForever);
    txCan();

    return;
}

uint8_t ascii2byte(uint8_t val)
{
    if (val >= 'a') {
        return val - 'a' + 10; // convert chars a-f
    }
    if (val >= 'A') {
        return val - 'A' + 10; // convert chars A-F
    }
    return val - '0';     // convert chars 0-9
}

uint8_t nibble2ascii(uint8_t byte) {
    byte &= 0x0F;
    return byte < 10 ? byte + 48 : byte + 55;
}
