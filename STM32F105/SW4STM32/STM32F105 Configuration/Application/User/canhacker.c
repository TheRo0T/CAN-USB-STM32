#include "canhacker.h"
#include "stm32f1xx_hal.h"
#include <string.h>
#include <ctype.h>
#include "utils.h"

void CanHacker_Init(CanHacker_HandleTypeDef *canhacker) {
    canhacker->timestamp = CANHACKER_TIMESTAMP_DISABLED;
}

__weak void CanHacker_ErrorCallback(CanHacker_HandleTypeDef *canhacker, char *message) {

}

__weak void CanHacker_CanTxMsgReadyCallback(CanHacker_HandleTypeDef *canhacker, CanTxMsgTypeDef *txMsg) {

}

__weak void CanHacker_UartMsgReadyCallback(CanHacker_HandleTypeDef *canhacker, uint8_t *line) {

}

void CanHacker_ExecGetSWVersion(CanHacker_HandleTypeDef *canhacker) {
    uint8_t str[7];
    str[0] = CANHACKER_GET_SW_VERSION;
    str[1] = nibble2ascii(CANHACKER_SW_VER_MAJOR >> 4);
    str[2] = nibble2ascii(CANHACKER_SW_VER_MAJOR);
    str[3] = nibble2ascii(CANHACKER_SW_VER_MINOR >> 4);
    str[4] = nibble2ascii(CANHACKER_SW_VER_MINOR);
    str[5] = CANHACKER_CR;
    str[6] = '\0';
    CanHacker_UartMsgReadyCallback(canhacker, str);
}

void CanHacker_ExecGetVersion(CanHacker_HandleTypeDef *canhacker) {
    uint8_t str[7];
    str[0] = CANHACKER_GET_VERSION;
    str[1] = nibble2ascii(CANHACKER_HW_VER >> 4);
    str[2] = nibble2ascii(CANHACKER_HW_VER);
    str[3] = nibble2ascii(CANHACKER_SW_VER >> 4);
    str[4] = nibble2ascii(CANHACKER_SW_VER);
    str[5] = CANHACKER_CR;
    str[6] = '\0';
    CanHacker_UartMsgReadyCallback(canhacker, str);
}

void CanHacker_ExecGetSerial(CanHacker_HandleTypeDef *canhacker) {
    uint8_t str[7];
    str[0] = CANHACKER_GET_SERIAL;
    memcpy(str+1, CANHACKER_SERIAL, strlen(CANHACKER_SERIAL));
    str[5] = CANHACKER_CR;
    str[6] = '\0';
    CanHacker_UartMsgReadyCallback(canhacker, str);
}

void CanHacker_ExecTransmit11bit(CanHacker_HandleTypeDef *canhacker, uint8_t *str) {

    const int idOffset = 1;
    const int dataOffset = 5;
    const int dlcOffset = 4;


    uint8_t cmd_len = strlen((char *)str); // get command length

    if ((cmd_len < 5) || (cmd_len > 21)) {   // check valid cmd length
        CanHacker_ErrorCallback(canhacker, "Error: unexpected command length");
        return;
    }

    uint8_t *cmd_buf_pntr2 = &(*str);    // point to start of received string
    cmd_buf_pntr2++;     // skip command identifier
    // check if all chars are valid hex chars
    while (*cmd_buf_pntr2) {
        if (!isxdigit (*cmd_buf_pntr2)) {
            CanHacker_ErrorCallback(canhacker, "Error: unexpected command data character");
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
        CanHacker_ErrorCallback(canhacker, message);
        return;
    }

    // check for valid length
    if (CAN_tx_msg.DLC > 8) {
        CanHacker_ErrorCallback(canhacker, "DLC > 8");
        return;
    }

    int i;
    for (i = 0; i < CAN_tx_msg.DLC; i++) {
        CAN_tx_msg.Data[i] = (ascii2byte(str[dataOffset+i*2]) << 4) + ascii2byte(str[dataOffset+i*2+1]);
    }

    CanHacker_CanTxMsgReadyCallback(canhacker, &CAN_tx_msg);
}

void CanHacker_Receive_Cmd(CanHacker_HandleTypeDef *canhacker, uint8_t *cmd_buf)
{
    char firstChar = *cmd_buf;

    switch (firstChar) {
        // get serial number
        case CANHACKER_GET_SERIAL: {
            CanHacker_ExecGetSerial(canhacker);
            return;
        }

        // get hard- and software version
        case CANHACKER_GET_VERSION: {
            CanHacker_ExecGetVersion(canhacker);
            return;
        }

        // get only software version
        case CANHACKER_GET_SW_VERSION: {
            CanHacker_ExecGetSWVersion(canhacker);
            return;
        }

        case CANHACKER_SEND_11BIT_ID:
            CanHacker_ExecTransmit11bit(canhacker, cmd_buf);
            return;

            // end with error on unknown commands
        default:
            CanHacker_ErrorCallback(canhacker, "Unexpected command");
            return;
    }

    return CanHacker_ErrorCallback(canhacker, "Should never reach this section");
}



void CanHacker_CanRxMsgToString(CanRxMsgTypeDef *msg, uint8_t *str) {
    str[0] = CANHACKER_SEND_11BIT_ID;
    int i;
    uint32_t id = msg->StdId;
    for(i=3; i>=1; i--) {
        str[i] = nibble2ascii(id);
        id >>= 4;
    }
    str[4] = nibble2ascii(msg->DLC);

    for (i=0; i<msg->DLC; i++) {
        uint8_t byte = msg->Data[i];
        str[5+i*2] = nibble2ascii(byte >> 4);
        str[5+i*2+1] = nibble2ascii(byte);
    }
    int length = 5 + msg->DLC * 2;
    str[length] = CANHACKER_CR;
    str[length+1] = '\0';
}

void CanHacker_Receive_CanMsg(CanHacker_HandleTypeDef *canhacker, CanRxMsgTypeDef *msg) {

    uint8_t str[CANHACKER_CMD_MAX_LENGTH];
    CanHacker_CanRxMsgToString(msg, str);
    CanHacker_UartMsgReadyCallback(canhacker, str);
}
