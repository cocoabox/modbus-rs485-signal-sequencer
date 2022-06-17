#pragma once

#define START_ADDRESS 1000

#define FLASH0_INTERVAL 480
#define FLASH1_INTERVAL 250
#define FLASH2_INTERVAL 180

// channel states
#define OFF 0
#define ON 1
#define FLASH0 2
#define FLASH1 3
#define FLASH2 4
#define FLASH0_INVERT 5
#define FLASH1_INVERT 6
#define FLASH2_INVERT 7

#define ERROR -1

#define READ_SWITCH_OK 0
#define READ_SWITCH_INVALID_VALUES 1
#define READ_SWITCH_CHANGED 2

// modbus holding register

#define DATA_SIZE 10

#define REG_VERSION 0  // R/O
#define REG_NUM_CHANNELS 1 // R/O
#define REG_CURRENT_MEASUREMENT 2 // R/O milliamps
#define REG_REMOTE_MODE 3 // R/W OK
#define REG_CH_0 4
#define REG_CH_1 5
#define REG_CH_2 6
#define REG_CH_3 7
#define REG_CH_4 8
#define REG_CH_5 9

#define LOCAL_MODE 0
#define SLAVE_MANUAL_MODE 1


#define CURRENT_MEASUREMENT_COUNT 5
