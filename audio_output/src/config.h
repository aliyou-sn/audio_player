#include <freertos/FreeRTOS.h>
#include <driver/i2s.h>
#include "driver/spi_master.h"

#define USE_I2S

// speaker settings - if using I2S
#define I2S_SPEAKER_SERIAL_CLOCK GPIO_NUM_27
#define I2S_SPEAKER_LEFT_RIGHT_CLOCK GPIO_NUM_14
#define I2S_SPEAKER_SERIAL_DATA GPIO_NUM_26

#define GPIO_BUTTON GPIO_NUM_2

// FLash chip (Default SPI Pins)
#define PIN_NUM_MISO GPIO_NUM_19
#define PIN_NUM_CLK GPIO_NUM_18
#define PIN_NUM_MOSI GPIO_NUM_23
#define PIN_NUM_CS GPIO_NUM_5

// i2s speaker pins definition
extern i2s_pin_config_t i2s_speaker_pins;

//#define MAX_BLOCKSIZE         128  // Total number of blocks
//#define MAX_SECTORSIZE        2048 // Total number of sectors

#define CMD_WRITE_ENABLE      0x06
#define CMD_WRITE_DISABLE     0x04
#define CMD_READ_STATUS_R1    0x05
#define CMD_READ_STATUS_R2    0x35
#define CMD_WRITE_STATUS_R    0x01 // Unimplemented
#define CMD_PAGE_PROGRAM      0x02
#define CMD_QUAD_PAGE_PROGRAM 0x32 // Unimplemented
#define CMD_BLOCK_ERASE64KB   0xd8
#define CMD_BLOCK_ERASE32KB   0x52
#define CMD_SECTOR_ERASE      0x20
#define CMD_CHIP_ERASE        0xC7
#define CMD_ERASE_SUPPEND     0x75 // Unimplemented
#define CMD_ERASE_RESUME      0x7A // Unimplemented
#define CMD_POWER_DOWN        0xB9
#define CMD_HIGH_PERFORM_MODE 0xA3 // Unimplemented
#define CMD_CNT_READ_MODE_RST 0xFF // Unimplemented
#define CMD_RELEASE_PDOWN_ID  0xAB // Unimplemented
#define CMD_MANUFACURER_ID    0x90
#define CMD_READ_UNIQUE_ID    0x4B
#define CMD_JEDEC_ID          0x9f

#define CMD_READ_DATA         0x03
#define CMD_READ_DATA4B       0x13
#define CMD_FAST_READ         0x0B
#define CMD_FAST_READ4B       0x0C
#define CMD_READ_DUAL_OUTPUT  0x3B // Unimplemented
#define CMD_READ_DUAL_IO      0xBB // Unimplemented
#define CMD_READ_QUAD_OUTPUT  0x6B // Unimplemented
#define CMD_READ_QUAD_IO      0xEB // Unimplemented
#define CMD_WORD_READ         0xE3 // Unimplemented

#define SR1_BUSY_MASK	0x01
#define SR1_WEN_MASK	0x02


