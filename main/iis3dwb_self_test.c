/*
 ******************************************************************************
 * @file    iis3dwb_self_test.c
 * @author  Sensors Software Solution Team
 * @brief   This file implements the self test procedure.
 *
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
 * All rights reserved.</center></h2>
 *
 * This software component is licensed by ST under BSD 3-Clause license,
 * the "License"; You may not use this file except in compliance with the
 * License. You may obtain a copy of the License at:
 *                        opensource.org/licenses/BSD-3-Clause
 *
 ******************************************************************************
 */

/*
 * This example was developed using the following STMicroelectronics
 * evaluation boards:
 *
 * - STEVAL_MKI109V3 + STEVAL-MKI208V1K
 * - DISCOVERY_SPC584B + STEVAL-MKI208V1K
 *
 * and STM32CubeMX tool with STM32CubeF4 MCU Package
 *
 * Used interfaces:
 *
 * STEVAL_MKI109V3    - Host side:   USB (Virtual COM)
 *                    - Sensor side: SPI(Default) / I2C(supported)
 *
 * DISCOVERY_SPC584B  - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 * 
 * ESP32 PLATFORM     - Host side: UART(COM) to USB bridge
 *                    - Sensor side: I2C(Default) / SPI(supported)
 *
 *
 * If you need to run this example on a different hardware platform a
 * modification of the functions: 
 * 
 * `platform_write`, 
 * `platform_read`,
 * `tx_com` 
 * and 
 * 'platform_init' 
 * 
 * is required.
 *
 */

/* STMicroelectronics evaluation boards definition
 *
 * Please uncomment ONLY the evaluation boards in use.
 * If a different hardware is used please comment all
 * following target board and redefine yours.
 */

//#define STEVAL_MKI109V3  /* little endian */
//#define SPC584B_DIS      /* big endian */
#define ESP32           /* little endian */


/* ATTENTION: By default the driver is little endian. If you need switch
 *            to big endian please see "Endianness definitions" in the
 *            header file of the driver (_reg.h).
 */

#if defined(STEVAL_MKI109V3) /* MKI109V3: Define communication interface */
#define SENSOR_BUS hspi2 /* MKI109V3: Vdd and Vddio power supply values */
#define PWM_3V3 915

#elif defined(SPC584B_DIS) /* DISCOVERY_SPC584B: Define communication interface */
#define SENSOR_BUS NULL

#elif defined(ESP32) /** -- platform ESP32 dependent definitions ------------------------------ */
// Uncomment one of the following defines to enable debug output for ESP32 Plataform, comment 
#define ESP32_LIS3DWB_DEBUG_LEVEL_1    // only error messages
//#define ESP32_LIS3DWB_DEBUG_LEVEL_2    // debug and error messages

#define iis3dwb_self_test app_main

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 4096


// SPI interface definitions for ESP32s3
#define SPI_BUS SPI3_HOST
#define SPI_CS_GPIO 10
#define SPI_MOSI_GPIO 11 // SDI
#define SPI_SCK_GPIO 12
#define SPI_MISO_GPIO 13 // SDO


// I2C interface defintions for ESP32 - NOT USING BY NOW
#define I2C_BUS       0
#define I2C_SCL_PIN   11 //11 for ESP32s3 | 22 for esp32
#define I2C_SDA_PIN   13 //13 for ESP32s3 | 21 for esp32
#define I2C_FREQ      I2C_FREQ_100K


// interrupt GPIOs defintions for and ESP32
#define INT1_PIN      5
#define INT2_PIN      4

#define LIS3DWB_SPI_BUF_SIZE       64      // SPI register data buffer size of ESP866

#define LIS3DWB_SPI_READ_FLAG      0x80
#define LIS3DWB_SPI_WRITE_FLAG     0x00
#define LIS3DWB_SPI_AND_OPE_FLAG   0x7F //AND operator value flag



//LED Data IN OUTPUT GPIO PIN 
#define BUILTIN_BOARD_RGBLED_00_GPIO            15    // 15 -> WLSVS3M01 GPIO PIN | 48 -> devKit
#define BUILTIN_BOARD_RGBLED_00_TYPE            RMT   //RMT for RGB led Strip | GPIO (simple/mono color) for gpio led

//LED ENable OUTPUT GPIO PIN 
#define BUILTIN_BOARD_LEDEN_GPIO            18    // This is the GPIO on which the output will be set
                                                   // 18 -> WLSVS2M01 LED EN | NOT -> devkit
#define BUILTIN_BOARD_LEDEN_GPIO_MODE    GPIO_MODE_OUTPUT


#endif

/* Includes ------------------------------------------------------------------*/
#include <string.h>
#include <stdio.h>
#include "iis3dwb_reg.h"

#if defined(NUCLEO_F411RE)
  #include "stm32f4xx_hal.h"
  #include "usart.h"
  #include "gpio.h"
  #include "i2c.h"

#elif defined(STEVAL_MKI109V3)
  #include "stm32f4xx_hal.h"
  #include "usbd_cdc_if.h"
  #include "gpio.h"
  #include "spi.h"
  #include "tim.h"

#elif defined(SPC584B_DIS)
  #include "components.h"

#elif defined(ESP32)
  #include <stdio.h>
  #include <stdlib.h>
  #include <string.h>
  #include <sys/time.h>

  #include "freertos/FreeRTOS.h"
  #include "freertos/task.h"
  #include "esp_system.h"

  #include "esp32_bus.h"

#endif

/* Private macro -------------------------------------------------------------*/
#define    BOOT_TIME        10 //ms
#define    WAIT_TIME       100 //ms

/* Self test limits. */
#define    MIN_ST_LIMIT_mg        800.0f
#define    MAX_ST_LIMIT_mg       3200.0f

/* Self test results. */
#define    ST_PASS     1U
#define    ST_FAIL     0U

#if defined(ESP32) /** -- Private macro ESP32 dependent definitions ------------------------------ */
  #if defined(ESP32_LIS3DWB_DEBUG_LEVEL_2)
  #define debug(s, f, ...) printf("%s %s: " s "\n", "LIS3DWB", f, ## __VA_ARGS__)
  #define error_dev(s, f, da, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3DWB-ERROR", f, SPI_BUS, da, ## __VA_ARGS__)
  #else
  #define debug(s, f, ...) printf("%s %s: " s "\n", "LIS3DWB", f, ## __VA_ARGS__)
  #define debug_dev(s, f, da, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3DWB", f, SPI_BUS, da, ## __VA_ARGS__)
  #endif

  #if defined(ESP32_LIS3DWB_DEBUG_LEVEL_1) || defined(ESP32_LIS3DWB_DEBUG_LEVEL_2)
  #define error(s, f, ...) printf("%s %s: " s "\n", "LIS3DWB-ERROR", f, ## __VA_ARGS__)
  #define error_dev(s, f, da, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3DWB-ERROR", f, SPI_BUS, da, ## __VA_ARGS__)
  #else
  #define error(s, f, ...) printf("%s %s: " s "\n", "LIS3DWB-ERROR", f, ## __VA_ARGS__)
  #define error_dev(s, f, da, ...) printf("%s %s: bus %d, addr %02x - " s "\n", "LIS3DWB-ERROR", f, SPI_BUS, da, ## __VA_ARGS__)
  #endif
#endif

/* Private variables ---------------------------------------------------------*/

/* Extern variables ----------------------------------------------------------*/

/* Private functions ---------------------------------------------------------*/

/*
 *   WARNING:
 *   Functions declare in this section are defined at the end of this file
 *   and are strictly related to the hardware platform used.
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len);
static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len);
static void tx_com( uint8_t *tx_buffer, uint16_t len );
static void platform_delay(uint32_t ms);
static void platform_init(void);






























/* Main Example --------------------------------------------------------------*/
void iis3dwb_self_test(void)
{
  uint8_t tx_buffer[1000];
  stmdev_ctx_t dev_ctx;
  int16_t data_raw[3];
  float val_st_off[3];
  float val_st_on[3];
  float test_val[3];
  uint8_t st_result;
  uint8_t whoamI;
  uint8_t drdy;
  uint8_t rst;
  uint8_t i;
  uint8_t j;
  /* Initialize mems driver interface */
  dev_ctx.write_reg = platform_write;
  dev_ctx.read_reg = platform_read;
  //dev_ctx.handle = &SENSOR_BUS;
  /* Init test platform */
  platform_init();
  /* Wait sensor boot time */
  platform_delay(BOOT_TIME);
  /* Check device ID */
  iis3dwb_device_id_get(&dev_ctx, &whoamI);

  debug ("whoamI: %02x\n\n", __FUNCTION__, whoamI);

  if (whoamI != IIS3DWB_ID) {
    error ("IIS3DWB_ID wrong!!!", __FUNCTION__);
    while (1);

  }



  /* Restore default configuration */
  iis3dwb_reset_set(&dev_ctx, PROPERTY_ENABLE);

  do {
    iis3dwb_reset_get(&dev_ctx, &rst);
  } while (rst);

  /* Enable Block Data Update */
  iis3dwb_block_data_update_set(&dev_ctx, PROPERTY_ENABLE);
  /*
   * Accelerometer Self Test
   */
  /* Set Output Data Rate */
  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_26k7Hz);
  /* Set full scale */
  iis3dwb_xl_full_scale_set(&dev_ctx, IIS3DWB_4g);
  /* Wait stable output */
  platform_delay(WAIT_TIME);

  /* Check if new value available */
  do {
    iis3dwb_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  iis3dwb_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_off, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      iis3dwb_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    iis3dwb_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_off[j] += iis3dwb_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_off[i] /= 5.0f;
  }

  /* Enable Self Test positive (or negative) */
  iis3dwb_xl_self_test_set(&dev_ctx, IIS3DWB_XL_ST_POSITIVE);
  //iis3dwb_xl_self_test_set(&dev_ctx, IIS3DWB_XL_ST_NEGATIVE);
  /* Wait stable output */
  platform_delay(WAIT_TIME);

  /* Check if new value available */
  do {
    iis3dwb_xl_flag_data_ready_get(&dev_ctx, &drdy);
  } while (!drdy);

  /* Read dummy data and discard it */
  iis3dwb_acceleration_raw_get(&dev_ctx, data_raw);
  /* Read 5 sample and get the average vale for each axis */
  memset(val_st_on, 0x00, 3 * sizeof(float));

  for (i = 0; i < 5; i++) {
    /* Check if new value available */
    do {
      iis3dwb_xl_flag_data_ready_get(&dev_ctx, &drdy);
    } while (!drdy);

    /* Read data and accumulate the mg value */
    iis3dwb_acceleration_raw_get(&dev_ctx, data_raw);

    for (j = 0; j < 3; j++) {
      val_st_on[j] += iis3dwb_from_fs4g_to_mg(data_raw[j]);
    }
  }

  /* Calculate the mg average values */
  for (i = 0; i < 3; i++) {
    val_st_on[i] /= 5.0f;
  }

  /* Calculate the mg values for self test */
  for (i = 0; i < 3; i++) {
    test_val[i] = fabs((val_st_on[i] - val_st_off[i]));
  }

  /* Check self test limit */
  st_result = ST_PASS;

  for (i = 0; i < 3; i++) {
    if (( MIN_ST_LIMIT_mg > test_val[i] ) ||
        ( test_val[i] > MAX_ST_LIMIT_mg)) {
      st_result = ST_FAIL;
    }
  }

  /* Disable Self Test */
  iis3dwb_xl_self_test_set(&dev_ctx, IIS3DWB_XL_ST_DISABLE);
  /* Disable sensor. */
  iis3dwb_xl_data_rate_set(&dev_ctx, IIS3DWB_XL_ODR_OFF);

  if (st_result == ST_PASS) {
    sprintf((char *)tx_buffer, "Self Test - PASS\r\n" );
  }

  else {
    sprintf((char *)tx_buffer, "Self Test - FAIL\r\n" );
  }

  tx_com(tx_buffer, strlen((char const *)tx_buffer));
}

























/*
 * @brief  Write generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to write
 * @param  bufp      pointer to data to write in register reg
 * @param  len       number of consecutive register to write
 *
 */
static int32_t platform_write(void *handle, uint8_t reg, const uint8_t *bufp, uint16_t len)
{
#ifdef STEVAL_MKI109V3
  if (handle == &hspi2) {
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
    HAL_SPI_Transmit(handle, &reg, 1, 1000);
    HAL_SPI_Transmit(handle, (uint8_t*) bufp, len, 1000);
    HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
  }
#elif defined(SPC584B_DIS)
  /* Add here the SPC5 write SPI interface */

#elif defined(ESP32)
  /* ESP32 SPC5 write SPI interface */
//static bool lis3dwb_spi_write(lis3dwb_sensor_t* dev, uint8_t reg, uint8_t *data, uint16_t len)
if (!bufp) return 1;

    uint8_t addr = (reg & LIS3DWB_SPI_AND_OPE_FLAG);// | LIS3DWB_SPI_WRITE_FLAG;// | LIS3DWB_SPI_AUTO_INC_FLAG;

    static uint8_t mosi[LIS3DWB_SPI_BUF_SIZE];

    if (len >= LIS3DWB_SPI_BUF_SIZE)
    {
    error ("Error on write from SPI slave on bus %d. Tried to transfer "
                "more than %d byte in one read operation.",
                __FUNCTION__, SPI_BUS, LIS3DWB_SPI_BUF_SIZE);
    return 1;
    }

    reg &= LIS3DWB_SPI_AND_OPE_FLAG;

    // first byte in output is the register address
    mosi[0] = addr;



    // shift data one byte right, first byte in output is the register address
    for (int i = 0; i < len; i++)
        mosi[i+1] = bufp[i];



    #ifdef ESP32_LIS3DWB_DEBUG_LEVEL_2
    printf("LIS3DWB %s: Write the following bytes to reg 0x[%02x]: ", __FUNCTION__, reg);
    for (int i = 1; i < len+1; i++)
        printf("0x(%02x )", mosi[i]);
    printf("\n");
    #endif

    if (!spi_transfer_pf (SPI_BUS, SPI_CS_GPIO, mosi, NULL, len+1)) {
      error_dev ("Could not write data to SPI.", __FUNCTION__, addr);
      return 1;
    }

    return 0;
#endif
}













/*
 * @brief  Read generic device register (platform dependent)
 *
 * @param  handle    customizable argument. In this examples is used in
 *                   order to select the correct sensor bus handler.
 * @param  reg       register to read
 * @param  bufp      pointer to buffer that store the data read
 * @param  len       number of consecutive register to read
 *
 */


static int32_t platform_read(void *handle, uint8_t reg, uint8_t *bufp, uint16_t len)
//static int32_t platform_read(uint8_t reg, uint8_t *bufp, uint16_t len)
{
  #ifdef STEVAL_MKI109V3
    if (handle == &hspi2) {
      /* Read command */
      reg |= 0x80;
      HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_RESET);
      HAL_SPI_Transmit(handle, &reg, 1, 1000);
      HAL_SPI_Receive(handle, bufp, len, 1000);
      HAL_GPIO_WritePin(CS_up_GPIO_Port, CS_up_Pin, GPIO_PIN_SET);
    }

  #elif defined(SPC584B_DIS)
    /* Add here the SPC5 read SPI interface */

  #elif defined(ESP32)
  /* ESP32 read SPI interface */
  if (!bufp) return 1;

  if (len >= LIS3DWB_SPI_BUF_SIZE)
  {
      error ("Error on read from SPI slave on bus %d. Tried to transfer "
                  "more than %d byte in one read operation.",
                  __FUNCTION__, SPI_BUS, LIS3DWB_SPI_BUF_SIZE);
      return 1;
  }

  uint8_t addr = (reg & LIS3DWB_SPI_AND_OPE_FLAG) | LIS3DWB_SPI_READ_FLAG;// | LIS3DWB_SPI_AUTO_INC_FLAG;

  static uint8_t mosi[LIS3DWB_SPI_BUF_SIZE];
  static uint8_t miso[LIS3DWB_SPI_BUF_SIZE];

  memset (mosi, 0xff, LIS3DWB_SPI_BUF_SIZE);
  memset (miso, 0xff, LIS3DWB_SPI_BUF_SIZE);

  mosi[0] = addr;

  if (!spi_transfer_pf (SPI_BUS, SPI_CS_GPIO, mosi, miso, len+1))
  {
      error_dev ("Could not read data (bufp) from SPI", __FUNCTION__, addr);
      return 1;
  }

  // shift data one by left, first byte received while sending register address is invalid
  for (int i=0; i < len; i++)
  bufp[i] = miso[i+1];





  #ifdef ESP32_LIS3DWB_DEBUG_LEVEL_2
  printf("LIS3DWB %s: read the following bytes from reg 0x[%02x]: ", __FUNCTION__, reg);
  for (int i=0; i < len; i++)
      printf("0x(%02x )", bufp[i]);
  printf("\n");
  #endif


  return 0; //no erros

  #endif
}

/*
 * @brief  Send buffer to console (platform dependent)
 *
 * @param  tx_buffer     buffer to transmit
 * @param  len           number of byte to send
 *
 */
static void tx_com(uint8_t *tx_buffer, uint16_t len)
{
#ifdef STEVAL_MKI109V3
  CDC_Transmit_FS(tx_buffer, len);
#elif defined(SPC584B_DIS)
  sd_lld_write(&SD2, tx_buffer, len);
#elif defined(ESP32)
/* ESP32 tx_com interface (Send buffer to console)*/
debug ("%s", __FUNCTION__, tx_buffer);
#endif
}

/*
 * @brief  platform specific delay (platform dependent)
 *
 * @param  ms        delay in ms
 *
 */
static void platform_delay(uint32_t ms)
{
#if defined(NUCLEO_F411RE) | defined(STEVAL_MKI109V3)
  HAL_Delay(ms);
#elif defined(SPC584B_DIS)
  osalThreadDelayMilliseconds(ms);
#elif defined(ESP32)
/*ESP32 platform specific delay*/
vTaskDelay(ms / portTICK_PERIOD_MS);
#endif
}

/*
 * @brief  platform specific initialization (platform dependent)
 */
static void platform_init(void)
{
  bool esp32_spi_init = false;
  #if defined(STEVAL_MKI109V3)
    TIM3->CCR1 = PWM_3V3;
    TIM3->CCR2 = PWM_3V3;
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
    HAL_Delay(1000);

  #elif defined(ESP32)
  // ESP32 platform BUS specific initialization
  esp32_spi_init = esp32_spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);
  platform_delay(100);
  if (esp32_spi_init)
  {
    debug ("ESP32 SPI Interface Initialized!", __FUNCTION__);


    if (!esp32_spi_device_init (SPI_BUS, SPI_CS_GPIO)) {
      error ("Could not initialize the sensor connnected to ESP32 SPI Interface.", __FUNCTION__);
    } else {
      debug ("Sensor connnected to ESP32 SPI Interface Initialized!", __FUNCTION__);
    }

  } else{
    error ("Could not initialize the ESP32 SPI Interface.", __FUNCTION__);
  }
  #endif


}