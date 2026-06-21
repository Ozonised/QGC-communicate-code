/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2026 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "gpio.h"
#include "i2c.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usbd_cdc_if.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#define MAVLINK_NO_SIGN_PACKET
#include "common/mavlink.h"
#include "lsm6dsl.hpp"
#define BMP390_RET_TYPE HAL_StatusTypeDef
#define BMP390_RET_TYPE_SUCCESS HAL_OK
#define BMP390_RET_TYPE_FAILURE HAL_ERROR
#define BMP390_RET_TYPE_BUSY HAL_BUSY
#include "AltitudeEKF.hpp"
#include "ExtendedKalmanFilter.hpp"
#include "bmp390.hpp"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
extern uint8_t UserRxBufferFS[APP_TX_DATA_SIZE];
extern volatile bool USBD_CDC_Transmit_Complete, USBD_CDC_Receive_Complete, ImuInterrupt = false, BaroInterrupt = false;
extern volatile uint32_t USB_CDC_Rx_Buf_Index, USB_CDC_Rx_Received_Len;

uint8_t MavlinkTxBuf[APP_TX_DATA_SIZE];
uint16_t MavlinkTxLen, MavlinkRxLen;
bool MavlinkResponseToSend = false, SendOtherMavlinkMessages = false;

uint32_t currentMillis, prevHeartBeatTime;
const uint32_t HEARTBEAT_MSG_INTERVAL = 1000;

mavlink_message_t MsgFromGCS, MsgToGCS;
mavlink_status_t chanStatus;
mavlink_heartbeat_t FcHeartBeat, GcsHeartBeat;
mavlink_param_request_list_t paramRequestList;
mavlink_command_long_t commandLong;
mavlink_sys_status_t SystemStatus;
mavlink_current_mode_t CurrentMode;

float P = 10.0f, I = 0.29f, D = 0.01f;
float *parameters[3] = {&P, &I, &D};

uint8_t FlightCustomVersion[8] = "DEV 0.1", MiddlewareCustomVersion[8] = "MAV 0.1", OsCustomVersion[8] = "CUS OS";

LSM6DSL_AccelRawData xl;
LSM6DSL_GyroRawData gy;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
bool HandleMavlinkMessage(mavlink_message_t *MsgFromGCS, mavlink_message_t *MsgToGCS, uint8_t *MavLinkTxBuf, uint16_t *MavLinkTxLen);

LSM6DSL_INTF_RET_TYPE LSM6DSL_I2C_Read(void *hInterface, uint8_t chipAddr, uint8_t RegAddr, uint8_t *buf, uint16_t len);
LSM6DSL_INTF_RET_TYPE LSM6DSL_I2C_Write(void *hInterface, uint8_t chipAddr, uint8_t RegAddr, uint8_t *buf, uint16_t len);

BMP390_RET_TYPE BMP390_I2CRead(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
BMP390_RET_TYPE BMP390_I2CWrite(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
LSM6DSL imu(static_cast<void *>(&hi2c2), LSM6DSL_I2C_Read, LSM6DSL_I2C_Write, HAL_Delay);
BMP390 baro(static_cast<void *>(&hi2c2), BMP390_I2CRead, BMP390_I2CWrite);
/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void) {

  /* USER CODE BEGIN 1 */
  HAL_StatusTypeDef ret;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick.
   */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 2 */
  imu.setI2CAddress(1);

  baro.Init(1, 0);

  ret = baro.IsPresent();

  while (ret != HAL_OK)
    ;

  ret = baro.ReadNVM();

  while (ret != HAL_OK)
    ;

  ret = baro.SetPressureOversampling(bmp390::TempPressOversampling::x8);

  while (ret != HAL_OK)
    ;

  ret = baro.SetTemperatureOversampling(bmp390::TempPressOversampling::x1);

  while (ret != HAL_OK)
    ;

  ret = baro.SetOutputDataRate(bmp390::TempPressODR::ODR_50Hz);

  while (ret != HAL_OK)
    ;

  ret = baro.SetIIRFilterCoefficient(bmp390::IIRFilterCoefficient::coef3);

  while (ret != HAL_OK)
    ;

  ret = baro.SetInterruptSource(1, 0, 0);

  while (ret != HAL_OK)
    ;

  ret = baro.ToggleTemperatureAndPressureMeasurement(1, 1);
  while (ret != HAL_OK)
    ;

  ret = baro.SetPowerMode(bmp390::PowerMode::Normal);

  imu.softwareReset();
  HAL_Delay(5);
  while (imu.isSoftwareResetComplete() != 0)
    ;
  imu.setAccelFSRange(LSM6DSL_XL_FS_4G);
  imu.setGyroFSRange(LSM6DSL_G_FS_1000DPS);
  imu.configGyroHPF(LSM6DSL_G_HPF_BW_0_260Hz);
  imu.configAccelDigitalLPF(LSM6DSL_XL_LPF_BW_ODR_9, 1);
  imu.INT1SourceConfig(static_cast<LSM6DSL_INT1_Sources>(LSM6DSL_INT1_DRDY_XL | LSM6DSL_INT1_DRDY_G));
  imu.setAccelODR(LSM6DSL_XL_ODR_416Hz);
  imu.setGyroODR(LSM6DSL_G_ODR_416Hz);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  FcHeartBeat.autopilot = MAV_AUTOPILOT_GENERIC;
  FcHeartBeat.type = MAV_TYPE_QUADROTOR;
  FcHeartBeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  FcHeartBeat.custom_mode = MAV_STANDARD_MODE_POSITION_HOLD;
  FcHeartBeat.system_status = MAV_STATE_STANDBY;

  SystemStatus.onboard_control_sensors_present = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                 MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                 MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;

  SystemStatus.onboard_control_sensors_enabled = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL;

  SystemStatus.onboard_control_sensors_health = MAV_SYS_STATUS_SENSOR_3D_GYRO | MAV_SYS_STATUS_SENSOR_3D_ACCEL |
                                                MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION | MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL |
                                                MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS;
  SystemStatus.load = 1;
  SystemStatus.voltage_battery = UINT16_MAX;
  SystemStatus.current_battery = -1;
  SystemStatus.battery_remaining = -1;
  SystemStatus.drop_rate_comm = 0;
  SystemStatus.errors_comm = 0;
  SystemStatus.errors_count1 = 0;
  SystemStatus.errors_count2 = 0;
  SystemStatus.errors_count3 = 0;
  SystemStatus.errors_count4 = 0;
  SystemStatus.onboard_control_sensors_enabled_extended = 0;
  SystemStatus.onboard_control_sensors_present_extended = 0;
  SystemStatus.onboard_control_sensors_health_extended = 0;

  CurrentMode.standard_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
  CurrentMode.custom_mode = MAV_STANDARD_MODE_POSITION_HOLD;
  CurrentMode.intended_custom_mode = MAV_STANDARD_MODE_POSITION_HOLD;

  uint8_t parseState = 0;

  while (1) {
    currentMillis = HAL_GetTick();

    if (!MavlinkResponseToSend && USBD_CDC_Transmit_Complete && (currentMillis - prevHeartBeatTime) >= HEARTBEAT_MSG_INTERVAL) {
      mavlink_msg_heartbeat_encode(1, MAV_COMP_ID_AUTOPILOT1, &MsgToGCS, &FcHeartBeat);
      MavlinkTxLen = mavlink_msg_to_send_buffer(MavlinkTxBuf, &MsgToGCS);
      USBD_CDC_Transmit_Complete = false;
      CDC_Transmit_FS(MavlinkTxBuf, MavlinkTxLen);
      prevHeartBeatTime = currentMillis;
    }

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

    if (USBD_CDC_Receive_Complete) {
      USBD_CDC_Receive_Complete = false;

      static uint32_t i = 0;
      uint32_t MavLinkRxLen = i + USB_CDC_Rx_Received_Len; // total number of bytes in the rx buffer

      while (i < MavLinkRxLen) {
        parseState = mavlink_parse_char(MAVLINK_COMM_0, UserRxBufferFS[i], &MsgFromGCS, &chanStatus);
        i++;

        if (parseState) {
          MavlinkResponseToSend = HandleMavlinkMessage(&MsgFromGCS, &MsgToGCS, MavlinkTxBuf, &MavlinkTxLen);
          parseState = 0;

          // send respone if previous transmission has completed and a mavlink
          // respone has to be sent
          if (USBD_CDC_Transmit_Complete && MavlinkResponseToSend) {
            USBD_CDC_Transmit_Complete = false;
            MavlinkResponseToSend = false;
            CDC_Transmit_FS(MavlinkTxBuf, MavlinkTxLen);
          }
        }
      }
      // Rx buffer overflow
      if (i > USB_CDC_Rx_Buf_Index)
        i = 0;
    }
  }
  /* USER CODE END 3 */
}

/**
 * @brief System Clock Configuration
 * @retval None
 */
void SystemClock_Config(void) {
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 12;
  RCC_OscInitStruct.PLL.PLLN = 96;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_3) != HAL_OK) {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
bool HandleMavlinkMessage(mavlink_message_t *MsgFromGCS, mavlink_message_t *MsgToGCS, uint8_t *MavLinkTxBuf, uint16_t *MavLinkTxLen) {
  bool messageToBeSent = false;
  uint32_t msgID = MsgFromGCS->msgid;
start:
  switch (msgID) {
  case MAVLINK_MSG_ID_SYS_STATUS:
    mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,
                                 MsgFromGCS->compid);
    *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);

    mavlink_msg_sys_status_encode(1, 1, MsgToGCS, &SystemStatus);
    *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
    messageToBeSent = true;
    break;

  case MAVLINK_MSG_ID_HEARTBEAT:
    mavlink_msg_heartbeat_decode(MsgFromGCS, &GcsHeartBeat);
    HAL_GPIO_TogglePin(LED_ARM_GPIO_Port, LED_ARM_Pin);
    break;

  case 20:
    // mavlink_msg_param_request_read_decode(MsgFromGCS, &param_request_read);
    break;

  case MAVLINK_MSG_ID_PARAM_REQUEST_LIST:
    mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, "P", *parameters[0], MAV_PARAM_TYPE_REAL32, 3, 0);
    *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
    mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, "I", *parameters[1], MAV_PARAM_TYPE_REAL32, 3, 1);
    *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
    mavlink_msg_param_value_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, "D", *parameters[2], MAV_PARAM_TYPE_REAL32, 3, 2);
    *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
    messageToBeSent = true;
    break;

  case MAVLINK_MSG_ID_COMMAND_LONG:
    mavlink_msg_command_long_decode(MsgFromGCS, &commandLong);

    switch (commandLong.command) {
    case MAV_CMD_REQUEST_MESSAGE:
      switch ((uint32_t)commandLong.param1) {
      case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
      case MAVLINK_MSG_ID_SYS_STATUS:
      case MAVLINK_MSG_ID_AVAILABLE_MODES:
        msgID = (uint32_t)commandLong.param1;
        goto start;
        break;

      default:
        mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_DENIED, UINT8_MAX, 0, MsgFromGCS->sysid, MsgFromGCS->compid);
        *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
        messageToBeSent = true;
        break;
      }
      break;

    case MAV_CMD_COMPONENT_ARM_DISARM:
      // arm
      if ((uint32_t)commandLong.param1 == 1) {
        FcHeartBeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
        FcHeartBeat.system_status = MAV_STATE_ACTIVE;
      } else {
        // disarm
        FcHeartBeat.base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
        FcHeartBeat.system_status = MAV_STATE_STANDBY;
      }
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, commandLong.command, MAV_RESULT_ACCEPTED, 100, 0, MsgFromGCS->sysid, MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      messageToBeSent = true;
      break;

    case MAV_CMD_DO_SET_MODE:
      if ((uint32_t)commandLong.param1 == MAV_MODE_FLAG_CUSTOM_MODE_ENABLED) {
        // update flight modes in both current mode and heartbeat object
        CurrentMode.custom_mode = (uint8_t)commandLong.param2;
        CurrentMode.intended_custom_mode = (uint8_t)commandLong.param2;
        FcHeartBeat.custom_mode = (uint8_t)commandLong.param2;
      }
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, commandLong.command, MAV_RESULT_ACCEPTED, 100, 0, MsgFromGCS->sysid, MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_current_mode_encode(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, &CurrentMode);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    default:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, commandLong.command, MAV_RESULT_UNSUPPORTED, UINT8_MAX, 0, MsgFromGCS->sysid,
                                   MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      messageToBeSent = true;
      break;
    }
    // HAL_GPIO_TogglePin(LED_ERROR_GPIO_Port, LED_ERROR_Pin);
    break;

  case MAVLINK_MSG_ID_MANUAL_CONTROL:
    // mavlink_msg_manual_control_decode(MsgFromGCS, &virtualJoystick);
    //		HAL_GPIO_TogglePin(LED_GPS_GPIO_Port, LED_GPS_Pin);
    break;

  case MAVLINK_MSG_ID_AUTOPILOT_VERSION:
    mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid, MsgFromGCS->compid);
    *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
    mavlink_msg_autopilot_version_pack(
        1, 1, MsgToGCS,
        MAV_PROTOCOL_CAPABILITY_COMMAND_INT | MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET | MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT |
            MAV_PROTOCOL_CAPABILITY_MAVLINK2 | MAV_PROTOCOL_CAPABILITY_PARAM_ENCODE_C_CAST | MAV_PROTOCOL_CAPABILITY_COMPONENT_ACCEPTS_GCS_CONTROL,
        FIRMWARE_VERSION_TYPE_DEV, 0, 1, 12, FlightCustomVersion, MiddlewareCustomVersion, OsCustomVersion, 123, 456, 10101, 0);
    *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
    messageToBeSent = true;
    break;

  case MAVLINK_MSG_ID_AVAILABLE_MODES:
    // if param2 is zero, then send all the mode else send only the requested mode
    switch ((uint32_t)commandLong.param2) {
    case 0:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 1, MAV_STANDARD_MODE_POSITION_HOLD, 1, 0, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 2, MAV_STANDARD_MODE_LAND, 2, MAV_MODE_PROPERTY_AUTO_MODE, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 3, MAV_STANDARD_MODE_ALTITUDE_HOLD, 3, 0, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 4, MAV_STANDARD_MODE_TAKEOFF, 4, MAV_MODE_PROPERTY_AUTO_MODE, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    case 1:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 1, MAV_STANDARD_MODE_POSITION_HOLD, 1, 0, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    case 2:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 2, MAV_STANDARD_MODE_LAND, 2, MAV_MODE_PROPERTY_AUTO_MODE, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    case 3:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 3, MAV_STANDARD_MODE_ALTITUDE_HOLD, 3, 0, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    case 4:
      mavlink_msg_command_ack_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MAV_CMD_REQUEST_MESSAGE, MAV_RESULT_ACCEPTED, UINT8_MAX, 0, MsgFromGCS->sysid,MsgFromGCS->compid);
      *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
      mavlink_msg_available_modes_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, 4, 4, MAV_STANDARD_MODE_TAKEOFF, 4, MAV_MODE_PROPERTY_AUTO_MODE, 0);
      *MavLinkTxLen += mavlink_msg_to_send_buffer(&MavLinkTxBuf[*MavLinkTxLen], MsgToGCS);
      messageToBeSent = true;
      break;

    default:
      break;
    }
    break;

  case MAVLINK_MSG_ID_MISSION_REQUEST_LIST:
    mavlink_msg_mission_count_pack(1, MAV_COMP_ID_AUTOPILOT1, MsgToGCS, MsgFromGCS->sysid, MsgFromGCS->compid, 0, MAV_MISSION_TYPE_MISSION, 0);
    *MavLinkTxLen = mavlink_msg_to_send_buffer(MavLinkTxBuf, MsgToGCS);
    messageToBeSent = true;
    break;

  case MAVLINK_MSG_ID_MISSION_ACK:
    SendOtherMavlinkMessages = true;
    break;

  default:
    break;
  }
  return messageToBeSent;
}

LSM6DSL_INTF_RET_TYPE LSM6DSL_I2C_Read(void *hInterface, uint8_t chipAddr, uint8_t RegAddr, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef *>(hInterface), chipAddr << 1, RegAddr, 1, buf, len, 10);
}

LSM6DSL_INTF_RET_TYPE LSM6DSL_I2C_Write(void *hInterface, uint8_t chipAddr, uint8_t RegAddr, uint8_t *buf, uint16_t len) {
  return HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef *>(hInterface), chipAddr << 1, RegAddr, 1, buf, len, 10);
}

BMP390_RET_TYPE BMP390_I2CRead(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  BMP390_RET_TYPE ret = HAL_ERROR;
  if (hInterface != nullptr) {
    ret = HAL_I2C_Mem_Read(static_cast<I2C_HandleTypeDef *>(hInterface), static_cast<uint16_t>(chipAddr << 1), static_cast<uint16_t>(reg), 1U, buf,
                           static_cast<uint16_t>(len), 10U);
    if (ret == HAL_TIMEOUT)
      ret = HAL_ERROR;
  }
  return ret;
}

BMP390_RET_TYPE BMP390_I2CWrite(void *hInterface, uint8_t chipAddr, uint8_t reg, uint8_t *buf, uint8_t len) {
  BMP390_RET_TYPE ret = HAL_ERROR;
  if (hInterface != nullptr) {
    ret = HAL_I2C_Mem_Write(static_cast<I2C_HandleTypeDef *>(hInterface), static_cast<uint16_t>(chipAddr << 1), static_cast<uint16_t>(reg), 1U, buf,
                            static_cast<uint16_t>(len), 10U);
    if (ret == HAL_TIMEOUT)
      ret = HAL_ERROR;
  }
  return ret;
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void) {
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1) {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t *file, uint32_t line) {
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line
     number, ex: printf("Wrong parameters value: file %s on line %d\r\n", file,
     line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
