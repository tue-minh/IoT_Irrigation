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
#include "adc.h"
#include "dma.h"
#include "i2c.h"
#include "usart.h"
#include "usb_device.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "ec600n.h"
#include "irrigation_ctrl.h"
#include <stdio.h>
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
ec600n_device_t ec600n;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
static void on_mqtt_message_received(const char *topic, const uint8_t *payload, uint16_t len)
{
  ec600n_log("[APP] MQTT Recv on topic '%s': %.*s\r\n", topic, len, payload);

  /* Process REST API JSON control command from backend via MQTT */
  char json_buf[128] = {0};
  if (len < sizeof(json_buf)) {
    memcpy(json_buf, payload, len);
    json_buf[len] = '\0';
    irrigation_process_rest_payload(json_buf, NULL, 0);
  }
}

static void on_sms_received(uint8_t index)
{
  ec600n_log("[APP] New SMS received at index %d\r\n", index);
  ec600n_sms_t sms;
  if (ec600n_sms_read(&ec600n, index, &sms) == EC600N_OK) {
    ec600n_log("[APP] SMS content: %s\r\n", sms.text);
    /* Process REST/command payload from SMS text */
    irrigation_process_rest_payload(sms.text, NULL, 0);
    ec600n_sms_delete(&ec600n, index);
  }
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_I2C3_Init();
  MX_USB_DEVICE_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  ec600n_log("\r\n==============================================\r\n");
  ec600n_log("   STM32F401 + Quectel EC600N 4G LTE System    \r\n");
  ec600n_log("==============================================\r\n");

  /* 0. Initialize Irrigation Actuators: Motor Pump (PB4) and Valves (PB5, PB8, PB9) */
  irrigation_init();

  /* Demonstrate Safety Interlock Rule via REST payload simulation */
  char test_resp[128];
  ec600n_log("[APP] Testing Safety Interlock...\r\n");
  /* Attempting to turn on pump with NO valves open -> DENIED by Safety Interlock */
  irrigation_process_rest_payload("{\"pump\":1,\"valve1\":0,\"valve2\":0,\"valve3\":0}", test_resp, sizeof(test_resp));
  ec600n_log("[APP] Interlock Test 1 (0 valves open): %s\r\n", test_resp);

  /* Turning on Valve 1 + Pump -> ALLOWED */
  irrigation_process_rest_payload("{\"pump\":1,\"valve1\":1,\"valve2\":0,\"valve3\":0}", test_resp, sizeof(test_resp));
  ec600n_log("[APP] Interlock Test 2 (1 valve open): %s\r\n", test_resp);

  /* Return to safe idle state (all off) */
  irrigation_emergency_stop();

  /* 1. Initialize EC600N driver with USART1 2-way DMA */
  if (ec600n_init(&ec600n, &huart1) == EC600N_OK) {
    ec600n_log("[APP] EC600N driver initialized successfully\r\n");
  }

  /* Register callback handlers */
  ec600n_set_mqtt_recv_cb(&ec600n, on_mqtt_message_received);
  ec600n_set_sms_recv_cb(&ec600n, on_sms_received);

  /* 2. Check communication with module (send AT) */
  ec600n_log("[APP] Checking communication with EC600N...\r\n");
  if (ec600n_check_communication(&ec600n) == EC600N_OK) {
    ec600n_log("[APP] EC600N response OK!\r\n");
    ec600n_echo(&ec600n, false); /* Disable echo for clean responses */

    /* 3. Read module details (Model, Firmware version, IMEI, IMSI) */
    ec600n_get_device_info(&ec600n);

    /* 4. Check SIM card */
    ec600n_log("[APP] Checking SIM status...\r\n");
    if (ec600n_check_sim(&ec600n) == EC600N_OK) {
      ec600n_log("[APP] SIM card is READY! ICCID: %s\r\n", ec600n.info.iccid);

      /* 5. Wait for 4G LTE network registration */
      ec600n_log("[APP] Waiting for 4G LTE registration...\r\n");
      if (ec600n_wait_network_registered(&ec600n, 20000) == EC600N_OK) {
        ec600n_log("[APP] 4G LTE Registered on: %s\r\n", ec600n.net_info.operator_name);
        ec600n_log("[APP] Signal: RSSI=%d (%d dBm), %d/5 bars\r\n",
                   ec600n.net_info.signal.rssi,
                   ec600n.net_info.signal.rssi_dbm,
                   ec600n.net_info.signal.bars);

        /* 6. Activate 4G Data (PDP Context) with APN */
        ec600n_log("[APP] Activating PDP context (APN: v-internet)...\r\n");
        if (ec600n_activate_pdp(&ec600n, "v-internet", NULL, NULL) == EC600N_OK) {
          ec600n_log("[APP] 4G Data Connected! IP: %s\r\n", ec600n.net_info.ip_address);

          /* 7. Query Real-Time Clock */
          ec600n_time_t now;
          if (ec600n_get_time(&ec600n, &now) == EC600N_OK) {
            ec600n_log("[APP] Network Time: 20%02d-%02d-%02d %02d:%02d:%02d\r\n",
                       now.year, now.month, now.day, now.hour, now.minute, now.second);
          }

          /* 8. Ping test to verify internet connectivity */
          uint32_t rtt = 0;
          if (ec600n_ping(&ec600n, "8.8.8.8", &rtt) == EC600N_OK) {
            ec600n_log("[APP] Ping 8.8.8.8 success: RTT = %lu ms\r\n", rtt);
          }
        } else {
          ec600n_log("[APP] Failed to activate PDP context\r\n");
        }
      } else {
        ec600n_log("[APP] Network registration timeout\r\n");
      }
    } else {
      ec600n_log("[APP] SIM card not ready (Status code: %d)\r\n", (int)ec600n.sim_status);
    }
  } else {
    ec600n_log("[APP] EC600N not responding. Check baudrate and wiring!\r\n");
  }
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  uint32_t last_heartbeat = 0;
  uint32_t last_rest_poll = 0;
  while (1)
  {
    /* Service EC600N event loop and incoming DMA / URC data */
    ec600n_process(&ec600n);

    /* Heartbeat LED blink every 1 second */
    if (HAL_GetTick() - last_heartbeat >= 1000) {
      last_heartbeat = HAL_GetTick();
      HAL_GPIO_TogglePin(LED_MCU_GPIO_Port, LED_MCU_Pin);
    }

    /* Optional: Periodic REST API poll from backend every 30 seconds */
    if (HAL_GetTick() - last_rest_poll >= 30000) {
      last_rest_poll = HAL_GetTick();
      if (ec600n.net_info.ip_address[0] != '\0') {
        /* Example: Poll custom backend REST endpoint */
        /* irrigation_poll_backend_api(&ec600n, "http://api.yourdomain.com/irrigation/control", NULL, 0); */
      }
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 15;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 5;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
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
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
