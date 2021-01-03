/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "mbedtls.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "open62541.h"
#include "cert.h"
#include "privateKey.h"
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

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .priority = (osPriority_t) osPriorityNormal,
  .stack_size = 512
};
/* Definitions for OPC_UA */
osThreadId_t OPC_UAHandle;
const osThreadAttr_t OPC_UA_attributes = {
  .name = "OPC_UA",
  .priority = (osPriority_t) osPriorityRealtime,
  .stack_size = 4096
};
/* USER CODE BEGIN PV */
UA_Server* mUaServer;
UA_NodeId outputNodeId;
UA_NodeId inputNodeId;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
void StartDefaultTask(void *argument);
void opcua_thread(void *argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  /* Up to user define the empty MX_MBEDTLS_Init() function located in mbedtls.c file */

  /* USER CODE BEGIN 2 */
  MX_LWIP_Init();
  MX_MBEDTLS_Init();
  /* USER CODE END 2 */
  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of OPC_UA */
  OPC_UAHandle = osThreadNew(opcua_thread, NULL, &OPC_UA_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
 
  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
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
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 432;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Activate the Over-Drive mode 
  */
  if (HAL_PWREx_EnableOverDrive() != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_7) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
UA_Double
computeOutput(UA_Double lastValue, UA_Double input) {
    return 0.07165 * input + 0.9299 * lastValue;
}

static void
updateOutputCallback(UA_Server *server, void *data) {
    UA_Variant processOutputVariant;
    UA_Variant inputVariant;
    UA_Server_readValue(server, outputNodeId, &processOutputVariant);
    UA_Server_readValue(server, inputNodeId, &inputVariant);

    UA_Double* processOutputValue = (UA_Double*) processOutputVariant.data;
    UA_Double* inputValue = (UA_Double*) inputVariant.data;
    UA_Double processOutput = computeOutput(*processOutputValue, *inputValue);

    UA_Variant newValue;
    UA_Variant_setScalar(&newValue, &processOutput, &UA_TYPES[UA_TYPES_DOUBLE]);
    UA_Server_writeValue(server, outputNodeId, newValue);
}

static void
addVariable(UA_Server *server, char *variableName, UA_Double initialValue) {
    /* Define the attribute of the variable variable node */
    UA_VariableAttributes attr = UA_VariableAttributes_default;
    UA_Double variable = initialValue;
    UA_Variant_setScalar(&attr.value, &variable, &UA_TYPES[UA_TYPES_DOUBLE]);
    attr.description = UA_LOCALIZEDTEXT("en-US", variableName);
    attr.displayName = UA_LOCALIZEDTEXT("en-US", variableName);
    attr.dataType = UA_TYPES[UA_TYPES_DOUBLE].typeId;
    attr.accessLevel = UA_ACCESSLEVELMASK_READ | UA_ACCESSLEVELMASK_WRITE;

    /* Add the variable node to the information model */
    UA_NodeId variableNodeId = UA_NODEID_STRING(1, variableName);
    UA_QualifiedName qualifiedVariableName = UA_QUALIFIEDNAME(1, variableName);
    UA_NodeId parentNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_OBJECTSFOLDER);
    UA_NodeId parentReferenceNodeId = UA_NODEID_NUMERIC(0, UA_NS0ID_ORGANIZES);
    UA_Server_addVariableNode(server, variableNodeId, parentNodeId,
                              parentReferenceNodeId, qualifiedVariableName,
                              UA_NODEID_NUMERIC(0, UA_NS0ID_BASEDATAVARIABLETYPE), attr, NULL, NULL);

    return variableNodeId;
}
/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_opcua_thread */
/**
* @brief Function implementing the OPC_UA thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_opcua_thread */
void opcua_thread(void *argument)
{
  /* USER CODE BEGIN opcua_thread */
    UA_ByteString c = UA_STRING_NULL;
    c.data = certificate;
    c.length = cert_len;

    UA_ByteString prvKey = UA_STRING_NULL;
    prvKey.data = privateKey;
    prvKey.length = privateKey_len;

    size_t trustListSize = 1;
    UA_ByteString trustList[trustListSize];
    trustList[0] = UA_STRING_NULL;
    trustList[0].data = certificate;
    trustList[0].length = cert_len;

    size_t issuerListSize = 0;
    UA_ByteString *issuerList = NULL;

    size_t revocationListSize = 0;
    UA_ByteString *revocationList = NULL;

    mUaServer = UA_Server_new();
    UA_ServerConfig *uaServerConfig = UA_Server_getConfig(mUaServer);

    UA_StatusCode retval = UA_ServerConfig_setDefaultWithSecurityPolicies(uaServerConfig, 4840,
                                                                          &c, &prvKey,
                                                                          trustList, trustListSize,
                                                                          issuerList, issuerListSize,
                                                                          revocationList, revocationListSize);
    UA_Server_addRepeatedCallback(mUaServer, updateOutputCallback, NULL, 1000, NULL);
    if (!retval) {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    UA_ServerConfig_setCustomHostname(uaServerConfig, UA_STRING("192.168.000.102"));
    UA_Boolean running = true;

    char* inputVariableName = "process-input";
    char* outputVariableName = "process-output";
    addVariable(mUaServer, inputVariableName, 0L);
    addVariable(mUaServer, outputVariableName, 0L);
    inputNodeId = UA_NODEID_STRING(1, inputVariableName);
    outputNodeId = UA_NODEID_STRING(1, outputVariableName);
//
//    UA_ValueCallback callback;
//    callback.onRead = beforeRead;
//    callback.onWrite = afterWrite;
//    UA_Server_setVariableNode_valueCallback(mUaServer, outputNodeId, callback);

    // server should run in loop after invocation of function line below
    retval = UA_Server_run(mUaServer, &running);

    // execution should never reach that point if everything works fine
    UA_Server_delete(mUaServer);
  /* USER CODE END opcua_thread */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */

  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
