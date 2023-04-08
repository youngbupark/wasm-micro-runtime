#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdbool.h>
#include "stm32l4xx_hal.h"  
#include "stm32l4s5i_iot01.h"
#include "wifi.h"

/* Define the default wifi ssid and password. The user can override this 
   via -D command line option or via project settings.  */

#ifndef WIFI_SSID
//#error "Symbol WIFI_SSID must be defined."
#define WIFI_SSID "wifi_name"
#endif /* WIFI_SSID  */

#ifndef WIFI_PASSWORD
//#error "Symbol WIFI_PASSWORD must be defined."
#define WIFI_PASSWORD "wifi_password"
#endif /* WIFI_PASSWORD  */

/* WIFI Security type, the security types are defined in wifi.h.
  WIFI_ECN_OPEN = 0x00,         
  WIFI_ECN_WEP = 0x01,          
  WIFI_ECN_WPA_PSK = 0x02,      
  WIFI_ECN_WPA2_PSK = 0x03,     
  WIFI_ECN_WPA_WPA2_PSK = 0x04, 
*/
#ifndef WIFI_SECURITY_TYPE
//#error "Symbol WIFI_SECURITY_TYPE must be defined."
#define WIFI_SECURITY_TYPE  WIFI_ECN_WPA2_PSK
#endif /* WIFI_SECURITY_TYPE  */

#define TERMINAL_USE
#define USE_COM_PORT

#ifndef RETRY_TIMES
#define RETRY_TIMES 3
#endif

#ifdef TERMINAL_USE
extern ES_WIFIObject_t    EsWifiObj;
#endif /* TERMINAL_USE */

#define WIFI_FAIL 1
#define WIFI_OK   0

extern  SPI_HandleTypeDef hspi;

#ifdef USE_COM_PORT
UART_HandleTypeDef UartHandle;
#endif


typedef enum
{
  WS_IDLE = 0,
  WS_CONNECTED,
  WS_DISCONNECTED,
  WS_ERROR,
} WebServerState_t;
uint8_t  MAC_Addr[6];
uint8_t  IP_Addr[4]; 
uint8_t  Gateway_Addr[4];
uint8_t  DNS1_Addr[4]; 
uint8_t  DNS2_Addr[4]; 

RNG_HandleTypeDef rng;

int hardware_rand(uint32_t *random);

#define NX_RAND_RESEED_COUNT 64

static uint32_t _nx_rand_count = 0xFFFFFFFF;

int rand_int32()
{
uint32_t r1;

    if (_nx_rand_count >= NX_RAND_RESEED_COUNT)
    {
        _nx_rand_count = 0;
        
        if (!hardware_rand(&r1))
        {
            srand(r1);
        }
    }
   
    _nx_rand_count++;

    if (RAND_MAX < 0x7FFFFFFF)
    {
        if (RAND_MAX < 0xFFFF)
        {
            r1 = rand();

            return((rand() & 0x7FFF) | ((rand() & 0x7FFF) << 16) | ((r1 & 1) << 31) | ((r1 & 2) << 15));
        }
        else
        {
            return((rand() & 0xFFFF) | ((rand() & 0xFFFF) << 16));
        }
    }
    else
    {
        return((rand() & 0x7FFFFFFF) | ((rand() & 1) << 31));
    }
}

void HAL_RNG_MspInit(RNG_HandleTypeDef *hrng)
{

  hrng -> Instance = RNG;

  __HAL_RCC_RNG_CLK_ENABLE();

  __HAL_RNG_ENABLE(hrng);

}

int hardware_rand(uint32_t *random)
{
int status;  
   
    status = HAL_RNG_GenerateRandomNumber(&rng,random);
    
    if(status)
    {
        printf("generate random number error \r\n");     
    }
    
    return status;
}

void HAL_TIM_MspPostInit(TIM_HandleTypeDef* htim)
{

  GPIO_InitTypeDef GPIO_InitStruct;
  if(htim->Instance==TIM8)
  {
  /* USER CODE BEGIN TIM8_MspPostInit 0 */

  /* USER CODE END TIM8_MspPostInit 0 */
  
    /**TIM8 GPIO Configuration    
    PA5     ------> TIM8_CH1N
    PB14     ------> TIM8_CH2N 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_5;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_14;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* USER CODE BEGIN TIM8_MspPostInit 1 */

  /* USER CODE END TIM8_MspPostInit 1 */
  }

}


/* TIM8 init function */
static void TIM8_Init(void)
{
  TIM_HandleTypeDef htim8;
  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  __HAL_RCC_TIM8_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  
  htim8.Instance = TIM8;
  htim8.Init.Prescaler = 19999;
  htim8.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim8.Init.Period = 1999;
  htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim8.Init.RepetitionCounter = 0;
  HAL_TIM_Base_Init(&htim8);

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  HAL_TIM_ConfigClockSource(&htim8, &sClockSourceConfig);
  HAL_TIM_PWM_Init(&htim8);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig);

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_LOW;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1);

  HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2);

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.BreakFilter = 0;
  sBreakDeadTimeConfig.Break2State = TIM_BREAK2_DISABLE;
  sBreakDeadTimeConfig.Break2Polarity = TIM_BREAK2POLARITY_HIGH;
  sBreakDeadTimeConfig.Break2Filter = 0;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig);
  HAL_TIM_MspPostInit(&htim8);
  TIM8 -> CCMR1 &= ~(TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE);
  TIM8 -> CCR1 = 2000;
  TIM8 -> CCR2 = 2000;
  HAL_TIM_PWM_Start(&htim8, 2);
  HAL_TIM_PWM_Start(&htim8, 6);
  
}

/* Configure both LEDs to Off(0)/On(1)/Flashing(2)/Oneshot(3).  */
void board_led_config(int status)
{
    if (status == 0)
    {
        TIM8 -> CR1 = (TIM8 -> CR1 & (~TIM_CR1_OPM)) | TIM_CR1_CEN;
        TIM8 -> CCR1 = 2000;
        TIM8 -> CCR2 = 2000;
    }
    else if (status == 1)
    {
        TIM8 -> CR1 = (TIM8 -> CR1 & (~TIM_CR1_OPM)) | TIM_CR1_CEN;
        TIM8 -> CCR1 = 0;
        TIM8 -> CCR2 = 0;
    }
    else if (status == 2)
    {
        TIM8 -> CR1 = (TIM8 -> CR1 & (~TIM_CR1_OPM)) | TIM_CR1_CEN;
        TIM8 -> CCR1 = 999;
        TIM8 -> CCR2 = 999;
    }
    else if (status == 3)
    {
        TIM8 -> CCR1 = 1;
        TIM8 -> CCR2 = 1;
        TIM8 -> CNT = 0;
        TIM8 -> CR1 = TIM8 -> CR1 & ~TIM_CR1_CEN;

        TIM8 -> CNT;
        TIM8 -> CR1 |= TIM_CR1_OPM;
        TIM8 -> CR1 |= TIM_CR1_CEN;
    }
}

/**
* @brief This function handles EXTI line[15:10] interrupts.
*/
void EXTI15_10_IRQHandler(void)
{

  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_13);
}
/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follows :
  *            System Clock source            = PLL (MSI MSE)
  *            SYSCLK(Hz)                     = 80000000
  *            HCLK(Hz)                       = 80000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 1
  *            APB2 Prescaler                 = 1
  *            MSI Frequency(Hz)              = 48000000
  *            PLL_M                          = 6
  *            PLL_N                          = 20
  *            PLL_R                          = 2
  *            PLL_P                          = 7
  *            PLL_Q                          = 2
  *            Flash Latency(WS)              = 4
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /**Configure LSE Drive Capability 
  */
  HAL_PWR_EnableBkUpAccess();
  __HAL_RCC_LSEDRIVE_CONFIG(RCC_LSEDRIVE_LOW);
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSE|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSEState = RCC_LSE_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_11;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 20;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    while(1);
  }
  /**Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    while(1);
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USART1|RCC_PERIPHCLK_USART3
                              |RCC_PERIPHCLK_I2C2|RCC_PERIPHCLK_DFSDM1
                              |RCC_PERIPHCLK_USB|RCC_PERIPHCLK_RNG;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Usart3ClockSelection = RCC_USART3CLKSOURCE_PCLK1;
  PeriphClkInit.I2c2ClockSelection = RCC_I2C2CLKSOURCE_PCLK1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLLSAI1;
  PeriphClkInit.RngClockSelection = RCC_RNGCLKSOURCE_PLLSAI1;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 6;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 12;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV7;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_48M2CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    while(1);
  }
  /**Configure the main internal regulator output voltage 
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    while(1);
  }
  /**Enable MSI Auto calibration 
  */
  HAL_RCCEx_EnableMSIPLLMode();
}

void verify_and_correct_boot_bank(void)
{

int BFB2_bit;
int FB_MODE_bit;

    
    /* Enable FLASH registers.  */
    RCC ->AHB1ENR |= RCC_AHB1ENR_FLASHEN;
    
    /* Enable SYSCFG.  */
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGEN;
    
    BFB2_bit = (FLASH -> OPTR & FLASH_OPTR_BFB2) == FLASH_OPTR_BFB2 ? 1 : 0;
    FB_MODE_bit = (SYSCFG -> MEMRMP & SYSCFG_MEMRMP_FB_MODE) == SYSCFG_MEMRMP_FB_MODE ? 1 : 0;
    
    if (BFB2_bit != FB_MODE_bit)
    {
        FLASH_OBProgramInitTypeDef OBInit;

        /* Allow Access to the Flash control registers and user Flash. */
        HAL_FLASH_Unlock();

        /* Clear OPTVERR bit set on virgin samples. */
        __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_OPTVERR);
        /* Allow Access to the option bytes sector. */
        HAL_FLASH_OB_Unlock();

        /* Get the Dual boot configuration status. */
        HAL_FLASHEx_OBGetConfig(&OBInit);

        /* Enable/Disable dual boot feature */
        OBInit.OptionType = OPTIONBYTE_USER;
        OBInit.USERType = OB_USER_BFB2;

        OBInit.USERConfig = ((OBInit.USERConfig & OB_BFB2_ENABLE) == OB_BFB2_ENABLE) ?
                            OB_BFB2_DISABLE : OB_BFB2_ENABLE;

        if (HAL_FLASHEx_OBProgram(&OBInit) != HAL_OK)
        { 
            /* Failed setting the option bytes configuration. */
            HAL_FLASH_Lock();
        }
        else
        {
            /* Start the Option Bytes programming process */
            if (HAL_FLASH_OB_Launch() != HAL_OK)
            {
                HAL_FLASH_Lock();
            }
        }
    }
}

int  board_setup(void)
{

uint32_t  retry_connect=0;
uint32_t random;

  /* Enable execution profile.  */
  CoreDebug -> DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
  DWT -> CTRL |= DWT_CTRL_CYCCNTENA_Msk;

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  
  verify_and_correct_boot_bank();
  
  /* Configure the system clock */
  SystemClock_Config();

#ifdef USE_COM_PORT
  UartHandle.Init.BaudRate       = 115200;
  UartHandle.Init.WordLength     = UART_WORDLENGTH_8B;
  UartHandle.Init.StopBits       = UART_STOPBITS_1;
  UartHandle.Init.Parity         = UART_PARITY_NONE;
  UartHandle.Init.HwFlowCtl      = UART_HWCONTROL_NONE;
  UartHandle.Init.Mode           = UART_MODE_TX_RX;
  UartHandle.Init.OverSampling   = UART_OVERSAMPLING_16;
  UartHandle.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  BSP_COM_Init(COM1, &UartHandle);
#endif

  /* Initialize the hardware random number generator.  */
  HAL_RNG_Init(&rng);
  
  hardware_rand(&random);
  
  srand(random);

  /* Configure TIM8 for LED flashing.  */
  TIM8_Init();
  
  /* Configure push button.  */
  BSP_PB_Init(BUTTON_USER, BUTTON_MODE_EXTI);
      
  /*Initialize and use WIFI module */
  if(WIFI_Init() ==  WIFI_STATUS_OK)
  {
      
#if defined (TERMINAL_USE)      
      /* Lib info.  */
      uint32_t hal_version = HAL_GetHalVersion();
      uint32_t bsp_version = BSP_GetVersion();
      
      printf("STM32L4XX Lib:\r\n");
      printf("> CMSIS Device Version: %d.%d.%d.%d.\r\n", __STM32L4_CMSIS_VERSION_MAIN, __STM32L4_CMSIS_VERSION_SUB1, __STM32L4_CMSIS_VERSION_SUB2, __STM32L4_CMSIS_VERSION_RC);
      printf("> HAL Driver Version: %ld.%ld.%ld.%ld.\r\n", ((hal_version >> 24U) & 0xFF), ((hal_version >> 16U) & 0xFF), ((hal_version >> 8U) & 0xFF), (hal_version & 0xFF));
      printf("> BSP Driver Version: %ld.%ld.%ld.%ld.\r\n", ((bsp_version >> 24U) & 0xFF), ((bsp_version >> 16U) & 0xFF), ((bsp_version >> 8U) & 0xFF), (bsp_version & 0xFF));
      
      /* ES-WIFI info.  */
      printf("ES-WIFI Firmware:\r\n");
      printf("> Product Name: %s\r\n", EsWifiObj.Product_Name);
      printf("> Product ID: %s\r\n", EsWifiObj.Product_ID);
      printf("> Firmware Version: %s\r\n", EsWifiObj.FW_Rev);
      printf("> API Version: %s\r\n", EsWifiObj.API_Rev);      
#endif /* TERMINAL_USE */     
    
      if(WIFI_GetMAC_Address(MAC_Addr) == WIFI_STATUS_OK)
      {       
#if defined (TERMINAL_USE)    
        printf("ES-WIFI MAC Address: %X:%X:%X:%X:%X:%X\r\n",     
               MAC_Addr[0],
               MAC_Addr[1],
               MAC_Addr[2],
               MAC_Addr[3],
               MAC_Addr[4],
               MAC_Addr[5]);   
#endif /* TERMINAL_USE */
      }
      else
      {
#if defined (TERMINAL_USE)  
        printf("!!!ERROR: ES-WIFI Get MAC Address Failed.\r\n");
#endif /* TERMINAL_USE */ 
        return WIFI_FAIL;
      }
    
      while((retry_connect++) < RETRY_TIMES)
      {   
        printf("wifi connect try %ld times\r\n",retry_connect);
        if( (WIFI_Connect(WIFI_SSID, WIFI_PASSWORD, WIFI_SECURITY_TYPE) == WIFI_STATUS_OK))
    {
#if defined (TERMINAL_USE)  
      printf("ES-WIFI Connected.\r\n");
#endif /* TERMINAL_USE */ 
      
      if(WIFI_GetIP_Address(IP_Addr) == WIFI_STATUS_OK)
      {
#if defined (TERMINAL_USE)
        printf("> ES-WIFI IP Address: %d.%d.%d.%d\r\n",     
               IP_Addr[0],
               IP_Addr[1],
               IP_Addr[2],
               IP_Addr[3]); 

        if(WIFI_GetGateway_Address(Gateway_Addr) == WIFI_STATUS_OK)
        {
            printf("> ES-WIFI Gateway Address: %d.%d.%d.%d\r\n",     
                   Gateway_Addr[0],
                   Gateway_Addr[1],
                   Gateway_Addr[2],
                   Gateway_Addr[3]); 
        }
        
        if(WIFI_GetDNS_Address(DNS1_Addr,DNS2_Addr) == WIFI_STATUS_OK)
        {
            printf("> ES-WIFI DNS1 Address: %d.%d.%d.%d\r\n",     
            DNS1_Addr[0],
            DNS1_Addr[1],
            DNS1_Addr[2],
            DNS1_Addr[3]); 
            
            printf("> ES-WIFI DNS2 Address: %d.%d.%d.%d\r\n",     
            DNS2_Addr[0],
            DNS2_Addr[1],
            DNS2_Addr[2],
            DNS2_Addr[3]);           
        }
 #endif /* TERMINAL_USE */       
        
        if((IP_Addr[0] == 0)&& 
           (IP_Addr[1] == 0)&& 
           (IP_Addr[2] == 0)&&
           (IP_Addr[3] == 0))
          return WIFI_FAIL;  
      }
      else
      {    
#if defined (TERMINAL_USE)  
        printf("!!!ERROR: ES-WIFI Get IP Address Failed.\r\n");
#endif /* TERMINAL_USE */
        return WIFI_FAIL;
      }
          
          break;
        }
    }
    if(retry_connect > RETRY_TIMES)
    {
      
#if defined (TERMINAL_USE)  
        printf("!!!ERROR: ES-WIFI NOT connected.\r\n");
#endif /* TERMINAL_USE */ 
        return WIFI_FAIL;
    }
  }
  else
  {   
#if defined (TERMINAL_USE)  
    printf("!!!ERROR: ES-WIFI Initialize Failed.\r\n"); 
#endif /* TERMINAL_USE */
    return WIFI_FAIL;
  }
  
  
  return WIFI_OK;
}

#ifdef USE_COM_PORT
size_t _write(int handle, const unsigned char *buf, size_t bufSize)
{

    /* Check for the command to flush all handles */
    if (handle == -1) 
    {    
        return 0;  
    }    
    /* Check for stdout and stderr      (only necessary if FILE descriptors are enabled.) */  
    if (handle != 1 && handle != 2)  
    {    
        return -1;  
    }   

    if(HAL_UART_Transmit(&UartHandle, (uint8_t*)buf, bufSize, 5000) != HAL_OK)
    {    
        return -1;  
    }    

    return bufSize;
}

size_t _read(int handle, unsigned char *buf, size_t bufSize)
{  

    /* Check for stdin      (only necessary if FILE descriptors are enabled) */ 
    if (handle != 0)  
    {    
        return -1;  
    }   

    if(HAL_UART_Receive(&UartHandle, (uint8_t *)buf, bufSize, 0x10000000) != HAL_OK)
    {    
        return -1;  
    }
    return bufSize;
}
#endif

__weak void user_button_callback()
{
}

void EXTI1_IRQHandler(void)
{
 HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_1);
}

void SPI3_IRQHandler(void)
{
  HAL_SPI_IRQHandler(&hspi);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  switch (GPIO_Pin)
  {
    case (GPIO_PIN_1):
    {
      SPI_WIFI_ISR();
      break;
    }
    case (USER_BUTTON_PIN):
    {
      user_button_callback();
      break;
    }
    default:
    {
      break;
    }
  }
}
