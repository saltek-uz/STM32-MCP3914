/*

  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
   STM32 with MCP3914 ADC - 3 channels of Voltage and 3 of Current 
   Ver. 1.000
  
    realized:
      1. Callback interface reading for ADC - 9 775,85 Hz
      2. Interrupt by integer periods of incoming signal
      3. RMS calculations
      4. Translate RMS values to optoisolated interface

    need to realize (urgent):

      5. Calculations for power (different for positive and negative)
      6. Count the summary energy (both) for  period beetween some ticks of sensor
      7. Translate Power and counter values to optoisolated interface
      8. Optimize & review code

  
*/

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "math.h"
#include "stdio.h"
#include "arm_math.h"

#define		 CS_LOW 	GPIOC->BSRR = GPIO_BSRR_BR4;
#define 	 CS_HIGH 	GPIOC->BSRR = GPIO_BSRR_BS4;


/* Private variables ---------------------------------------------------------*/
IWDG_HandleTypeDef hiwdg;
SPI_HandleTypeDef hspi1;
UART_HandleTypeDef huart2;

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_IWDG_Init(void);


const uint16_t _frame_size = 50 ; // equals to 25 integer periods of incomind sinusoide
uint8_t  spi_rx[25];    // 2 buffers for receive/transmit SPI (0x41 - is opcode for reading from 0th register of MCP3914
uint8_t  spi_tx[25] = {0x41,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

uint8_t  block_ready = 0, energy_ready = 0;       
uint8_t k;                      
uint16_t  i,j,n;                 
uint32_t a;                     

int32_t       adc[6] = {0,0,0,0,0,0};   
int32_t       adc_offset[6] = {0x00800CE0, 0x00800BB0, 0x00800CA0, 0x00800D00, 0x00801000, 0x00800F00};
//  int32_t       adc_offset[6] = {0x00800D00, 0x00800000, 0x00800000, 0x00800000, 0x00800000, 0x00800000};

uint64_t   sq_sum[6] = {0,0,0,0,0,0};    
 int64_t  pos_sum[3] = {0,0,0};          
 int64_t  neg_sum[3] = {0,0,0};         

float32_t pos_power[3];             // positive mult
float32_t neg_power[3];             // negative mult
int64_t _tmp;

float32_t pos_energy[3], _pos_energy[3];  
float32_t neg_energy[3], _neg_energy[3];
 
float32_t _rms[6];     
float32_t  rms[6];      

uint16_t _cnt =0, periods = 0;  

//------------------------------------------------------------------------------
 uint32_t Read3914(uint8_t address) // Simple read one register
{
	uint8_t read_address;
        uint8_t ii[4];
	uint32_t data;

	data=0;
	read_address=(address<<1);
	read_address|=0x41;
        
 CS_LOW;

  HAL_SPI_TransmitReceive(&hspi1, &read_address, ii, 4, 0x1000);

  CS_HIGH;
        
       data = (ii[1] << 16) + (ii[2] << 8) + ii[3];
	return(data);
}

//------------------------------------------------------------------------------
void Write3914(uint8_t address, uint8_t b1,uint8_t b2, uint8_t b3) // write register
{
	uint8_t write_address;
        uint8_t ii[4],jj[4];
	
 CS_LOW;
	write_address=(address<<1);
	write_address|=0x40;
        
         ii[0] = write_address; ii[1] = b1; ii[2] = b2; ii[3] = b3;
          
	HAL_SPI_TransmitReceive(&hspi1, ii, jj, 4, 0x1000);

CS_HIGH;

}

//------------------------------------------------------------------------------
void MCP_Init(void)     // Ïåðâè÷íàÿ íàñòðîéêà ÀÖÏ
{
uint8_t i;
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_5, GPIO_PIN_SET); // MCP_Reset pin

    Write3914(0x1F,0xA5,0,0);           // Unlock to change registers 
    Write3914(0x0E,0,0,0);              // 

    for (i=0;i<0x1F;i++) Read3914(0);   // 
    
    
    Write3914(0x1F,0xA5,0,0);    
    Write3914(0x0E,0,0,0); 

         Write3914(0x09,0,0,0);
         Write3914(0x0A,0,0,0);
         Write3914(0x0B,0x01,0x24,0x00);  // 
         Write3914(0x0C,0xA9,0,0);
         Write3914(0x0D,0xF8,0x60,0x50);

    Write3914(0x1F,0x00,0,0);    // after set up, lock settings for prevent unexpected changes of it 

}
//------------------------------------------------------------------------------

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) 
{
  if (GPIO_Pin == GPIO_PIN_1)  // here comes MCP_Data_Ready 
    {                          
      CS_LOW;                  
         HAL_IWDG_Refresh(&hiwdg);  
      while (!(  HAL_SPI_TransmitReceive_IT(&hspi1,spi_tx,spi_rx,19) == HAL_OK)) {}; // sending/receiving 19 bytes from MCP
    }	// ! Actually MCP3914 has independent 8 ADC channels, but I need just 6 of it
    else if (GPIO_Pin == GPIO_PIN_2)    // impulse from zero crossing detection sheme
    {
      HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_9); 
      periods++;                        
    } 
    

};

void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) // 19 bytes in the buffer now
{
    HAL_IWDG_Refresh(&hiwdg);   
  if (hspi == &hspi1) {            
    CS_HIGH;
     for (i=0;i<6;i++) 
      {
        adc[i] =  ((uint32_t)spi_rx[i*3+1] << 16) + ((uint32_t)spi_rx[i*3+2] << 8) + (uint32_t)spi_rx[i*3+3] ;
        adc[i] =  ( ( adc[i] ^ 0x00800000) - adc_offset[i]); 
      };

     for (i=0;i<6;i++) 
              sq_sum[i] = sq_sum[i] + (uint64_t) adc[i] *  (uint64_t) adc[i]; // sum of squares

    for (i=0;i<3;i++)  // energy 
    {
      switch (i)
      {
        case 0: _tmp = (int64_t) adc[0] * (int64_t) adc[5]; break;
        case 1: _tmp =  - (int64_t) adc[1] * (int64_t) adc[3]; break;
        case 2: _tmp = (int64_t) adc[2] * (int64_t) adc[4]; break; 
      };
      
      if (_tmp > 0) 
       {
          pos_sum[i] += _tmp;
       } 
      else  
       {
          neg_sum[i] += _tmp;
       }
    }

 _cnt++;

 if (periods > _frame_size)  
   {
     for (i=0;i<6;i++) 
      {
         _rms[i] = (double) sq_sum[i] / (_cnt+1);   
           sq_sum[i] =0;                                
            block_ready = 1;                      
            periods = 0;
      }

    for (i=0;i<3;i++) 
    {
      pos_power[i] =   pos_sum[i]  / 2000000; pos_power[i] = pos_power[i] / (_cnt+1);          
      neg_power[i] = - neg_sum[i]  / 2000000; neg_power[i] = neg_power[i] / (_cnt+1);
      pos_sum[i] = 0;
      neg_sum[i] = 0;
    }
     _cnt =0;
   }
        };
}

int main(void)
{

  HAL_Init();   /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  SystemClock_Config(); /* Configure the system clock */
  MX_GPIO_Init();
  MX_SPI1_Init();         /* Initialize all configured peripherals */
  MX_USART2_UART_Init(); 
  MX_IWDG_Init();

  MCP_Init();
  HAL_IWDG_Refresh(&hiwdg);

while (1)
  {
    HAL_IWDG_Refresh(&hiwdg); // 

    if (block_ready == 1)  // 
     {
       k =0xFE; HAL_UART_Transmit(&huart2,&k,1,1000); //  
        for(n=0; n<6; n++) 
         { 
           rms[n] = sqrt(_rms[n]); a = round(rms[n]); // 
           for(j=0; j<6; j++)                         // 
             {
                k = a & 0x0000000F; a = a >> 4; if (k < 10) k = k+ 48; else k =k + 55; 
                HAL_UART_Transmit(&huart2,&k,1,1000);  
             };
         };   

        for(n=0; n<3; n++)   // Power - positive 
         { 
           a = round(pos_power[n]); 
           for(j=0; j<6; j++)                         
             {
                k = a & 0x0000000F; a = a >> 4; if (k < 10) k = k+ 48; else k =k + 55; 
                HAL_UART_Transmit(&huart2,&k,1,1000);  
             };
         };   

        for(n=0; n<3; n++) // Power -negative
         { 
           a = round(neg_power[n]); 
           for(j=0; j<6; j++)                         
             {
                k = a & 0x0000000F; a = a >> 4; if (k < 10) k = k+ 48; else k =k + 55; 
                HAL_UART_Transmit(&huart2,&k,1,1000);  
             };
         };  
     
        k =0xFF; HAL_UART_Transmit(&huart2,&k,1,1000);  
            block_ready = 0;                                    
     };
  }; 

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 160;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* IWDG init function */
static void MX_IWDG_Init(void)
{

  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_8;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* SPI1 init function */
static void MX_SPI1_Init(void)
{

  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_16;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
     PC3   ------> I2S2_SD
     PA4   ------> I2S3_WS
     PB10   ------> I2S2_CK
     PC7   ------> I2S3_MCK
     PA9   ------> USB_OTG_FS_VBUS
     PA10   ------> USB_OTG_FS_ID
     PA11   ------> USB_OTG_FS_DM
     PA12   ------> USB_OTG_FS_DP
     PC10   ------> I2S3_CK
     PC12   ------> I2S3_SD
     PB6   ------> I2C1_SCL
     PB9   ------> I2C1_SDA
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PowerSwitchOn_GPIO_Port, OTG_FS_PowerSwitchOn_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : CS_I2C_SPI_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(CS_I2C_SPI_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_PowerSwitchOn_Pin PC4 PC5 */
  GPIO_InitStruct.Pin = OTG_FS_PowerSwitchOn_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PDM_OUT_Pin */
  GPIO_InitStruct.Pin = PDM_OUT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(PDM_OUT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_FALLING;//GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : I2S3_WS_Pin */
  GPIO_InitStruct.Pin = I2S3_WS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(I2S3_WS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : BOOT1_Pin */
  GPIO_InitStruct.Pin = BOOT1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(BOOT1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PE9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : CLK_IN_Pin */
  GPIO_InitStruct.Pin = CLK_IN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF5_SPI2;
  HAL_GPIO_Init(CLK_IN_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LD4_Pin LD3_Pin LD5_Pin LD6_Pin 
                           Audio_RST_Pin */
  GPIO_InitStruct.Pin = LD4_Pin|LD3_Pin|LD5_Pin|LD6_Pin 
                          |Audio_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : I2S3_MCK_Pin I2S3_SCK_Pin I2S3_SD_Pin */
  GPIO_InitStruct.Pin = I2S3_MCK_Pin|I2S3_SCK_Pin|I2S3_SD_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF6_SPI3;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : VBUS_FS_Pin */
  GPIO_InitStruct.Pin = VBUS_FS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(VBUS_FS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : OTG_FS_ID_Pin OTG_FS_DM_Pin OTG_FS_DP_Pin */
  GPIO_InitStruct.Pin = OTG_FS_ID_Pin|OTG_FS_DM_Pin|OTG_FS_DP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF10_OTG_FS;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_OverCurrent_Pin */
  GPIO_InitStruct.Pin = OTG_FS_OverCurrent_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(OTG_FS_OverCurrent_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : Audio_SCL_Pin Audio_SDA_Pin */
  GPIO_InitStruct.Pin = Audio_SCL_Pin|Audio_SDA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.Alternate = GPIO_AF4_I2C1;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  /*Configure GPIO pin : c2_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;//GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

/* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_IRQn);
  
  
   /*Configure GPIO pin : B0 */
  GPIO_InitStruct.Pin = GPIO_PIN_0;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct); 
  
/* USER CODE END 4 */ 
  
}



/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
