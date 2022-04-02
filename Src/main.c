/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2022 STMicroelectronics.
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
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
//#include "ssd1306.h"
#include "sh1106.h"
#include "MAX31855.h"
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
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

I2C_HandleTypeDef hi2c1;

IWDG_HandleTypeDef hiwdg;

SPI_HandleTypeDef hspi2;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM2_Init(void);
static void MX_SPI2_Init(void);
static void MX_IWDG_Init(void);
/* USER CODE BEGIN PFP */
void TIM2_IRQHandler(void);
void Breizinheim(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void screen10(void);
void screen1(void);
void screen2(void);
void screen3(void);
char R[10]="Ch323";
uint16_t EncoderVal,y1,y2,target1=200,target2,choise_enc;
char Char_RX[64]="Ch323\r\n";
char Char_TX[64];


volatile uint16_t dma[2];
volatile int16_t adc[2];
uint8_t i,P1=4,I1=2,D1=1,P2=0,I2=0,D2=0,X1,X2,x_1,x_2,Blynk=0,flash_flag=0,flag=1,Button=0,choise_en=1,choise=0,choise_h=0,choise_t=0;
int16_t errorsold1[256];
int16_t  Error1;
int16_t errorsold2[256];
int16_t  Error2;
int32_t regD2=0,regD1=0,regP2=0,regP1=0,regI1=0,regI2=0,PID1,PID2;
uint8_t Buf[4];
MAX31855_Sensor termocoupe_up, termocoupe_down;


void  HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
  {
    adc[0]=dma[0]/10-50;
    adc[1]=dma[1]/10-50;
    
    HAL_ADC_Stop_DMA(&hadc1);
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
  MX_TIM3_Init();
  MX_TIM2_Init();
  MX_USB_DEVICE_Init();
  MX_SPI2_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  ssd1306_Init();
  HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
  HAL_TIM_PWM_Start(&htim2,TIM_CHANNEL_ALL);
  HAL_GPIO_WritePin(SPI2_CS_T1_GPIO_Port,SPI2_CS_T1_Pin,GPIO_PIN_SET); 
  HAL_GPIO_WritePin(SPI2_CS_T2_GPIO_Port,SPI2_CS_T2_Pin,GPIO_PIN_SET); 
  //NVIC_EnableIRQ(TIM2_IRQn); 
  uint32_t oldtimeB=HAL_GetTick();
  uint32_t oldtimeA=HAL_GetTick();
  
  //----------Horse---------------
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(32,16); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(64,32); 
  startScreen();
  ssd1306_Fill(Black);
  ssd1306_SetCursor(96,16); 
  startScreen();

  uint32_t set1,set2,set3,set4,flash_ret;
  uint8_t iter;
  //-------read from flash--------  
  FLASH_EraseInitTypeDef Erase;
  set1=*(__IO uint32_t*)User_Page_Adress;    //4 adresa
  set3=*(__IO uint32_t*)User_Page_Adress4;    //4 adresa
  set4=*(__IO uint32_t*)User_Page_Adress8;    //4 adresa
  target1=set1;
  target2=set1>>16;
  if (target1>1000){target1=100;}
  if (target2>1000){target2=100;}
  P1=set3;
  I1=set3>>8;
  D1=set3>>16;
  P2=set4;
  I2=set4>>8;
  D2=set4>>16;

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
          //----------Every scan---------------
    
    
    EncoderVal=__HAL_TIM_GET_COUNTER(&htim3);
    if (HAL_GPIO_ReadPin(Choise_heater_GPIO_Port,Choise_heater_Pin)==GPIO_PIN_RESET)
    {
      choise_h=0;
    }else{
      choise_h=1;
    }
    if (HAL_GPIO_ReadPin(Choise_t_GPIO_Port,Choise_t_Pin)==GPIO_PIN_RESET)
    {
      choise_t=0;
    }else{
      choise_t=1;
    }
     
    //--------------Blynk-------------
    if (HAL_GetTick()>(oldtimeA+200))
    {
      HAL_IWDG_Refresh(&hiwdg);
      oldtimeA=HAL_GetTick();
      Blynk=~Blynk&0x01;
      if (HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_4)==GPIO_PIN_RESET)
      {
        Button=1;
      }else
      {
        Button=0;
      }
         for (iter=0; iter<58; iter++)
      {
         if ((Char_RX[iter]=='C')&&(Char_RX[iter+1]=='h')&&(Char_RX[iter+2]=='5'))
        {
          target1=((Char_RX[iter+4]<<8)&0xFF00)|Char_RX[iter+3];
          if (target1>1000){target1=100;}
        }
         if ((Char_RX[iter]=='C')&&(Char_RX[iter+1]=='h')&&(Char_RX[iter+2]=='6'))
        {
          target2=((Char_RX[iter+4]<<8)&0xFF00)|Char_RX[iter+3];
          if (target2>1000){target2=100;}
        }
        if ((Char_RX[iter]=='C')&&(Char_RX[iter+1]=='h')&&(Char_RX[iter+2]=='7'))
        {
          if (Char_RX[iter+3]=='3'){flag=3;}
          if ((Char_RX[iter+3]=='2')&&(flag==3)){Button=1;}
        }
      } 
    
          
    
      //-------------screen choise-------------
     switch(flag){
     case 1:
       
       screen1();
     break;
     case 2:
       screen2();
     break;
     case 3:
       screen3();
     break;   
     case 10:
       screen10();
     break;  
     } 
      
    }
    

   
    
    
    //---------------startuem---------------
    /*
    if ((flag==3)&&(Button==0))
    {
              
    }
    */
    if ((flag==3)&&(Button==1)){
        flag=1;
        Button=0;
        NVIC_DisableIRQ(EXTI3_IRQn);  //external interrupt enable
        //HAL_TIM_Base_Stop_IT(&htim2); 
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_13, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(GPIOB,GPIO_PIN_12, GPIO_PIN_RESET);
    }
    
    
    //-----------time_int PID+ADC+USB 500ms-------
    if (HAL_GetTick()>(oldtimeB+500))
    {
      oldtimeB=HAL_GetTick();
      //----pid1-----      
      Error1=target1-adc[0];   
      regI1=0;
      for (iter=0; iter<I1; iter++)  
      {
        regI1=regI1+errorsold1[iter];
      } 
      regP1=Error1*P1;  //???????????????? ?????
      regI1 = regI1 + Error1; // ???????????? ????? 
      regD1=(Error1-errorsold1[0])*D1; //????????????????? ????? 
      if (regD1<(-10000))
      {
        regD1=(-10000);
      }
      if (regD1>(10000))
      {
        regD1=(10000);
      }
      if (regI1<(-10000))
      {
        regI1=(-10000);
      }
      if (regI1>(10000))
      {
        regI1=(10000);
      }
      for (iter=254; iter>0; iter--)
      {
      errorsold1[iter+1]=errorsold1[iter];
      } 
      errorsold1[0]=Error1;
      if (Error1!=0)    //???? ????? +-1
      {
        PID1=regP1+regI1+regD1; 
        if (PID1<0)
        {
          X1=0;
        }
        if (PID1>=10000)
        {
          X1=100;
        }
        if((PID1>=0)&&(PID1<10000))
        {
          X1=PID1/100; //koeff moshnosti
        }
      }
      //----pid2-----
      Error2=target2-adc[1];
      regI2=0;
      for (iter=0; iter<I2; iter++) 
      {
        regI2=regI2+errorsold2[iter];
      }       
      regP2=Error2*P2;  //???????????????? ?????
      regI2 = regI2 + Error2; // ???????????? ????? 
      regD2=(Error2-errorsold2[0])*D2; //????????????????? ????? 
      if (regD2<(-10000))
      {
        regD2=(-10000);
      }
      if (regD2>(10000))
      {
        regD2=(10000);
      }
      if (regI2<(-10000))
      {
        regI2=(-10000);
      }
      if (regI2>(10000))
      {
        regI2=(10000);
      }
      for (iter=254; iter>0; iter--)
      {
      errorsold2[iter+1]=errorsold2[iter];
      } 
      errorsold2[0]=Error2;
      if (Error2!=0) //???? ????? +-1
      {
        PID2=regP2+regI2+regD2; 
        if (PID2<0)
        {
          X2=0;
        }
        if (PID2>=10000)
        {
          X2=100;
        }
        if((PID2>=0)&&(PID2<10000))
        {
          X2=PID2/100; //koeff moshnosti
        }
               
      }

      //----------DMA---------
      if (choise_t){
      HAL_ADC_Start_DMA(&hadc1,(uint32_t*)&dma,2); 
      }else{
        
        HAL_GPIO_WritePin(SPI2_CS_T1_GPIO_Port,SPI2_CS_T1_Pin,GPIO_PIN_RESET);
        HAL_SPI_Receive(&hspi2,Buf,4,100);
        HAL_GPIO_WritePin(SPI2_CS_T1_GPIO_Port,SPI2_CS_T1_Pin,GPIO_PIN_SET); 
        termocoupe_up.Buf=Buf;
        MAX31855_convert_buf(&termocoupe_up);
        HAL_GPIO_WritePin(SPI2_CS_T2_GPIO_Port,SPI2_CS_T2_Pin,GPIO_PIN_RESET);
        HAL_SPI_Receive(&hspi2,Buf,4,100);
        HAL_GPIO_WritePin(SPI2_CS_T2_GPIO_Port,SPI2_CS_T2_Pin,GPIO_PIN_SET); 
        termocoupe_down.Buf=Buf;
        MAX31855_convert_buf(&termocoupe_down);   
        if (termocoupe_up.OC|termocoupe_up.SCG|termocoupe_up.SCV)
        {
          adc[0]=500;
        }else{
          adc[0]=(int16_t)termocoupe_up.t_termocoupe;
        }
        if (termocoupe_down.OC|termocoupe_down.SCG|termocoupe_down.SCV)
        {
          adc[1]=500;
        }else{
          adc[1]=(int16_t)termocoupe_down.t_termocoupe;
        }
        
      }
      
     
    //  sprintf(Char_TX,"%s %d %s %d %s %d %s %d %s %d %s", "Up-" , adc[0], "Down-" , adc[1], "state" , flag, "setUp" , target1, "setD" , target2, "\r\n");
     // CDC_Transmit_FS(Char_TX, strlen(Char_TX)); 
    
    }  
    
    
     //----flash_write---
    if (flash_flag==2)  
    {

      set2=(D1<<16)&0x00FF0000;
      set3=set2;
      set2=(I1<<8)&0x0000FF00;
      set3|=set2;
      set2=P1&0xFF;  
      set3|=set2;
      set2=(target2<<16)&0xFFFF0000;
      set1=set2;
      set2=target1;   
      set1=set1|set2;
      set2=(D2<<16)&0x00FF0000;
      set4=set2;
      set2=(I2<<8)&0x0000FF00;
      set4|=set2;
      set2=P2&0xFF;  
      set4|=set2;
      
      HAL_FLASH_Unlock();
      Erase.TypeErase=FLASH_TYPEERASE_PAGES;
      Erase.PageAddress=User_Page_Adress;
      Erase.NbPages=1;
      HAL_FLASHEx_Erase(&Erase,&flash_ret);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress,set1);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress4,set3);
      HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD ,User_Page_Adress8,set4);
      HAL_FLASH_Lock();
      flash_flag=0;
      
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
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
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_USB;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV4;
  PeriphClkInit.UsbClockSelection = RCC_USBCLKSOURCE_PLL;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_71CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.ClockSpeed = 400000;
  hi2c1.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c1.Init.OwnAddress1 = 44;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_16;
  hiwdg.Init.Reload = 4095;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES_RXONLY;
  hspi2.Init.DataSize = SPI_DATASIZE_16BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_128;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 470;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 50;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 1;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI1;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_10|SPI2_CS_T1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI2_CS_T2_GPIO_Port, SPI2_CS_T2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC14 PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_14|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA2 PA3 PA4 PA5
                           PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pins : PB0 PB1 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : Choise_t_Pin Choise_heater_Pin PB4 PB5 */
  GPIO_InitStruct.Pin = Choise_t_Pin|Choise_heater_Pin|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB10 SPI2_CS_T1_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_10|SPI2_CS_T1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : PB12 PB8 PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_12|GPIO_PIN_8|GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI2_CS_T2_Pin */
  GPIO_InitStruct.Pin = SPI2_CS_T2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI2_CS_T2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB3 */
  GPIO_InitStruct.Pin = GPIO_PIN_3;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */
//-------------------------screen1 MAIN------------------------
void screen1() 
{
  if (choise_en==1){
    choise_enc=EncoderVal;
  }
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,4);
  ssd1306_WriteString("  Up T:", Font_7x10, White);  
  sprintf(R,"%d",target1);
  ssd1306_SetCursor(49,0);
  if (choise_enc%5==1)
  {
    ssd1306_WriteString(R, Font_11x18, Blynk);
    if ((Button==1)&&(choise_en==1))
    {
      choise_en=0;
      Button=0;
      __HAL_TIM_SET_COUNTER(&htim3,target1);
    }
    if (choise_en==0)
    {
      target1=EncoderVal;
      if (Button==1)
      {
        choise_en=1;
        Button=0;
        __HAL_TIM_SET_COUNTER(&htim3,0);
        flash_flag=2;
      }
    }
  } else {
    ssd1306_WriteString(R, Font_11x18, White);
  }
  ssd1306_SetCursor(90,0);
  if (choise_t){
    ssd1306_WriteString("ADC", Font_7x10, White);
  }else{
    ssd1306_WriteString("Digit", Font_7x10, White);
  }
  ssd1306_SetCursor(0,26);
  ssd1306_WriteString("Down T:", Font_7x10, White); 
  ssd1306_SetCursor(49,22);
  sprintf(R,"%d",target2);
    if (choise_enc%5==2)
  {
    ssd1306_WriteString(R, Font_11x18, Blynk);
    if ((Button==1)&&(choise_en==1))
    {
      choise_en=0;
      Button=0;
      __HAL_TIM_SET_COUNTER(&htim3,target2);
    }
    if (choise_en==0)
    {
      target2=EncoderVal;
      if (Button==1)
      {
        choise_en=1;
        Button=0;
        __HAL_TIM_SET_COUNTER(&htim3,0);
        flash_flag=2;
      }
    }
  } else {
    ssd1306_WriteString(R, Font_11x18, White);
  }

  ssd1306_SetCursor(0,52);
    if (choise_enc%5==3)
  {
    ssd1306_WriteString("PID", Font_7x10, Blynk);
    if (Button==1)
    {
        Button=0;
        __HAL_TIM_SET_COUNTER(&htim3,P1);
        flag=2;
        choise=0;
    }
   
  } else {
    ssd1306_WriteString("PID", Font_7x10, White);
  }
  
  ssd1306_SetCursor(30,44);
  if (choise_enc%5==4)
  {
    ssd1306_WriteString("Start", Font_11x18, Blynk);
    if (Button==1)
    {
      flag=3;
      Button=0;
      NVIC_EnableIRQ(EXTI3_IRQn);  //external interrupt enable
    }
    
  } else {
    ssd1306_WriteString("Start", Font_11x18, White);
  }
  
  ssd1306_UpdateScreen();

}


//-------------------------screen2 PID------------------------
void screen2() 
{
  if (Button){
    choise++;
    Button=0;
    if (choise==1){__HAL_TIM_SET_COUNTER(&htim3,I1);}
    if (choise==2){__HAL_TIM_SET_COUNTER(&htim3,D1);}
    if (choise==3){__HAL_TIM_SET_COUNTER(&htim3,P2);}
    if (choise==4){__HAL_TIM_SET_COUNTER(&htim3,I2);}
    if (choise==5){__HAL_TIM_SET_COUNTER(&htim3,D2);}
    if (choise==6){ flag=1; flash_flag=2;}
  }
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("PID 1:", Font_7x10, White);
  ssd1306_SetCursor(64,0);
  ssd1306_WriteString("PID 2:", Font_7x10, White);
  ssd1306_SetCursor(0,20);
  ssd1306_WriteString("P1-", Font_7x10, White);
  ssd1306_SetCursor(64,20);
  ssd1306_WriteString("P2-", Font_7x10, White);
  ssd1306_SetCursor(0,34);
  ssd1306_WriteString("I1-", Font_7x10, White);
  ssd1306_SetCursor(64,34);
  ssd1306_WriteString("I2-", Font_7x10, White);
  ssd1306_SetCursor(0,48);
  ssd1306_WriteString("D1-", Font_7x10, White);
  ssd1306_SetCursor(64,48);
  ssd1306_WriteString("D2-", Font_7x10, White);
  ssd1306_SetCursor(21,20);
  sprintf(R,"%d",P1);
  
  if (choise==0)
  {
    P1=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }
  ssd1306_SetCursor(85,20);
  sprintf(R,"%d",P2);
  if (choise==3)
  {
    P2=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }
  ssd1306_SetCursor(21,34);
  sprintf(R,"%d",I1);
  if (choise==1)
  {
    I1=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }
  ssd1306_SetCursor(85,34);
  sprintf(R,"%d",I2);
  if (choise==4)
  {
    I2=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }
  ssd1306_SetCursor(21,48);
  sprintf(R,"%d",D1);
  if (choise==2)
  {
    D1=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }
  ssd1306_SetCursor(85,48);
  sprintf(R,"%d",D2);
  if (choise==5)
  {
    D2=EncoderVal;
    ssd1306_WriteString(R, Font_7x10, Blynk);
  } else {
    ssd1306_WriteString(R, Font_7x10, White);
  }

  ssd1306_UpdateScreen();

}
     
//-------------------------screen3 work------------------------
void screen3() 
{
  
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("Set:", Font_7x10, White);
  ssd1306_SetCursor(63,0);
  ssd1306_WriteString("Up", Font_7x10, White);
  ssd1306_SetCursor(80,0);
  sprintf(R,"%d%s",X1,"%");
  ssd1306_WriteString(R, Font_7x10, White);
  
  ssd1306_SetCursor(0,15);
  ssd1306_WriteString("Cur:", Font_7x10, White);
  ssd1306_SetCursor(0,33);
  ssd1306_WriteString("Set:", Font_7x10, White);
  ssd1306_SetCursor(63,33);
  ssd1306_WriteString("Down", Font_7x10, White);
  ssd1306_SetCursor(94,33);
  sprintf(R,"%d%s",X2,"%");
  ssd1306_WriteString(R, Font_7x10, White);
  
  
  
  ssd1306_SetCursor(0,49);
  ssd1306_WriteString("Cur:", Font_7x10, White);

  sprintf(R,"%d",target1);
  ssd1306_SetCursor(28,0);
  ssd1306_WriteString(R, Font_7x10, White);
  sprintf(R,"%d",adc[0]);  //adc value
  ssd1306_SetCursor(28,11);
  ssd1306_WriteString(R, Font_11x18, White); 
  
  sprintf(R,"%d",target2);
  ssd1306_SetCursor(28,33);
  ssd1306_WriteString(R, Font_7x10, White);
  sprintf(R,"%d",adc[1]);
  ssd1306_SetCursor(28,45);
  ssd1306_WriteString(R, Font_11x18, White); 
  ssd1306_SetCursor(65,45);
  heater(choise_h);
  
  
  NVIC_DisableIRQ(EXTI3_IRQn);  //external interrupt enable
  ssd1306_UpdateScreen();
  NVIC_EnableIRQ(EXTI3_IRQn);  //external interrupt enable

}

//------------------------OUT-------------------------
void Breizinheim(void) 
{
  i++;
  uint8_t x;
  x=i*x_1/100;
    if (x==y1)
    {   //off
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_RESET);
    }
    else 
    {   //on
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_1, GPIO_PIN_SET);
    }
  y1=x;
  x=i*x_2/100;
    if (x==y2)
    {   //off
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_RESET);
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10, GPIO_PIN_RESET);
    }
    else 
    {   //on 
      HAL_GPIO_WritePin(GPIOB,GPIO_PIN_0, GPIO_PIN_SET);
      if (choise_h){HAL_GPIO_WritePin(GPIOB,GPIO_PIN_10, GPIO_PIN_SET);}
    }
  y2=x;
  if (i==100)
  {
    i=0; y1=0; y2=0;x_1=X1;x_2=X2;
  }
}

//-------------------------interrupt-----------------------


void EXTI3_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
    HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
    Breizinheim();
}


//-------------------------screen10------------------------
void screen10() {
  
  ssd1306_Fill(Black);
  ssd1306_SetCursor(0,0);
  ssd1306_WriteString("ADC 1:", Font_11x18, White);
  sprintf(R,"%d",adc[0]);
  ssd1306_WriteString(R, Font_11x18, White);
  ssd1306_SetCursor(0, 18);
  ssd1306_WriteString("PID ", Font_16x26, White);
  sprintf(R,"%d",X1);
  ssd1306_WriteString(R, Font_16x26, White);
  ssd1306_SetCursor(0, 45);
  ssd1306_WriteString("ADC 2:", Font_11x18, White);
  sprintf(R,"%d",adc[1]);
  ssd1306_WriteString(R, Font_11x18, White);
  ssd1306_UpdateScreen();
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1)
  {
  }
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
