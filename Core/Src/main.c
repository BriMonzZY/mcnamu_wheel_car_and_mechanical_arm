/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */


#include "Mcnamu_wheel_drive.h"
#include "SEEKFREE_IPS114_SPI.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

int encoder1, encoder2, encoder3, encoder4, encoder_filter1[5], encoder_filter2[5], encoder_filter3[5], encoder_filter4[5], encoder_filter_index = 0;
float wheel1_speed = 0, wheel2_speed = 0, wheel3_speed = 0, wheel4_speed = 0;

PID wheel_speed_1;
PID wheel_speed_2;
PID wheel_speed_3;
PID wheel_speed_4;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* printf重定向 */
int fputc(int ch, FILE *f)
{
	uint8_t temp[1] = {ch};
	HAL_UART_Transmit(&huart1, temp, 1, 2);//huart1需要根据你的配置修改
	return ch;
}


/**
 * @breif:			编码器数值读取
 *
 */
int Read_Encoder(uint8_t TIMx)
{
	int Encoder_TIM;
	switch(TIMx) {
		case 1: Encoder_TIM= (short)TIM1 -> CNT;  TIM1 -> CNT=0;break;
		case 2: Encoder_TIM= (short)TIM2 -> CNT;  TIM2 -> CNT=0;break;
		case 3: Encoder_TIM= (short)TIM3 -> CNT;  TIM3 -> CNT=0;break;
		case 4: Encoder_TIM= (short)TIM4 -> CNT;  TIM4 -> CNT=0;break;
		case 5: Encoder_TIM= (short)TIM5 -> CNT;  TIM5 -> CNT=0;break;
		case 6: Encoder_TIM= (short)TIM6 -> CNT;  TIM6-> CNT=0;break;
		case 7: Encoder_TIM= (short)TIM7 -> CNT;  TIM7 -> CNT=0;break;
		case 8: Encoder_TIM= (short)TIM8 -> CNT;  TIM8 -> CNT=0;break;
		#if 1
		case 9: Encoder_TIM= (short)TIM9 -> CNT;  TIM9 -> CNT=0;break;
		case 10: Encoder_TIM= (short)TIM10 -> CNT;  TIM10 -> CNT=0;break;
		case 11: Encoder_TIM= (short)TIM11 -> CNT;  TIM11 -> CNT=0;break;
		case 12: Encoder_TIM= (short)TIM12 -> CNT;  TIM12 -> CNT=0;break;
		case 13: Encoder_TIM= (short)TIM13 -> CNT;  TIM13 -> CNT=0;break;
		case 14: Encoder_TIM= (short)TIM14 -> CNT;  TIM14 -> CNT=0;break;
		#endif
	}
	return Encoder_TIM;
}



/* *****无线串口接收***** */

uint8_t rx_buff;
//uint8_t rx_buff2[3];
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	int16_t time;

#if 0	
	HAL_UART_Transmit(&huart5,&rx_buff,1,100);  /* 数据回传 */
	printf("%d\n", rx_buff2[0]);
	printf("%d\n", rx_buff2[1]);
	printf("%d\n", rx_buff2[2]);
#endif
	
	printf("%d\n", rx_buff);
	
	while(HAL_UART_Receive_IT(&huart5,&rx_buff,1) != HAL_OK){
		time++;
		if(time > 200){
			huart1.RxState = HAL_UART_STATE_READY;
			__HAL_UNLOCK(&huart5);
		}
	}
}


void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET) 
	{
			__HAL_UART_CLEAR_OREFLAG(huart);
	}
	HAL_UART_Receive_IT(huart,&rx_buff,1);
}


/* 无线串口发送数据函数 */

void wireLessTransmit(uint8_t test[])
{
	uint8_t i;
	if(!HAL_GPIO_ReadPin(RTS_GPIO_Port,RTS_Pin)) printf("逐飞科技无线转串口测试\r\n");//一次发送数据最好不要超过30字节
	if(!HAL_GPIO_ReadPin(RTS_GPIO_Port,RTS_Pin)){
		i = 0;
		while(test[i] != '\0'){
			while(HAL_GPIO_ReadPin(RTS_GPIO_Port,RTS_Pin));   //等待模块可以接受数据
			HAL_UART_Transmit(&huart5,&test[i],1,100);
			i++;
		}
	}
}





/* 为PID变量申请内存，范围指向pp的指针 */
void PIDInit (PID *pp)   
{
	memset(pp, 0, sizeof(PID)); /* 字节的内容全部设置为ch指定的ASCII值，块的大小由第三个参数指定 */
}



double PIDCalc(PID *pp, double NextPoint);




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
  MX_USART1_UART_Init();
  MX_TIM1_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_TIM10_Init();
  MX_TIM11_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */
	
	/* IPS初始化 */
	ips114_init();
	ips114_clear(WHITE);
	ips114_showstr(0,0,(uint8 *)"strat init");
	HAL_Delay(100);
	
	/* 启动uart5中断 */
	HAL_UART_Receive_IT(&huart5,&rx_buff,1);
	__HAL_UART_ENABLE_IT(&huart5, UART_IT_ERR);
	
	HAL_TIM_Encoder_Start(&htim1, TIM_CHANNEL_ALL);  /* 启动编码器模式 */
	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL);
	HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);
	HAL_TIM_Base_Start_IT(&htim5);   /* 定时器5使能 */
	/* 初始化电机PWM */
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim8,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim9,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim11,TIM_CHANNEL_1);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim8, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim9, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim11, TIM_CHANNEL_1, 0);
	
	
	
	PIDInit(&wheel_speed_1);
	PIDInit(&wheel_speed_2);
	PIDInit(&wheel_speed_3);
	PIDInit(&wheel_speed_4);
	
	wheel_speed_1.Kp = 2.55;  /* 0.26 0.035 0.017 */
	wheel_speed_1.Ki = 0.053;
	wheel_speed_1.Kd = 0.03;
	wheel_speed_1.SetPoint = 0;  /* 满速500 */
	
	wheel_speed_2.Kp = 2.55;
	wheel_speed_2.Ki = 0.053;
	wheel_speed_2.Kd = 0.03;
	wheel_speed_2.SetPoint = 0;  /* 满速500 最低速20 */
	
	wheel_speed_3.Kp = 2;
	wheel_speed_3.Ki = 0.05;
	wheel_speed_3.Kd = 0.02;
	wheel_speed_3.SetPoint = 0;  /* 15ms 满速500 */
	
	wheel_speed_4.Kp = 2.55;  // 0.009
	wheel_speed_4.Ki = 0.053;
	wheel_speed_4.Kd = 0.03;  //0.002
	wheel_speed_4.SetPoint = 0;  /* 满速500 */
	
	


//	wheel1_speed_set(200);
//	wheel2_speed_set(200);
//	wheel3_speed_set(200);
//	wheel4_speed_set(200);

//	all_directions(35, 90, 0);
//	HAL_GPIO_WritePin(BEEP_GPIO_Port, BEEP_Pin, GPIO_PIN_RESET);

	/* 初始化完成 */
	printf("init\n\n");
	ips114_showstr(0,0,(uint8 *)"init finished");
	HAL_Delay(100);
	ips114_clear(WHITE);
	ips114_display_chinese(0,1*16,16,chinese_test[8],3,RED);
	ips114_display_chinese(0,3*16,16,chinese_test[24],3,BLUE);
	ips114_display_chinese(5*16,3*16,16,chinese_test[14],3,BLUE);
	ips114_display_chinese(10*16,3*16,16,chinese_test[20],2,BLUE);
	ips114_showuint16(0,5,2021);
	
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while(1)
  {
		//printf("%d\n", rx_buff);
		
#if 1
		if(rx_buff == 0x00) {  /* 停止 */
			wheel1_speed_set(0);
			wheel2_speed_set(0);
			wheel3_speed_set(0);
			wheel4_speed_set(0);
			wheel_speed_1.SetPoint = 0;
			wheel_speed_2.SetPoint = 0;
			wheel_speed_3.SetPoint = 0;
			wheel_speed_4.SetPoint = 0;
		}
		if(rx_buff == 0x01) {  /* 向前 */
			wheel_speed_1.SetPoint = 100;
			wheel_speed_2.SetPoint = 100;
			wheel_speed_3.SetPoint = 100;
			wheel_speed_4.SetPoint = 100;
		}
		if(rx_buff == 0x02) {  /* 向后 */
			wheel_speed_1.SetPoint = -100;
			wheel_speed_2.SetPoint = -100;
			wheel_speed_3.SetPoint = -100;
			wheel_speed_4.SetPoint = -100;
		}
		if(rx_buff == 0x03) {  /* 向左 */
			wheel_speed_1.SetPoint = 100;
			wheel_speed_2.SetPoint = -100;
			wheel_speed_3.SetPoint = 100;
			wheel_speed_4.SetPoint = -100;
		}
		if(rx_buff == 0x04) {  /* 向右 */
			wheel_speed_1.SetPoint = -100;
			wheel_speed_2.SetPoint = 100;
			wheel_speed_3.SetPoint = -100;
			wheel_speed_4.SetPoint = 100;
		}
#endif	
		
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
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 180;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
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
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

uint16_t tim_cnt_20 = 0;
uint16_t tim_cnt_15 = 0;
uint16_t tim_cnt_10 = 0;
int angle_tmp1 = 90;


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == htim5.Instance){
		tim_cnt_20++;
		tim_cnt_15++;
		tim_cnt_10++;
		
		/* 15ms编码器采样周期 进行滑动均值滤波 */
		if(tim_cnt_15 >= 15){
			int sum = 0;
			tim_cnt_15 = 0;
			
			encoder_filter1[encoder_filter_index] = -Read_Encoder(1);
			encoder_filter2[encoder_filter_index] = Read_Encoder(4);
			encoder_filter3[encoder_filter_index] = Read_Encoder(3);
			encoder_filter4[encoder_filter_index] = Read_Encoder(2);
			encoder_filter_index++;
			if(encoder_filter_index >= 5) encoder_filter_index = 0;
			
			for(uint8_t i = 0; i < 5; i++) {
				sum += encoder_filter1[i];
			}
			encoder1 = sum/5;
			sum = 0;
			
			for(uint8_t i = 0; i < 5; i++) {
				sum += encoder_filter2[i];
			}
			encoder2 = sum/5;
			sum = 0;
			
			for(uint8_t i = 0; i < 5; i++) {
				sum += encoder_filter3[i];
			}
			encoder3 = sum/5;
			sum = 0;
			
			for(uint8_t i = 0; i < 5; i++) {
				sum += encoder_filter4[i];
			}
			encoder4 = sum/5;
			sum = 0;
		}
		
		
		/* 速度闭环控制 */
		if(tim_cnt_20 >= 20) {
			tim_cnt_20 = 0;
			printf("%d, %.2f, %d, %.2f, %d, %.2f, %d, %.2f\n", encoder1, wheel1_speed, encoder2, wheel2_speed, encoder3, wheel3_speed, encoder4, wheel4_speed);

#if 1
			wheel1_speed = PIDCalc(&wheel_speed_1, encoder1);
			wheel1_speed = fabs(wheel1_speed);
			if(wheel_speed_1.SetPoint < 0) wheel1_speed *= -1;
			wheel1_speed_set(wheel1_speed);
			
			wheel2_speed = PIDCalc(&wheel_speed_2, encoder2);
			wheel2_speed = fabs(wheel2_speed);
			if(wheel_speed_2.SetPoint < 0) wheel2_speed *= -1;
			wheel2_speed_set(wheel2_speed);
			
			wheel3_speed = PIDCalc(&wheel_speed_3, encoder3);
			wheel3_speed = fabs(wheel3_speed);
			if(wheel_speed_3.SetPoint < 0) wheel3_speed *= -1;
			wheel3_speed_set(wheel3_speed);
			
			wheel4_speed = PIDCalc(&wheel_speed_4, encoder4);
			wheel4_speed = fabs(wheel4_speed);
			if(wheel_speed_4.SetPoint < 0) wheel4_speed *= -1;
			wheel4_speed_set(wheel4_speed);
#endif
			 
			//printf("%d,%d,%d,%d\n", encoder1, encoder2, encoder3, encoder4);
		}
		
		if(tim_cnt_10 >= 10){
			tim_cnt_10 = 0;
#if 1
			if(rx_buff == 0x05) {
				all_directions(150, angle_tmp1, 0);
				angle_tmp1 += 3;
				if(angle_tmp1 >= 360) angle_tmp1 = 0; 
			}
#endif
			
		}
	}
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
  __disable_irq();
  while (1)
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
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
