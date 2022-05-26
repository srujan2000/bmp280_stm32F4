/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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
void config_clk(void);
void config_gpio(void);
void config_i2c(void);
void i2c_start(void);
void i2c_stop(void);
void i2c_address(unsigned char);
void i2c_write(unsigned char);
void set_bmp280(void);
void get_temp(int);
void print_value();
long temp_calc(long);
void uart2_config(void);
void send_char(unsigned char);
void send_string(char*);

#include "main.h"
#include <stdio.h>
#include <string.h>

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

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

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
unsigned char data_buffer[20];
long raw_temp;
int temperature;
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
  /* USER CODE BEGIN 2 */
   config_clk();
   config_gpio();
   config_i2c();
   uart2_config();

   i2c_start();
   i2c_address(0xEC); //Address of BMP280

   set_bmp280();
   delay1();
   i2c_stop();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	   get_temp(3);//3 bytes as temp is 20bit data
	   print_value();
	   HAL_Delay(500);
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}


void config_clk(){
	RCC->APB1ENR |= RCC_APB1ENR_I2C1EN;
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;
}

void config_gpio(){
    GPIOB->MODER |= GPIO_MODER_MODE9_1| GPIO_MODER_MODE8_1; //alternate mode
    GPIOB->MODER &= ~(GPIO_MODER_MODE9_0)| ~(GPIO_MODER_MODE8_0);

    GPIOB->OTYPER |= GPIO_OTYPER_OT8 |GPIO_OTYPER_OT9; //open drain
    GPIOB->OSPEEDR|= GPIO_OSPEEDER_OSPEEDR8_0 |GPIO_OSPEEDER_OSPEEDR8_1|GPIO_OSPEEDER_OSPEEDR9_0 |GPIO_OSPEEDER_OSPEEDR9_1;

    GPIOB->PUPDR |= GPIO_PUPDR_PUPD8_0 | GPIO_PUPDR_PUPD9_0; //pull-up
    GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPD8_1) | ~(GPIO_PUPDR_PUPD9_1);

    GPIOB->AFR[1] = 0x00000044UL;
}

void config_i2c(){
	 I2C1->CR1 |=  I2C_CR1_SWRST; //reset
     I2C1->CR1 &= ~(I2C_CR1_SWRST);

     I2C1->CR2 |= (16<<0); //f_clk 16Mhz
     I2C1->CCR |= 80<<0; //i2c-100khz
     I2C1->TRISE = 17;

     I2C1->CR1 |= I2C_CR1_PE; // I2C enable
}

void i2c_start(){
    I2C1->CR1 |= I2C_CR1_START; //i2c start
    while(!(I2C1->SR1 & I2C_SR1_SB)); //wait till start bit is set
}

void i2c_stop(){
   I2C1->CR1 |= I2C_CR1_STOP;//i2c stop
}

void i2c_address(unsigned char addr){
    I2C1->DR = addr;
    while(!(I2C1->SR1 & I2C_SR1_ADDR)); //wait till address bit is set
    unsigned char temp = I2C1->SR1 | I2C1->SR2;
}

void i2c_write(unsigned char data){
	while(!(I2C1->SR1 & I2C_SR1_TXE));//wait till data register is empty
	I2C1->DR = data;
	while(!(I2C1->SR1 & I2C_SR1_BTF));//wait till byte transfer is complete
}

void set_bmp280(){
	i2c_write(0xF4);//Mode and sampling regsiter
	i2c_write(0x23);//Normal Mode ,16x sampling
	i2c_write(0xF5);//config register
	i2c_write(0x80);//standby_mode-500 and filter and NO SPI
}

void get_temp(int byteno){
    int i=0;
    i2c_start();
    i2c_address(0xEC);
    i2c_write(0xFA);//setting pointer to MSB of temperature register

    i2c_start();
    i2c_address(0xED);//address + Read
    unsigned char temp = I2C1->SR1 | I2C1->SR2; //clearing SR1 and SR2

    for(i=0;i<byteno-1;i++){
       I2C1->CR1 |= I2C_CR1_ACK;//Acknowledge
       while(!(I2C1->SR1 & I2C_SR1_RXNE)); //wait till data register is empty
       data_buffer[i] = I2C1->DR; //read the data from DR
    }

    I2C1->CR1 &= ~(I2C_CR1_ACK);//no ACK
    while(!(I2C1->SR1 & I2C_SR1_RXNE));//wait till data register is empty
    data_buffer[i] = I2C1->DR;

    i2c_stop();
}

void print_value(){
	char str2[10];

	raw_temp = (((long)data_buffer[0]<<12) | ((long)data_buffer[1]<<4)|((long)data_buffer[2]>>4))& 0xFFFFFFFF ;
    temperature = temp_calc(raw_temp);

	sprintf(str2,"%d",temperature);
	for(int i=0;i<strlen(str2);i++){
		send_char(str2[i]);
		if(i==1){
			send_char('.');
		}
	}
	send_char('C');
	send_char('\n');

}

long temp_calc(long adc_T){
	long int t_fine;
	long signed int var1, var2, T;
	unsigned short dig_T1 = 27504;
	short dig_T2 = 26435;
	short dig_T3 = -1000;
	var1 = ((((adc_T >> 3) - ((long int)dig_T1 << 1))) * ((long int)dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((long int)dig_T1)) * ((adc_T >> 4) - ((long int)dig_T1))) >> 12) * ((long int)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

void uart2_config(void){
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN; //Enable UART clk
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  //Enable GPIO A

	GPIOA->MODER |= GPIO_MODER_MODE2_1|GPIO_MODER_MODE3_1;
	GPIOA->MODER &=  ~(GPIO_MODER_MODE2_0)| ~(GPIO_MODER_MODE3_0);

	GPIOA->OSPEEDR |= GPIO_OSPEEDER_OSPEEDR2_0 |GPIO_OSPEEDER_OSPEEDR2_1|GPIO_OSPEEDER_OSPEEDR3_0|GPIO_OSPEEDER_OSPEEDR3_1; //High speed for pin2 and 3;

	GPIOA->AFR[0] |= 0x00007700UL; //Alternate AF7(usart2) for pin and Alternate AF8(usart2) for pin2

	USART2->CR1 = 0x00;
	USART2->CR1 |= USART_CR1_UE; //usart enable
	USART2->CR1 &= ~(USART_CR1_M);; //8-bit word length
	USART2->BRR = (3<<0) | (104<<4); //baud rate of 9600 at 16Mhz

	USART2->CR1 |= USART_CR1_RE; //rx enable
	USART2->CR1 |= USART_CR1_TE;//tx enable

    USART2->CR1 |= USART_CR1_TXEIE; //tx interrupt enable
    USART2->CR1 |= USART_CR1_TCIE; //TCIE interrupt enable
}

void send_char(unsigned char data){
	USART2->DR = data;
	while(!(USART2->SR & (USART_SR_TC))); //Wait for tx complete
}

void send_string(char *str){
	for(int i=0;i<strlen(str);i++){
		send_char(str[i]);
	}
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
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
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
