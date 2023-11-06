/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "stdio.h"
#include "string.h"
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
UART_HandleTypeDef huart2;

/* USER CODE BEGIN PV */
#define trig_TIM TIM3
char msg[]="\r\n Ejemplo con sensor HC SR-04 sin IC\n\r";
uint32_t numTicks;
const float Vel_Sonido=0.0343;
float distancia;
char msg_res[20]="";
uint8_t P_MED=0; //Variable de entorno

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void); //FUNCIONES CREADAS POR STMCUBE PARA LA COMUNICACIÓN PUERTO SERIE
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */

void trigger_TIM()
{

//---------CONFIGURACIÓN DE <BT> DEL TIMER 4---------------------------//

	trig_TIM->ARR = 19; //el período de mi señal va a ser 20us
	trig_TIM->PSC =83; //tengo una resolución Tu=1 microsegundo
	trig_TIM->CNT =0; //la cuenta empieza en 0

//------------------CONFIGURACIÓN INTERNA DEL TIMER 4--------------------------------------//

	trig_TIM->CR1 &=~(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(11<<5)|(1<<7)|(11<<8); //RESETEAMOS LAS CONFIGURACIONES
	trig_TIM->CR1 |=(1<<2)|(1<<7);//(1<<3)//activamos OPM "el contador se desactiva una vez que llega el evento UE"
	trig_TIM->DIER &=~(1<<0);//RESET
	trig_TIM->DIER |=(1<<0);// Activar evento UE
	trig_TIM->SMCR =0; //DESACTIVADO
	trig_TIM->CR2=0; // DESACTIVADO
//---------CONFIGURACIÓN MODO TOC PARA GENERAR SEÑAL 10us---------------------------//

	trig_TIM->CCMR1 =0; //Reset de TODAS las configuraciones
	trig_TIM->CCMR1 &=~(11<<0); //configuración <CANAL 1> como OUTPUT (TOC) vamos a sacar la señal PWM
	trig_TIM->CCMR1 |=(6<<4); //Configuramos como PWM con el primer semiciclo a "1"
	trig_TIM->CCER=0; //RESET de TODAS las posiciones
	trig_TIM->CCER |=(1<<0); //ACTIVAR el modo comparación del <CANAL 1>
	trig_TIM->CCR1 = 9; //Ponemos el CCR1 de manera que nos genere un DC=50% y esté encendido 0.5s

//----------------ACTIVACIÓN FINAL DEL TIMER-------------------------//

	trig_TIM->EGR = 1; //actualizacion de eventos
	trig_TIM->SR &=~1; //limpiar el flag
	trig_TIM->CR1 |=(1<<0); //Habilitar el TIMER


}
//void TIMER_MED_TIC()
//{
 /*	//---------CONFIGURACIÓN DE <BT> DEL TIMER 4---------------------------//

		trig_TIM->ARR = 19; //el período de mi señal va a ser 20us
		trig_TIM->PSC =83; //tengo una resolución Tu=1 microsegundo
		trig_TIM->CNT =0; //la cuenta empieza en 0

	//------------------CONFIGURACIÓN INTERNA DEL TIMER 4--------------------------------------//

		trig_TIM->CR1 &=~(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(11<<5)|(1<<7)|(11<<8); //RESETEAMOS LAS CONFIGURACIONES
		trig_TIM->CR1 |=(1<<3)|(1<<2)|(1<<7);//activamos OPM "el contador se desactiva una vez que llega el evento UE"
		trig_TIM->DIER &=~(1<<0);//RESET
		trig_TIM->DIER |=(1<<0);// Activar evento UE
		trig_TIM->SMCR =0; //DESACTIVADO
		trig_TIM->CR2=0; // DESACTIVADO
	//---------CONFIGURACIÓN MODO TOC PARA GENERAR SEÑAL 10us---------------------------//

		trig_TIM->CCMR1 =0; //Reset de TODAS las configuraciones
		trig_TIM->CCMR1 &=~(11<<0); //configuración <CANAL 1> como OUTPUT (TOC) vamos a sacar la señal PWM
		trig_TIM->CCMR1 |=(6<<4); //Configuramos como PWM con el primer semiciclo a "1"
		trig_TIM->CCER=0; //RESET de TODAS las posiciones
		trig_TIM->CCER |=(1<<0); //ACTIVAR el modo comparación del <CANAL 1>
		trig_TIM->CCR1 = 9; //Ponemos el CCR1 de manera que nos genere un DC=50% y esté encendido 0.5s

	//----------------ACTIVACIÓN FINAL DEL TIMER-------------------------//

		trig_TIM->EGR = 1; //actualizacion de eventos
		trig_TIM->SR &=~1; //limpiar el flag
		trig_TIM->CR1 |=(1<<0); //Habilitar el TIMER
		*/
//}
void clk_cnfig()
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); //reloj puerto A
	__HAL_RCC_GPIOC_CLK_ENABLE(); //reloj puerto C
	//RCC->APB1ENR |=(1<<2);// reloj TIM4
	RCC->APB1ENR |=(1<<1);// reloj TIM3
}
void EXTII_CNFIG()
{
	SYSCFG->EXTICR[3] &= (1111 << 4);// reset
	SYSCFG->EXTICR[3] |= ( 1 << 5); //Habilita interrupcion puerto C
	EXTI->IMR |= (1 << 13);
	EXTI->RTSR &= ~(1 << 13);//reset
	EXTI->FTSR |= (1 << 13);//flanco bajada activado
	//NVIC
	NVIC_EnableIRQ(EXTI15_10_IRQn);

}
void GPIO_CONF(void){

GPIOC->MODER &= ~(11 << (13*2)); //entrada PA13 INPUT

GPIOA->MODER &= ~(11 << (9*2)); //entrada PA9
//-----------CONF SEÑAL TRIGGER MODO TOC----------------------------//

GPIOA->MODER &=~ (11 << (6*2)); //RESET PA6
GPIOA->MODER |= (10 << (6*2)); //salida AFR PA6
GPIOA->AFR[0] =0;//RESET
GPIOA->AFR[0] |=(1<<(6*4+1));// configuramos columna AF02 del puerto A en pin 6 relacionado con TIM3

//-------------LED DE ESTADO DE INTERRUPCIÓN------------------------//

GPIOA->MODER |= (01 << (5*2)); //salida PA5
GPIOA->OTYPER &= ~(1 << (5)); //Push-Pull PA5

}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1<<13))
  {
	   P_MED=1;
	   GPIOA->BSRR |= (1<<5); //PROBAMOS SI SE EJECUTA EXTI
  }
EXTI->PR |= (1 << 13); // Clear interrupt}
}
/*void TIM4_IRQHandler(){
	if((TIM4->SR & (1<<0)) == 1) { //Es un evento de update
	GPIOA->ODR^=(1<<5); //Toggle PA5
	TIM4 ->SR &=~(1<<0); //reset del flag
	}
}*/
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef _GNUC_
/*With GCC, small printf (option LD Linker->Libraries->Small
printf set to 'Yes') calls __io_putchar()*/
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#define GETCHAR_PROTOTYPE int __io_getchar(void)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#define GETCHAR_PROTOTYPE int fgetc(FILE *f)
#endif /*_GNUC_*/
PUTCHAR_PROTOTYPE
{
HAL_UART_Transmit(&huart2,(uint8_t*)&ch,1,100);
return ch;
}
GETCHAR_PROTOTYPE
{
/* Place your implementation of fgetc here
e.g. write a character to the EVAL_COM1 and Loop until the
end of transmission */
char ch;
HAL_UART_Receive(&huart2, (uint8_t *)&ch, 1, 0xFFFF);
return ch;
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	setvbuf (stdin, NULL, _IONBF, 0);
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
  MX_USART2_UART_Init();
  clk_cnfig();
  GPIO_CONF();
  EXTII_CNFIG();
  trigger_TIM();//envia la PWM para activar el sensor de SONIDO
  //NVIC->ISER[0] |= (1 << 30);// ACTIVAR INTERRUPCION TIMER 4

HAL_UART_Transmit(&huart2,(char *) msg, strlen(msg),100);

  /* USER CODE BEGIN 2 */

printf(msg);
char msg1[]="Ahora vamos a medir distancias";
HAL_UART_Transmit(&huart2,(char *) msg1, strlen(msg1),100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
{

  if (P_MED==1)
  {
       printf("Medida\n\r");



       while((GPIOA->IDR & (1 << 9)) == (uint32_t) 0); // 2. Wait for ECHO pin rising edge in PA9
       numTicks = 0; // 3. Start measuring ECHO Pulse width in us

      while((GPIOA->IDR & (1<<9))==(uint32_t)(1<<9))
      {
	     numTicks++;
	     //usDelay(2);

      }

    distancia = (numTicks + 0.0f) *1.2* Vel_Sonido/2;

    sprintf(msg_res,"Distance (cm)= %.f\r\n", distancia);
    HAL_UART_Transmit(&huart2, (uint8_t *) msg_res, strlen(msg_res), 100);
    HAL_Delay(1000);

 } else

 {
	 continue;
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 7;
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

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
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
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
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
