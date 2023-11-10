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
#define deb_TIM TIM5
#define med_TIM TIM2
#define trig_TIM TIM3
uint8_t PUL=0;// variable para imprimir por pantalla lo que ocurre
uint8_t aux=0; //esta variable nos avisará si entra en la interrupción BOTÓN PC13
uint8_t TON=0; //esta variable avisará si entra en la interrupción del TIMER5 (deboucing)
char msg[]="\r\n Ejemplo con sensor HC SR-04 sin IC\n\r";
uint32_t numTicks1=0; //almacena valor de la primera captura
uint32_t numTicks2=0; // almacena valor de la segunda captura
uint8_t i=0; //controlar el impreso por pantalla del resultado
uint8_t captura=1; //para llevar un control de la captura que se está realizando
char msg_1[50]="";//variable que nos muestra los avisos de las diferentes fases que va pasando el programa
char msg_res[20]=""; //variable que almacena la respuesta de medición


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/

void SystemClock_Config(void); //FUNCIONES CREADAS POR STMCUBE PARA LA COMUNICACIÓN PUERTO SERIE
static void MX_GPIO_Init(void);
static void MX_USART2_UART_Init(void);

/* USER CODE BEGIN PFP */
void deboucing_TIMMER()
{
 //-------------------------CONFIGURAR BT DEL TIMMER 5-----------------------------//
/* En este caso deseamos que el TIMER genere una demora de aproximadamente unos 250ms que sería el tiempo que tarda
 * el botón (elemento metálico) en estabilizarse y no generar rebotes.
 */
 deb_TIM->CNT=0; //la cuenta del timmer empieza en 0
 deb_TIM->PSC=8399; //conseguimos una resolución Tu=0,1ms
 deb_TIM->ARR=2500; //conseguimos una demora de 250ms que es el tiempo suficiente para evitar los rebotes

 //-----------------CONFIGURACIÓN INTERNA DEL TIMMER 5---------------------------//

 deb_TIM->CR1=0; //RESETEAMOS TODAS LAS CONFIGURACIONES solo queremos que se comporte como temporizador y dejamos APAGADO
 deb_TIM->CR2=0; //no necesitamos este modo así que lo dejamos deshabilitado
 deb_TIM->SMCR =0; //DESACTIVADO
 deb_TIM->CCMR1 =0; //Reset de TODAS las configuraciones
 deb_TIM->CCER=0; // DESACTIVADO

 //-------------------ACTUALIZAMOS los eventos y limpiamos los flag-------------------//

 deb_TIM->EGR=0;
 deb_TIM->EGR |=(1<<0); //ACTUALIZAMOS registros genera un UPDATE
 deb_TIM->SR=0; //limpiamos el flag

//---------------------CONFIGURAMOS la interrupción del TIMER-----------------------//

 deb_TIM->DIER=0;
 deb_TIM->DIER |=(1<<0); //Habilitamos la interrupción cuando ocurra evento UPDATE

 NVIC_EnableIRQ(TIM5_IRQn); //habilitamos interrupción Timer avisamos al NVIC

}
void trigger_TIM()
{
//---------CONFIGURACIÓN DE <BT> DEL TIMER 3---------------------------//

	trig_TIM->ARR = 35999; //el período de mi señal va a ser 46ms  (10 us + 10000us + 36000us)
	trig_TIM->PSC =83; //tengo una resolución Tu=1 microsegundo
	trig_TIM->CNT =0; //la cuenta empieza en 0

//------------------CONFIGURACIÓN INTERNA DEL TIMER 3--------------------------------------//

	trig_TIM->CR1 &=~(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(11<<5)|(1<<7)|(11<<8); //RESETEAMOS LAS CONFIGURACIONES
	trig_TIM->CR1 |=(1<<2)|(1<<7);//|(1<<3);//activamos OPM "el contador se desactiva una vez que llega el evento UE"
	trig_TIM->SMCR =0; //DESACTIVADO
	trig_TIM->CR2=0; // DESACTIVADO

//---------CONFIGURACIÓN MODO TOC PARA GENERAR SEÑAL 10us---------------------------//

	trig_TIM->CCMR1 =0; //Reset de TODAS las configuraciones
	trig_TIM->CCMR1 &=~(11<<0); //configuración <CANAL 1> como OUTPUT (TOC) vamos a sacar la señal PWM
	trig_TIM->CCMR1 |=(6<<4); //Configuramos como PWM con el primer semiciclo a "1"
	trig_TIM->CCER=0; //RESET de TODAS las posiciones
	//trig_TIM->CCER |=(1<<1); //activo a low
	trig_TIM->CCER |=(1<<0); //ACTIVAR el modo comparación del <CANAL 1>
	trig_TIM->CCR1 = 9; //Ponemos el CCR1 de manera que nos genere un DC=50% y esté encendido 0.5s

//----------------ACTIVACIÓN FINAL DEL TIMER 3----------------------------------//
	trig_TIM->EGR=0;
	trig_TIM->EGR |= (1<<0); //actualizacion de eventos GENERAMOS un UPDATE antes de activar la interrupción para que no genere
	//ninguna señal antes de tiempo
	trig_TIM->SR =0; //limpiar el flag
	trig_TIM->DIER &=~(1<<0);//RESET
	trig_TIM->DIER |=(1<<0);// Activar evento UPDATE


}
void TIMER_MED_TIC()
{
 //---------CONFIGURACIÓN DE <BT> DEL TIMER 2---------------------------//

	med_TIM->ARR = 35999; //el período de mi señal va a ser 65ms
/*Esto lo hacemos porque si el objeto está demasiado cerca o demasiado lejos el ancho de pulso que nos entrega el sensor
 * es de unos 36ms indicando que la medida está FUERA DE RANGO
 */	med_TIM->PSC =83; //tengo una resolución Tu=0.001ms  (1us)
	med_TIM->CNT =0; //la cuenta empieza en 0

//------------------CONFIGURACIÓN INTERNA DEL TIMER 2--------------------------------------//

	med_TIM->CR1 =0;//Desactivamos TODAS las fnciones de CR1
	med_TIM->CR2=0;//Desactivamos TODAS las funciones de CR2
	med_TIM->SMCR =0; //DESACTIVADO el modo esclavo

//---------CONFIGURACIÓN MODO TIC PARA CAPTURAR LA SEÑAL DE TIEMPO---------------------------//

	med_TIM->CCMR1 =0; //Reset de TODAS las configuraciones
	med_TIM->CCMR1 |=(01<<8); //configuración <CANAL 2> como INPUT (TIC) vamos a MEDIR señal tiempo de "ECHO"
	med_TIM->CCER=0; //RESET de TODAS las posiciones
	med_TIM->CCER |=(1<<4); //ACTIVAR el modo CAPTURA (TIC) del <CANAL 2>
    med_TIM->CCER |=(01<<5); //Activo por flanco de SUBIDA y flanco de BAJADA (detectar el rango de tiempo)
	//No queremos utilizar ningún FILTRO ni tampoco el modo PSC que emplea el propio registro CCER
    //NOTA--> EN VEZ DE PONER 2 LINEAS de código si pones <med_TIM->CCER |=(7<<4);> también funcionaría
//----------------ACTIVACIÓN FINAL DEL TIMER-------------------------//

	med_TIM->EGR = 0;
	med_TIM->EGR |=(1<<0);//actualizacion de eventos GENERAMOS un UPDATE antes de activar la interrupción para que no genere
	//ninguna señal antes de tiempo
	med_TIM->SR =0; //limpiar el flag
	med_TIM->DIER =0;//RESET
	med_TIM->DIER |=((1<<0)|(1<<2));// Se genera una interrupción al ocurrir una CAPTURA o un UPDATE <CANAL 2>

	NVIC_EnableIRQ(TIM2_IRQn); //habilitamos interrupción del TIMER2

}
void clk_cnfig()
{
	__HAL_RCC_GPIOA_CLK_ENABLE(); //reloj puerto A
	__HAL_RCC_GPIOC_CLK_ENABLE(); //reloj puerto C
	__HAL_RCC_GPIOB_CLK_ENABLE(); //Habilitamos reloj puerto B
	RCC->APB1ENR |=(1<<3);// reloj TIM5
	RCC->APB1ENR |=((1<<1)|(1<<0));// reloj TIM3 y TIM2
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
//-------------CONFIGURACIÓN BOTÓN USUARIO PC13-------------------//

GPIOC->MODER &= ~(11 << (13*2)); //entrada PA13 INPUT

//----------------CONFIGURACIÓN PIN DE MEDICIÓN TIEMPO MODO TIC-------------------//

GPIOB->MODER &= ~(11 << (3*2)); // reset de PB3 por este pin se realizará la medición de TIEMPO enviada por el sensor
	GPIOB->MODER |=(10<<(3*2));//Configuramos como AF el pin PB3
	GPIOB->AFR[0] &=~(1111<<(3*4));//Reset de las posiciones
	GPIOB->AFR[0] |=(1<<(3*4));//Configuración de AF01 para el PB3 relacionado con CH2 del TIM2

//-----------CONFIGURACIÓN SEÑAL TRIGGER MODO TOC----------------------------//

GPIOA->MODER &=~ (11 << (6*2)); //RESET PA6
GPIOA->MODER |= (10 << (6*2)); //salida AFR PA6
GPIOA->AFR[0] &=~(1111<<(6*4));//RESET DEL REGISTRO AF02 PA6 <CANAL 1>
GPIOA->AFR[0] |=(1<<(6*4)<<1);// configuramos columna AF02 del puerto A en pin 6 relacionado con TIM3 <CANAL 1>

//-------------LED DE ESTADO TIMER------------------------//

GPIOA->MODER |= (01 << (5*2)); //salida PA5
GPIOA->OTYPER &= ~(1 << 5); //Push-Pull PA5

}

void EXTI15_10_IRQHandler(void) {
  if (EXTI->PR & (1<<13)&&(aux==0))
  {
	   deb_TIM->CR1 |=(1<<0); //ACTIVAMOS el TIMER 5 que es el deboucing

	   aux=1;

	 // GPIOA->BSRR |= (1<<5); //PROBAMOS SI SE EJECUTA EXTI
  }
  else
    {
	__NOP(); //no realiza ninguna acción
    }
EXTI->PR |= (1 << 13); // Clear interrupt
}

//-------------------INTERRUPCIÓN DEL TIMER5  (deboucing TIMMER)-------------------------//

void TIM5_IRQHandler()
{
	deb_TIM->CR1 &=~ (1<<0); //apagamos el TIMER, no queremos que siga contando ya llegó a 250ms así que lo apagamos

	if(deb_TIM->SR & 1)
	{
		med_TIM->CR1 |=(1<<0); //Activamos el TIMER de medición TIC al pulsar botón y esperar 250ms
		trig_TIM->CR1 |=(1<<0); //envia la PWM para activar el sensor de SONIDO

		GPIOA->BSRR |=(1<<5);//Enciende el LED del PA5 para asegurarnos que el TIMER de medición está actuando

		PUL=1; //nos indicará que hemos activado correctamente el timmer de medición
	}

	deb_TIM->SR &=~(1<<0); //limpiamos flag del evento UPDATE
	aux=0; //para que cuando se ejecute la EXTI vuelva a pasar por el TIMMER
}

///---------------------INTERRUPCIÓN TIMER2 (TIMER DE MEDICIÓN)-----------------------------------///

void TIM2_IRQHandler()
{

	if((med_TIM->SR & 1))   //Se ha desbordado. Se acabó el tiempo llegó a 36ms
	{
	PUL=3;
	//med_TIM->CR1 &= ~(1<<0); //Apaga timer. No se ha producido captura FUERA DE RANGO
	GPIOA->BSRR |= (1<<5)<<16; //Apaga LED
	TON=0;//variable de control
	med_TIM->SR &= ~(1<<0); // Limpio los flags del contador
	}
	if ((med_TIM->SR & (1<<2)))
		{ // Se produce una captura TIC en el <CANAL 2>
		PUL=4;
		numTicks2=med_TIM->CCR2;
		med_TIM->SR &= ~(1<<2);
			/*if (i==2)
			{ //cuando se hallan realizado 2 capturas me imprimirá por pantalla el resultado

				PUL=4;// Actualizo para que el programa principal imprima el calculo por pantalla
				i=0;
			}
			if(captura==1)
			{
				numTicks1=med_TIM->CCR2; 	// Se tomo el número de Tics en el flanco de subida 1r CAPTURA <CANAL 2>
				captura=2; //como ha realizado la primera captura, ahora vamos a realizar la segunda captura
				med_TIM->SR &= ~(1<<2);	 // Limpio los flags del contador <CANAL 2>
				i++;
			} else if(captura==2)
			{
				numTicks2=med_TIM->CCR2; 	// Se tomo el número de Tics flanco de caída 2nd CAPTURA <CANAL 2>
				captura=1; //se realiza la segunda captura, reiniciamos el contador para que vuelva a hacer 1r captura
				med_TIM->SR &= ~(1<<2);
				i++;
			}*/

		}
	//med_TIM->SR &= ~(1<<2);
}
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

	float Tiempo =0;
	//uint32_t conteo=0; //aquí se almacena el resultado del conteo
	float distancia=0;
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
  deboucing_TIMMER();
  trigger_TIM();
  TIMER_MED_TIC();

HAL_UART_Transmit(&huart2,(char *) msg, strlen(msg),100);

  /* USER CODE BEGIN 2 */

//printf(msg);
char msg1[]="Ahora vamos a medir distancias\r\n";
HAL_UART_Transmit(&huart2,(char *) msg1, strlen(msg1),100);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
{
	  if (PUL==1)
	  {
	  PUL=0; // Si pulso lo pongo a 0 para la próxima interrupción
	  sprintf(msg_1, "Timer encendido por usuario\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg_1, strlen(msg_1), HAL_MAX_DELAY);
	  HAL_Delay(1000);
	  }
	  if (PUL==2)
	  {
	  PUL=0; // Si pulso lo pongo a 0 para la próxima interrupción
	  sprintf(msg_1, "Timer apagado por usuario\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg_1, strlen(msg_1), HAL_MAX_DELAY);
	 HAL_Delay(1000);
	  }
	  if (PUL==3)
	  {
	  PUL=0; // Si pulso lo pongo a 0 para la próxima interrupción
	  sprintf(msg_1, "Medida fuera de rango (>36ms)\r\n");
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg_1, strlen(msg_1), HAL_MAX_DELAY);
	 HAL_Delay(1000);
	  }
	  if (PUL==4)
	  {
	  PUL=0; // Si pulso lo pongo a 0 para la próxima interrupción
	  Tiempo =  (numTicks2-numTicks1); // En us PARA ANALIZAR QUÉ ESTÁ OCURRIENDO la distancia me la da la VARIABLE de ABAJO
	  //Tiempo =  numTicks2*0.034/2; //en cm/us
	  distancia= Tiempo; //Distancia en cm
	  sprintf(msg_1, "Distancia en (cm)= %.2f\r\n",   distancia);
	  HAL_UART_Transmit(&huart2, (uint8_t*) msg_1, strlen(msg_1), HAL_MAX_DELAY);
	 HAL_Delay(1000);

	  }

/*
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

*/
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
