/*
 * ESTE PROGRAMA GENERA 2 SEÑALES PWM UNA SE DESEA QUE SEA DE 1HZ Y LA OTRA SE DESEA QUE SEA DE 2HZ SE IMPLEMENTA MEDIANTE
 * EL USO DE BOTONES Y TIMERS PARA PODER CAMBIAR EL PERIODO "T" DEL TIMER QUE SERIA LA FRECUENCIA DE TRABAJO
 *
 */
#include "main.h"

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
uint8_t c=0;
void EXTII_CONFIG()
{
	SYSCFG->EXTICR[3] &=~ (1111<< 4); //reseteamos
	SYSCFG->EXTICR[3] |= (1<< 5); //trabajamos con el puerto C EXTI 13

	EXTI->IMR &=~ (1 << 13); //resetea posicion 13
	EXTI->IMR |=  (1 << 13);// setea y activa la EXTI 13

	EXTI->RTSR &=~(1 << 13);// queremos que el flanco de bajada esté desactivado reseteamos
	EXTI->FTSR &=~(1 << 13);//reseteamos el flanco de bajada
	EXTI->FTSR |=  (1 << 13);// seteamos y activamos el flanco de bajada

	EXTI->PR |= (1 << 13)|(1 << 6)|(1 << 7);// RESETEAMOS EL FLAG CON UN "1"
}
void TIMER_CONFIG()
{
//----------------------CONFIGURACIÓN DE LA BASE DE TIEMPO (BT)-------------------------------//
	TIM2->CNT =0; //almacena el valor de la cuenta de mi timer, en este caso lo definimos en 0 para que empiece en 0
	TIM2->PSC = 8399; //con este prescaler conseguimos un tiempo Tupdate=0,1ms para una señal con periodo de 1 s
	TIM2->ARR = 199; ////50Hz señal a 20ms de periodo


//---------------------CONFIGURACIÓN GENERAL INTERNA DEL TIMER--------------------------------//

	TIM2->CR1 &=~(1<<7); //Reseteamos ARPE
	TIM2->CR1 |=(1<<7); //HABILITAMOS EL ARPE PORQUE DESEAMOS CAMBIAR EL ARR Y FRECUENCIA DE PARPADEO DEL LED
	TIM2->CR1 &=~(11<<8); //Nos aseguramos de que el clock division CKD está deshabilitado ya que no queremos realizar más divisiones
	TIM2->CR1 &=~(11<<5);//RESETEAMOS el CMS para que cuente de manera ascendente o descendente y no haga otras funciones
	TIM2->CR1 &=~(1<<4);//Reseteamos el DIR para que la cuenta sea ascendente y no descendente
	TIM2->CR1 &=~(1<<3);//Reseteamos para que el contador no se pare en el update event OPM
	TIM2->CR1 &=~(1<<2);//Reseteamos para asegurarnos que está a 0
	TIM2->CR1 |=(1<<2);// Seteamos porque nos interesa que solo al hacer match "overflow" salte el evento en el flag
	TIM2->CR1 &=~(1<<1);//Reseteamos para asegurarnos que está a 0

	//-----------CONFIGURACIÓN DE INTERRUPCIONES DEL TIMER------------------------------------//

	TIM2->CR2 =0;//También lo mantenemos a "0" ya que no lo necesitamos utilizar y lo queremos deshabilitado
	TIM2->SMCR=0;//NO UTILIZAMOS ESTE REGISTRO, sirve para concatenar TIMERS y que uno dependa de otro que es el maestro
	TIM2->DIER &=~ (1<<0);//Reseteamos el registro
	TIM2->DIER |= (1<<0); //Setamos para habilitar interrupción del update (match)
	TIM2->EGR &=~(1<<0)|(1<<1)|(1<<2)|(1<<3)|(1<<4)|(1<<5)|(1<<6)|(1<<7); //Reseteamos todos los bits ya que no vamos a generar
	//ningún evento excepto el update


	//-----------------REGISTROS CONFIGURACIÓN MODO TOC-------------------------------------------------//

    TIM2->CCER &=~(1<<1);//En este caso estamos definiendo que el modo captura/comparación se active en un flanco de bajada "low"
    // <<CANAL 1>>
    TIM2->CCER &=~(1<<0);//reseteamos el bit <<CANAL 1>>
    TIM2->CCER |=(1<<0);//seteamos para activar el canal de captura/comparación 1 que es el que estamos utilizando <<CANAL 1>>

    TIM2->CCMR1 &=~(7<<4);//RESET de la posición <<CANAL 1>>
    TIM2->CCMR1 |=(6<<4);//configuramos el modo de trabajo como "PWM modo2" de nuestra señal de salida <<CANAL 1>>

	TIM2->CCMR1 &=~(11<<0);//RESET de la posición CONFIGURAMOS como salida AFR  <<CANAL 1>>

	TIM2->EGR |=(1<<0); // Habilitamos el "update generation" para que reinicie el contador y genere una actualización
		// "update" de los registros ya que vamos a utilizar el flag y necesitamos que se actualice.
	TIM2->SR &=~(1<<0);// limpiamos el flag del status registrer


}
void clock_config()
{
	RCC->AHB1ENR &=~(1<<0); //Reseteamos el reloj del puerto A
	RCC->AHB1ENR |=(1<<0);// Habilitamos el reloj del puerto A

	RCC->AHB1ENR &=~(1<<2); //Reseteamos el reloj del puerto C
	RCC->AHB1ENR |=(1<<2);// Habilitamos el reloj del puerto C

	RCC->APB1ENR &=~(1<<0); //Reseteamos el TIMER2
	RCC->APB1ENR |=(1<<0); //Habilitamos el TIMER2
}
void GPIO_CONFIG()
{
///-----------------CONFIGURACIÓN LED USUARIO ESTADO DEL TIMER--------------------------///

	GPIOA->MODER &=~(11<<5*2); //reseteamos el PIN5
	GPIOA->MODER |=(1<<5*2);//configuramos como salida
	GPIOA->OTYPER &=~(1<<5); //NOS ASEGURAMOS que esté en push-pull la salida del pin 5

///-----------------CONFIGURACIÓN COMO FUNCION ALTERNATIVA---------------------------///

	GPIOA->MODER &=~(11<<0*2); //reseteamos el PIN0
	GPIOA->MODER |=(10<<0*2); //configuramos como función alternativa PIN0
	GPIOA->AFR[0] &=~(1111<<0*4);//reseteamos la posición
	GPIOA->AFR[0] |=(1<<0*4);//registro de 4 bits en el pin 0, necesitamos seleccionar la AFR adecuada
//para ello necesitamos la página 45 del data sheet y ver con qué TIMER y que esté con el PIN0 la AFR que deseamos
//el corchete [0] nos permite seleccinar los primeros 32 bits.

	//----------------------CONFIGURACIÓN DE BOTONES------------------------------//
	GPIOC->MODER &=~(11<<13*2); //reseteamos el PIN13 y lo configuramos como ENTRADA

}
void EXTI15_10_IRQHandler()
{
	if(EXTI->PR & (1<<13))
	{
		if (c==0)
		{
			c++;
			TIM2->CR1 &=~(1<<0);//reseteo de CEN
			TIM2->CR1 |=(1<<0);// CEN este habilita la cuenta en "1" y deshabilita cuenta en "0"
		//SIEMPRE SE PONE AL FINAL DESPUÉS DE CONFIGURAR TODO LAS FUNCIONALIDADES DEL TIMER
		}
		else
		{
			c=0;
			TIM2->CR1 &=~(1<<0);//reseteo de CEN
		}
		EXTI ->PR |=(1<<13); //resetear flag EXTI
	}

}

void TIM2_IRQHandler()
{
	if(TIM2->SR & 1)
	{

		GPIOA->ODR ^=(1<<5); //toggle del pin5 LED USUARIO
		TIM2->SR &=~(1<<0); //limpiar el flag del timer

	}
}
void Delay_ms(uint16_t ms) {
    uint32_t ticks = ms * 21000; // Aproximadamente 1 ms por iteración a 42 MHz
    while (ticks-- > 0);
}


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
	clock_config();
	GPIO_CONFIG();
	EXTII_CONFIG();
	TIMER_CONFIG();
	NVIC->ISER[0] |= (1<<28); //Activar interrupción del TIMER 2
	NVIC_SetPriority(EXTI15_10_IRQn,0); //Activar la prioridad del EXTI de la 10 a la 15
	NVIC_EnableIRQ(EXTI15_10_IRQn);//Activar interrupción del EXTI de la 10 a la 15


  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
     TIM2->CCR1=14; //posición inicial pulso 1,5ms 0º
     Delay_ms(1000);
     TIM2->CCR1=17; //pulso 1,75ms se mueve 45º
     Delay_ms(1000);
     TIM2->CCR1=19;//pulso 2ms 90º
     Delay_ms(1000);
     TIM2->CCR1=19; //pulso 1ms -90º
     Delay_ms(1000);
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
