
#include <main.h>

void TIMER_MED_TIC()
{
 //---------CONFIGURACIÓN DE <BT> DEL TIMER 2---------------------------//

	TIM2->ARR = 35999; //el período de mi señal va a ser 36ms
/*Esto lo hacemos porque si el objeto está demasiado cerca o demasiado lejos el ancho de pulso que nos entrega el sensor
 * es de unos 36ms indicando que la medida está FUERA DE RANGO
 */	TIM2->PSC =83; //tengo una resolución Tu=0.001ms  (1us)
	TIM2->CNT =0; //la cuenta empieza en 0

//------------------CONFIGURACIÓN INTERNA DEL TIMER 2--------------------------------------//

	TIM2->CR1 =0;//Desactivamos TODAS las fnciones de CR1
	TIM2->CR2=0;//Desactivamos TODAS las funciones de CR2
	TIM2->SMCR =0; //DESACTIVADO el modo esclavo

//---------CONFIGURACIÓN MODO TIC PARA CAPTURAR LA SEÑAL DE TIEMPO---------------------------//

	TIM2->CCMR1 =0; //Reset de TODAS las configuraciones
	TIM2->CCMR1 |=(01<<8); //configuración <CANAL 2> como INPUT (TIC) vamos a MEDIR señal tiempo de "ECHO"
	TIM2->CCER=0; //RESET de TODAS las posiciones
	TIM2->CCER |=(1<<4); //ACTIVAR el modo CAPTURA (TIC) del <CANAL 2>
    TIM2->CCER &=~(11<<5); //Activo por flanco de SUBIDA
	//No queremos utilizar ningún FILTRO ni tampoco el modo PSC que emplea el propio registro CCER
    //NOTA--> EN VEZ DE PONER 2 LINEAS de código si pones <TIM2->CCER |=(7<<4);> también funcionaría

    TIM2->CCMR1 |=(01<<0);//configuración <CANAL 1> como INPUT (TIC)
    TIM2->CCER |=(7<<0); //ACTIVAR el modo CAPTURA (TIC) del <CANAL 1>
   // TIM2->CCER &=~(11<<1); //Activo por flanco de BAJADA

//----------------ACTIVACIÓN FINAL DEL TIMER-------------------------//

	TIM2->EGR = 0;
	TIM2->EGR |=(1<<0);//actualizacion de eventos GENERAMOS un UPDATE antes de activar la interrupción para que no genere
	//ninguna señal antes de tiempo
	TIM2->SR =0; //limpiar el flag
	TIM2->DIER =0;//RESET
	TIM2->DIER |=((1<<0)|(1<<2)|(1<<1));// Se genera una interrupción al ocurrir una CAPTURA o un UPDATE <CANAL 2>

	TIM2->CR1 |=(1<<0);
	NVIC_EnableIRQ(TIM2_IRQn); //habilitamos interrupción del TIMER2

}
