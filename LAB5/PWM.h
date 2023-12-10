/*
 * deboucing_TIM.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Jose
 */
//********************************************************************************************//
//***********************FICHERO DE CONFIGURACIÓN DEL TIMER 5*********************************//
//Este fichero contiene la configuración del TIMER5 que se encarga de eliminar los rebotes
// generados por el elemento mecánico del botón.
//********************************************************************************************//
#include <main.h>
void PWM()
{
  //******************************BT TIMER4****************************************//

	TIM4->CNT=0;
	TIM4->PSC=8400-1; // Tu=0,1ms
	TIM4->ARR=200-1; //  20ms     SEÑAL DE 50Hz PERÍODO

  //------------------------CONFIGURACIONES INTERNAS-------------------------------//

	TIM4->CR1=0;//RESET
	TIM4->CR1 |=(1<<2)|(1<<7); //Habilitar UDIS solo evento update y Habilitar ARPE para cambiar CCRy
	TIM4->CR2=0; //Deshabilitado
	TIM4->SMCR=0; //Deshabilitado

  //------------------------CONFIGURACIONES MODO TOC-------------------------------//

	TIM4->CCMR2 &=~(11<<0); //Modo TOC <CANAL 3>
	TIM4->CCMR2 &=~(6<<4); // PWM modo 1 Activo a nivel ALTO <CANAL 3>
	TIM4->CCMR2 |=(6<<4); // PWM modo 1 Activo a nivel ALTO <CANAL 3>

	TIM4->CCER &=~(11<<8); //RESET y dejamos por defecto que CCRy se active a nivel ALTO
	//TIM4->CCER |=(1<<8);// HABILITAMOS la PWM <CANAL 3>

	TIM4->CCR3=150-1;	// SEÑAL PWM1 con DC=75%

	TIM4->CCMR2 &=~(11<<8); //Modo TOC <CANAL 4>
	TIM4->CCMR2 &=~(6<<12); // PWM modo 1 Activo a nivel ALTO <CANAL 4>
	TIM4->CCMR2 |=(6<<12); // PWM modo 1 Activo a nivel ALTO <CANAL 4>

	TIM4->CCER &=~(1<<12); //RESET y dejamos por defecto que CCRy se active a nivel ALTO
	//TIM4->CCER |=(1<<12);// HABILITAMOS la PWM  <CANAL 4>

	TIM4->CCR4=50-1; 	// SEÑAL PWM2 con DC=25%

  //*************************INTERRUPCIONES*****************************************//

	TIM4->DIER=0; //RESET
	TIM4->DIER |=(1<<0); //IRQ update ACTIVADA
	TIM4->EGR=0; //RESET
	TIM4->EGR |=(1<<0);// UPDATE de los registros

	NVIC_EnableIRQ(TIM4_IRQn); //avisamos al NVIC de la interrupción UPDATE

  //***********************LISTOS GO!!*********************************************//

	TIM4->SR&=~(1<<0); //RESET FLAG
	//TIM4->CR1 |=(1<<0);
}

