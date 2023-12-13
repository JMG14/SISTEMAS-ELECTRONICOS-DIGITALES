/*
 * deboucing_TIM.h
 *
 *  Created on: Nov 28, 2023
 *      Author: Jose
 */
//********************************************************************************************//
//********************************************************************************************//

#include <main.h>

void PWM()
{
  //******************************BT TIMER4****************************************//

	TIM3->CNT=0;
	TIM3->PSC=8400-1; // Tu=0,1ms
	TIM3->ARR=10-1; //  1ms     SEÑAL DE 1KHz PERÍODO

  //------------------------CONFIGURACIONES INTERNAS-------------------------------//

	TIM3->CR1=0;//RESET
	TIM3->CR1 |=(1<<2)|(1<<7); //Habilitar UDIS solo evento update y Habilitar ARPE para cambiar CCRy
	TIM3->CR2=0; //Deshabilitado
	TIM3->SMCR=0; //Deshabilitado

  //------------------------CONFIGURACIONES MODO TOC-------------------------------//

	TIM3->CCMR2 &=~(11<<0); //Modo TOC <CANAL 3>
	TIM3->CCMR2 &=~(6<<4); // PWM modo 1 Activo a nivel ALTO <CANAL 3>
	TIM3->CCMR2 |=(6<<4); // PWM modo 1 Activo a nivel ALTO <CANAL 3>

	TIM3->CCER &=~(11<<8); //RESET y dejamos por defecto que CCRy se active a nivel ALTO
	TIM3->CCER |=(1<<8);// HABILITAMOS la PWM <CANAL 3>

	TIM3->CCR3=4;
/*
	TIM3->CCMR2 &=~(11<<8); //Modo TOC <CANAL 4>
	TIM3->CCMR2 &=~(6<<12); // PWM modo 1 Activo a nivel ALTO <CANAL 4>
	TIM3->CCMR2 |=(6<<12); // PWM modo 1 Activo a nivel ALTO <CANAL 4>

	TIM3->CCER &=~(1<<12); //RESET y dejamos por defecto que CCRy se active a nivel ALTO
	//TIM3->CCER |=(1<<12);// HABILITAMOS la PWM  <CANAL 4>

	TIM3->CCR4=50-1; 	// SEÑAL PWM2 con DC=25%
*/
  //*************************INTERRUPCIONES*****************************************//

	TIM3->DIER=0; //RESET
	TIM3->DIER |=(1<<0); //IRQ update ACTIVADA
	TIM3->EGR=0; //RESET
	TIM3->EGR |=(1<<0);// UPDATE de los registros

	//NVIC_EnableIRQ(TIM3_IRQn); //avisamos al NVIC de la interrupción UPDATE

  //***********************LISTOS GO!!*********************************************//

	TIM3->SR&=~(1<<0); //RESET FLAG
	TIM3->CR1 |=(1<<0);
}

