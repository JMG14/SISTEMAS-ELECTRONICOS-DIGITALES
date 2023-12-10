/*
 * TIM3_Trigger.h
 *
 *  Created on: Dec 3, 2023
 *      Author: Jose
 */
#include <main.h>

void trigger_TIM() //ESTE TIMER ME VA A DISPARAR LA CONVERSIÓN DE MI ADC
{
  //******************************BT TIMER3****************************************//

	TIM3->ARR=2500-1; // 250ms
	TIM3->PSC=8400-1; // Tu=0,1ms
	TIM3->CNT=0;

  //------------------------CONFIGURACIONES INTERNAS-------------------------------//

	TIM3->CR1=0;//RESET
	TIM3->CR1 |=(1<<2); //Habilitar UDIS solo evento update
	TIM3->CR2=0; //Deshabilitado
	TIM3->SMCR=0; //Deshabilitado

  //*************************INTERRUPCIONES*****************************************//

	TIM3->DIER=0; //RESET
	TIM3->DIER |=(1<<0); //IRQ update ACTIVADA
	TIM3->EGR=0; //RESET
	TIM3->EGR |=(1<<0);// UPDATE de los registros

	NVIC_EnableIRQ(TIM3_IRQn); //avisamos al NVIC de la interrupción UPDATE

  //***********************LISTOS GO!!*********************************************//

	TIM3->SR=0; //RESET FLAG
	//TIM3->CR1 |=(1<<0); //ESTE CÓDIGO ACTIVA EL TIMER (LO ENCIENDE)

}


