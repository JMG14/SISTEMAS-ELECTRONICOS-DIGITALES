
//********************************************************************************************//
//***********************FICHERO DE CONFIGURACIÓN DEL TIMER 1*********************************//
//Este ficheor contiene la configuración del TIMER1 que se encarga de gestionar CUANDO
// se imprime por pantalla el promedio de las medidas
//********************************************************************************************//
#include <main.h>
void stream_TIM()
{
  //******************************BT TIMER1****************************************//

	TIM1->ARR=10000-1; // 1 segundo
	TIM1->PSC=8400-1; // Tu=0,1ms
	TIM1->CNT=0;

  //------------------------CONFIGURACIONES INTERNAS-------------------------------//

	TIM1->CR1=0;//RESET
	TIM1->CR1 |=(1<<2); //Habilitar UDIS solo evento update
	TIM1->CR2=0; //Deshabilitado
	TIM1->SMCR=0; //Deshabilitado

  //*************************INTERRUPCIONES*****************************************//

	TIM1->DIER=0; //RESET
	TIM1->DIER |=(1<<0); //IRQ update ACTIVADA
	TIM1->EGR=0; //RESET
	TIM1->EGR |=(1<<0);// UPDATE de los registros

	NVIC_EnableIRQ(TIM1_UP_TIM10_IRQn); //avisamos al NVIC de la interrupción UPDATE

  //***********************LISTOS GO!!*********************************************//

	TIM1->SR=0; //RESET FLAG
	//TIM1->CR1 |=(1<<0); //ESTE CÓDIGO ACTIVA EL TIMER (LO ENCIENDE)

}
