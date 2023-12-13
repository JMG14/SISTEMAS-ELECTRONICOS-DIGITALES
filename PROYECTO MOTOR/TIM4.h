
//********************************************************************************************//
//***********************FICHERO DE CONFIGURACIÓN DEL TIMER 1*********************************//
//Este ficheor contiene la configuración del TIMER1 que se encarga de gestionar CUANDO
// se imprime por pantalla el promedio de las medidas
//********************************************************************************************//
#include <main.h>
void deb_TIM()
{
  //******************************BT TIMER4****************************************//

	TIM4->ARR=2500-1; // 250ms
	TIM4->PSC=8400-1; // Tu=0,1ms
	TIM4->CNT=0;

  //------------------------CONFIGURACIONES INTERNAS-------------------------------//

	TIM4->CR1=0;//RESET
	TIM4->CR1 |=(1<<2); //Habilitar UDIS solo evento update
	TIM4->CR2=0; //Deshabilitado
	TIM4->SMCR=0; //Deshabilitado

  //*************************INTERRUPCIONES*****************************************//

	TIM4->DIER=0; //RESET
	TIM4->DIER |=(1<<0); //IRQ update ACTIVADA
	TIM4->EGR=0; //RESET
	TIM4->EGR |=(1<<0);// UPDATE de los registros

	NVIC_EnableIRQ(TIM4_IRQn); //avisamos al NVIC de la interrupción UPDATE

  //***********************LISTOS GO!!*********************************************//

	TIM4->SR=0; //RESET FLAG


}
