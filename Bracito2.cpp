#include <stm32f7xx.h>
#include <stdio.h>
#include <cmath>
#include <math.h>  // Para usar sin() y fmod
#include <string.h>
#include "LCD.h"
#define angles2use 5
#define prom_number 50
#define prom_difference 2.5


uint8_t rx_master_slave = 2; //1 si es para maestro y 0 para esclavo

//USART ESCLAVO
uint8_t usart6_index = 0;
uint8_t usart6_rx[angles2use+2]; 

volatile uint32_t tm_ms = 0; 
volatile int usart_ind = 0;
char txt[50];
char rx_txt[20];
float dt = 0;
uint8_t angle_array[angles2use];
uint8_t measured_angle_array[angles2use]; //angulo medido
float converted_angle_array[angles2use]; //angulo medido
uint8_t adc_sc = 1, adc_seq_index = 0;
int adc = 0;
int send_lcd = 0, lcd_ready = 0;
char lcd_txt[60];
int ang2_3 = 0;
float theta1 = 0, theta2 = 0, theta3 = 0, theta4 = 0, theta5 = 0, theta_garra = 0;
//float angle_eq[angles2use][2] = {{0.8035, 4.292},
//                                 {1.0553, -42.729},
//                                 {-0.9905, 219.98},
//                                 {0.782, -19.739},
//                                 {1.0288, - 49.121}};

float angle_eq[angles2use][2] = {{0.8047, 4.215},
                                 {1.0469, -41.521}, 
                                 {0.9911, - 37.619},
                                 {0.782, -19.739},
                                 {0.9911, -37.619}};


float prom_indexes[angles2use];
float valores_adc[angles2use][prom_number] = {0.0};
int prom_index[angles2use];   // índice circular (última posición escrita)
float prom_count[angles2use];   // cuántos elementos válidos hay (<= prom_number)
float prom_sum[angles2use];   // sólo para la versión optimizada (puede inicializarse a 0)


void SysTick_Init(void);                   //funciones para los delays
void SysTick_Wait(uint32_t n);
void SysTick_Wait1ms(uint32_t delay);
void SysTick_Wait1us(uint32_t delay);
void Ms_Counter();
void Delay_ms(uint16_t ms);

void RCC_SETUP();
void USART_SETUP();
void GPIO_SETUP();
void EXTI_SETUP();
void TIM_SETUP();
void ADC_SETUP();
void USART3_SendChar(int value);	
void Serial_buffer(char *send_txt);
void Serial();
void Gyr_Calib();
void Usart_Funct();

void DMA_Setup();
void DMA_Usart(char *send_txt);
void DMA_Usart6(uint8_t *send_txt);
uint16_t string_len(char *send_txt);
float map(int value, int fromLow, int fromHigh, int toLow, int toHigh);
void update_promedio_fast(int idx, float sample);
void LCD_txt_buffer(char *txt);



int main(){  
    SysTick_Init(); //Configurado con interrupciones para que contara cada 1ms
    NVIC_EnableIRQ(SysTick_IRQn);
    RCC_SETUP();  
    GPIO_SETUP(); 
    USART_SETUP();
    TIM_SETUP();
    DMA_Setup();
    ADC_SETUP();
    Ms_Counter(); //Tener cuidado con poner la interruopci?n m?s corta que el proceso de obtenci?n de datos
    Delay_ms(5);
    NVIC_EnableIRQ(TIM5_IRQn);
//    TIM2->DIER |= 0x1; //Habilita Interrupcion
//    NVIC_EnableIRQ(TIM2_IRQn);
    memset(angle_array, 90, sizeof(angle_array)); //angulo deseado
    memset(measured_angle_array, 0, sizeof(measured_angle_array));
    txt[0] = 255;
    txt[6] = 200;
    adc_sc = 1;
    ADC1->CR2 |= (1<<30);
    LCD_Init();
	lcd_ready = 1;
	
	memset(prom_index, 0, sizeof(prom_index));
    memset(prom_count, 0, sizeof(prom_count));
    memset(prom_sum, 0, sizeof(prom_sum));
    //memset(valores_adc, 0, sizeof(valores_adc));
    memset(converted_angle_array, 0, sizeof(converted_angle_array));
    Delay_ms(100);

    while(1){
        TIM5->CCR1 = (int) map(angle_array[0], 0, 180, 500, 2500); //PA0    
        //TIM5->CCR3 = pwm3; //PA2
			
        TIM5->CCR4 = (int) map(angle_array[1], 0, 180, 500, 2500); //PA3
        TIM4->CCR1 = (int) map(angle_array[2], 0, 180, 500, 2500); //PD12
        TIM4->CCR2 = (int) map(angle_array[3], 0, 180, 500, 2340); //PD13
        TIM4->CCR3 = (int) map(angle_array[4], 0, 180, 500, 2500); //PD14
        //TIM4->CCR4= map(angle_array[5], 0, 180, 500, 2500); //PD15
        TIM4->CCR4 = (int) map(theta_garra, 0, 180, 500, 2500); //PD15
        if(send_lcd){
            for (int i = 0; i < angles2use; i++)
            {
                //converted_angle_array[i] = measured_angle_array[i]*angle_eq[i][0] + angle_eq[i][1]; 
                float ang_medido = measured_angle_array[i] * angle_eq[i][0] + angle_eq[i][1];
                update_promedio_fast(i, ang_medido);
            }
            float ang2_3_lcd[2] = {converted_angle_array[1]-5, converted_angle_array[2]+4};
            LCD_Fila1();
            //sprintf(lcd_txt, "A1%05.1f|%05.1f|%05.1f\n", converted_angle_array[0], converted_angle_array[1], -1*(180-(converted_angle_array[2])-90));
            sprintf(lcd_txt, "1:%05.1f  %d:%05.1f\n", converted_angle_array[0], ang2_3+2, ang2_3_lcd[ang2_3]);
            LCD_txt_buffer(lcd_txt);
            LCD_Fila2();
            sprintf(lcd_txt, "4:%05.1f  5:%05.1f  \n", converted_angle_array[3]-13, converted_angle_array[4]-3, fabs(int(((180-(converted_angle_array[2])))*10)%10)-7);
            LCD_txt_buffer(lcd_txt);
            
            send_lcd = 0;
        }
    }
    
}

void update_promedio_fast(int idx, float sample) {
    if (prom_count[idx] == 0) {
        prom_index[idx] = 0;
        prom_count[idx] = 1;
        valores_adc[idx][0] = sample;
        prom_sum[idx] = sample;
        converted_angle_array[idx] = sample;
        return;
    }

    if (fabsf(sample - converted_angle_array[idx]) > prom_difference) {
        prom_index[idx] = 0;
        prom_count[idx] = 1;
        valores_adc[idx][0] = sample;
        prom_sum[idx] = sample;
        converted_angle_array[idx] = sample;
        return;
    }

    // se calcula el siguiente índice a escribir
    prom_index[idx] = (prom_index[idx] + 1) % prom_number;

    if (prom_count[idx] < prom_number) {
        // todavía no lleno el buffer
        valores_adc[idx][prom_index[idx]] = sample;
        prom_sum[idx] += sample;
        prom_count[idx]++;
    } else {
        // buffer ya lleno: restar valor viejo y añadir nuevo
        prom_sum[idx] += sample - valores_adc[idx][prom_index[idx]];
        valores_adc[idx][prom_index[idx]] = sample;
    }
    converted_angle_array[idx] = prom_sum[idx] / prom_count[idx];
}


void LCD_txt_buffer(char *txt){
    while (*txt != '\n')
    {
        LCD_Write(*txt);
        txt++;
        Delay_ms(1);
    }
    
}

void Usart_Funct(){
    Serial_buffer(txt);
}

void Serial(){
    sprintf(txt, "Hola\n");
    USART3_SendChar(txt[usart_ind]);
}
void Serial_buffer(char *send_txt){
    while(*send_txt != '\n'){
        USART3_SendChar(*send_txt);
        send_txt++;
        Delay_ms(10);
        //SysTick_Wait1ms(5);
    }
    USART3_SendChar(10);
}

void RCC_SETUP(){
    RCC->AHB1ENR |= 0xF;
    RCC->APB1ENR |= (1<<21) | (1<<18); //I2C EN y USART3 EN
    RCC->APB2ENR |= (1<<14)|(7<<8)|(1<<5);//EXTI EN HABILITAR ADC1,2,3 y USART6
    RCC->APB1ENR |= (1<<3)|(1<<2); //Habilita Tim 5 y 4
}

void GPIO_SETUP(){
    GPIOA->MODER |=(2<<0)|(2<<2)|(2<<4)|(2<<6)|(0xFF<<8); //A0-3 es alternante pwm; 4-7 adc
    GPIOA->AFR[0]|= 0x2222; //Funci�n alternante para pwm tim5
    
    GPIOC->MODER |= (0xF<<4)|(0xA000); // PC2-3 ADC y PC6-7 USART6
    GPIOC->AFR[0] |= (0x88<<24); //USART6
    
    GPIOD->MODER |= (0xAA<<24); //PD12-15 como alternante para pwm tim4
    GPIOD->AFR[1]|= (0x2222<<16); //Funci�n alternante para pwm tim4
    GPIOB->MODER |= (1<<0)|(1<<2)|(1<<28)|(1<<14)|(0xA<<16)|(3<<2);// lEDs e I2C PB8(SCL) Y PB9(SDA) como alternante, PB1 ADC
    GPIOD->MODER |= 0xA0000; //Para USART3 PD8(TX) Y PD9(RX) como alternante
    GPIOB->OTYPER |= 0x300; //PB8 y 9 como open drain
    GPIOB->OSPEEDR |= 0xF0000; //PB8 y 9 como high speed
    GPIOB->PUPDR |= 0x50000;
    
    GPIOB->AFR[1] |= 0x44;
    GPIOD->AFR[1] |= 0x77; //Su respectiva funci?n alternante, se encuentra en mapa AF
}

void ADC_SETUP(){   
    //ADC1 4 -> 5 -> 6 -> 7 -> 12 -> 13 (PA4 -> PA5 -> PA6 -> PB1 -> PC2 -> PC3)

    ADC->CCR|=(1<<16); //Prescaler de 2
    
    //de momento no se usar� el DMA ni modo cont�nuo, solo scan
    //ADC1->CR2 |= 0x3; // Activa el Conversor A/D y la conversion continua
    ADC1->CR2 |= (1<<10); // Activa el Conversor A/D y EOCS para que avise al final de cada secuencia
    ADC1->CR1 |= (1<<26)|(2<<24)|(0<<5)|(1<<8); //8 bit de resolicion, EOCIE, Scan_Mode
    ADC1->SMPR1 |= (3<<12)|(3<<15)|(3<<18)|(3<<21); // se ponen 15 ciclos 
    ADC1->SMPR2 |= (3<<6)|(3<<9); //  se ponen 15 ciclos
    ADC1->SQR1 |= ((angles2use-1)<<20); //#de conversiones
	
    ADC1->SQR3 = (4 << 0) |  // primer canal = 4
                (5 << 5)  |  // segundo canal = 5
                (6 << 10) |  // tercer canal = 6
                (9 << 15) |  // cuarto canal = 9
                (12 << 20)|  // quinto canal = 12
                (13 << 25);  // sexto canal = 13

    ADC1->CR2 |= (3<<8)|(1<<1); //DMA disable y DMA mode en - CONT
    ADC1->CR2 |= 1; //ADC ON

//    ADC2->CR2 |= 0x3; // Activa el Conversor A/D y la conversion continua
//    ADC2->CR1 |= (3<<24)|(1<<5)|(1<<8); //8 bit de resolicion, EOCIE, Scan_Mode
//    ADC2->SMPR1 |= (1<<6)|(1<<9); // PC2 tiene canal 12 Y PC3 tiene canal 13; se ponen 15 ciclos
//    ADC2->SQR3= (12<<0)|(13<<4);  
//    
//    ADC3->CR2 |= 0x3; // Activa el Conversor A/D y la conversion continua
//    ADC3->CR1 |= (3<<24)|(1<<5)|(1<<8); //8 bit de resolicion, EOCIE, Scan_Mode
//    ADC3->SMPR1|=(1<<15); // PF5 tiene canal 9; se ponen 15 ciclos 
//    ADC3->SMPR2|=(1<<27); // PF3 tiene canal 9; se ponen 15 ciclos 
//    ADC3->SQR3= (9<<0)|(15<<4); 

    NVIC_EnableIRQ(ADC_IRQn); //Por si queremos la interrupci?n que igual trabaja con el ARR

}

void USART_SETUP(){
    USART3->BRR |= 0x113; //BAUD RATE 115200 -0x113
    //USART3->CR1 |= ((0xD) | (0<<15));
    USART3->CR1 |= ((0x2D) | (1<<15)); //Pal oversampling que va al baud rate
    //USART EN, Reciever y Transmiter EN, RX INTERRUPT EN
    USART3->CR3 |= (1<<7); //EN DMA para transmitir
    NVIC_EnableIRQ(USART3_IRQn); //nombre en tabla del NVIC

    USART6->BRR |= 0x8B;
    USART6->CR1 |=((0x2C) | (0<<15));//ovsersampling by 8
    USART6->CR3 |= (1<<7); //EN DMA para transmitir
    USART6->CR1 |= 1; //usart enable ->poner de últimas
    
    NVIC_EnableIRQ(USART6_IRQn);
}

void USART3_SendChar(int value){
    USART3->TDR = value; //Se escribe lo que se va a enviar
    while(!(USART3->ISR & USART_ISR_TXE)); //A la posici?n 7
    //No sale de la funci?n hasta que indica que envi? el dato
}

void DMA_Usart(char *send_txt){
    DMA1_Stream3->CR &= ~1;
    if(DMA1->LISR&(1<<27)){
        DMA1->LIFCR |= (1<<27);	
    }
    uint16_t len = string_len(send_txt);
    DMA1_Stream3->M0AR = (uint32_t)send_txt;
    DMA1_Stream3->NDTR = len;//angles2use+2;
    DMA1_Stream3->CR |= 1;
}

void DMA_Usart6(uint8_t *send_txt){
    DMA2_Stream7->CR &= ~1;
    if(DMA2->HISR&(1<<27)){
        DMA2->HIFCR |= (1<<27);	
    }
    uint16_t len = sizeof(send_txt);
    DMA2_Stream7->M0AR = (uint32_t)send_txt;
    DMA2_Stream7->NDTR = angles2use+2;
    DMA2_Stream7->CR |= 1;
}

uint16_t string_len(char *send_txt){
    uint16_t len = 0;
    while(*send_txt != '\n'){
        len++;
        send_txt++;
    }
    return len+1;
}

void DMA_Setup(){
    DMA1_Stream3->CR &= ~1;
    RCC->AHB1ENR |= (1<<21);
    DMA1_Stream3->PAR = (uint32_t)&USART3->TDR;
     
    //Definir longitud
    //la idea es que la longitud se defina desde el inicio pero va a estar cambiando

    DMA1_Stream3->CR |= (4<<25)|(1<<6); //canal, Mem2Per
    DMA1_Stream3->CR |= (0x00<<11); //definir 8 bits para cada palabra pues el usart trabaja de a bytes
    DMA1_Stream3->CR |= (1<<10); //Poner incremento de la memoria del puntero

    DMA2_Stream0->CR &= ~1;
    RCC->AHB1ENR |= (1<<22);
    DMA2_Stream0->CR |= (1<<8)|(2<<16); //Modo circular
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR; //puntero al periferico
    DMA2_Stream0->M0AR = (uint32_t)measured_angle_array;
    DMA2_Stream0->NDTR = angles2use; //# de transacciones
     
    //Definir longitud
    //la idea es que la longitud se defina desde el inicio pero va a estar cambiando

    DMA2_Stream0->CR |= (0<<25)|(0<<6); //canal, Per2Mem
    DMA2_Stream0->CR |= (0x00<<11); //definir 8 bits para cada palabra pues el usart trabaja de a bytes
    DMA2_Stream0->CR |= (1<<10); //Poner incremento de la memoria del puntero
    DMA2_Stream0->CR |= 1;
    //NVIC_EnableIRQ(DMA2_Stream0_IRQHandler);


    //DMA del USART2
    DMA2_Stream7->CR &= ~1;

    DMA2_Stream7->PAR = (uint32_t)&USART6->TDR;
    DMA2_Stream7->CR |= (5<<25)|(1<<6); //canal, Mem2Per
    DMA2_Stream7->CR |= (0x00<<11); //definir 8 bits para cada palabra pues el usart trabaja de a bytes
    DMA2_Stream7->CR |= (1<<10); //Poner incremento de la memoria del puntero

}

void SysTick_Init(void) {
    SysTick->LOAD = 0xFFFFFF; //Cargar el valor en el que se da el overload
    SysTick->CTRL = 0x00000005; //Habilita systick, interrupci?n (bit 1) desactivada, y la fuente de reloj
}
void SysTick_Wait(uint32_t n) {
    SysTick->LOAD = n - 1;
    SysTick->VAL = 0;
    SysTick->CTRL &= ~(1<<1); //Desahibilitamos la interrupci?n
    while ((SysTick->CTRL & 0x00010000) == 0); //Se ponga en alto la flag del overload
}
void SysTick_Wait1ms(uint32_t delay) {
    for (uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16000);
    }
}
void SysTick_Wait1us(uint32_t delay) {
    for (uint32_t i = 0; i < delay; i++) {
        SysTick_Wait(16);
    }
}
void Ms_Counter(){
    dt = 0;
    SysTick->LOAD = 63999;
    SysTick->VAL = 0;
    SysTick->CTRL |= (1<<1); //Habilitamos la interrupci?n
    //while ((SysTick->CTRL & 0x00010000) == 0);
}

void TIM_SETUP(){

    RCC->APB1ENR|=0x3B; //Prender Timer 6, 5 y 2
    TIM6->ARR = 2000-1; //ARR
    TIM6->PSC = 8-1; //PSC Para tener 1 kHz
    TIM6->EGR|=0x1; //Reinicio del conteo
    TIM6->CR1 |= 0x1; //Habilitar conteo

    TIM3->ARR = 50000-1; //ARR
    TIM3->PSC = 160-1; //PSC Para tener 50KHz
    TIM3->EGR|=0x1; //Reinicio del conteo
    TIM3->CR1 |= 0x1; //Habilitar conteo
    TIM3->DIER |= 0x1; // habilita interruupci?n

    TIM2->ARR = 20000-1; //ARR
    TIM2->PSC = 1600-1; //PSC Para tener 50KHz
    TIM2->EGR|=0x1; //Reinicio del conteo
    TIM2->CR1 |= 0x1; //Habilitar conteo
    TIM2->DIER |= 0x1; // habilita interruupci?n

    TIM4->ARR=20000-1; //ARR
    TIM4->PSC=16-1; //PSC Para tener 50 Hz
    TIM4->DIER|=0X1; //Habilitar interrupci?n
    TIM4->EGR|=0X1; //Reinicio del conteo
    TIM4->CCMR1|=0X6060; //Modo PWM Tim4  CH1 y 2 - Como salida 00 en CC1S
    TIM4->CCMR2|=0X6060; //Modo PWM Tim4  CH3 y 4 - Como salida 00 en CC1S
    TIM4->CCER|=(1<<0) | (1<<4)|(1<<8) | (1<<12);  //Habilitar salida de comparaci?n y captura 
    TIM4->CR1|=0X1; //Habilitar conteo

    TIM5->ARR=20000-1; //ARR
    TIM5->PSC=16-1; //PSC Para tener 50 Hz
    TIM5->DIER|=0X1; //Habilitar interrupci?n
    TIM5->EGR|=0X1; //Reinicio del conteo
    TIM5->CCMR1|=0X6060; //Modo PWM Tim5  CH1 y 2 - Como salida 00 en CC1S
    TIM5->CCMR2|=0X6060; //Modo PWM Tim5  CH3 y 4 - Como salida 00 en CC1S
    TIM5->CCER|=(1<<0) | (1<<4)|(1<<8) | (1<<12);  //Habilitar salida de comparaci?n y captura 
    TIM5->CR1|=0X1; //Habilitar conteo

    TIM7->PSC = 160-1;  // Para tener 10kHz
    TIM7->ARR = 10000-1; //ARR Para tener 10Hz
    TIM7->DIER |= 0x1; //Habilita Interrupcion
    TIM7->EGR |= 0x1; //Reinicio del conteo
    TIM7->CR1|=0X1; //Habilitar conteo

    NVIC_EnableIRQ(TIM7_IRQn);
	NVIC_EnableIRQ(TIM5_IRQn);
    NVIC_EnableIRQ(TIM4_IRQn);
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_EnableIRQ(TIM3_IRQn);
	
	
	
	while (!(TIM6->SR & (1<<0)));  // UIF: Update interrupt flag..  This bit is set by hardware when the registers are updated
}

void Delay_ms(uint16_t ms){
    TIM6->CNT = 0;
    for(int i = 0; i < ms; i++){ 
        while(!(TIM6->SR & (1<<0)));
        TIM6->SR &=~(1UL<<0);
    }
    
}

float map(int value, int fromLow, int fromHigh, int toLow, int toHigh) {
    if (value > fromHigh || value < fromLow)
        return value;
    
    return (value - fromLow) * (toHigh - toLow) / (fromHigh - fromLow) + toLow;
}

extern "C"{
    void USART3_IRQHandler(void){

		while (USART3->ISR & USART_ISR_RXNE){
            uint8_t rx = (uint8_t)(USART3->RDR & 0xFF);
			if (rx > 180){
                    if (rx == 255){
                        adc_seq_index = 0;
                        rx_master_slave = 1;
                    }else if(rx == 254){
                        usart6_index = 1;
                        usart6_rx[0] = 254;
                        rx_master_slave = 0;
                    }else if(rx_master_slave == 0 && ((rx == 200)||(rx == 250))){
                        usart6_rx[angles2use + 1] = rx;
                        DMA_Usart6(usart6_rx);
                    }else if (rx == 200){
                            theta_garra = 180; //abierta
                        }else if (rx == 250){
                            theta_garra = 0; //cerrada
                        }
                    
                    
                    
                }else if(rx_master_slave == 1){
                    if(rx <=180 && adc_seq_index < angles2use){
                        angle_array[adc_seq_index] = rx;
                        adc_seq_index++;
                    }
                }else if(rx_master_slave == 0){
                    if(rx <=180 && usart6_index <= angles2use){
                        usart6_rx[usart6_index] = rx;
                        usart6_index++;
                    }
                    if(usart6_index >= angles2use){
                        //send_dma
                    }
                }
            
            
//						theta1 = angle_array[0]+angle_array[1]/100.0;
//						theta2 = angle_array[2]+angle_array[3]/100.0;
//						theta3 = angle_array[4]+angle_array[5]/100.0;
//						theta4 = angle_array[6]+angle_array[7]/100.0;
//						theta5 = angle_array[8]+angle_array[9]/100.0;
						
							
						
            
//            char rx = (char)(USART3->RDR & 0xFF);
//            switch(rx) //rx is the variable to compare
//            {
//            case 'S': //Task for char 'M';
//                GPIOB->BSRR |= (1<<0);
//                break;
//            case 'N': //Task for char 'N';	
//                GPIOB->BSRR |= (1<<7);
//                break;
//            default:
//                GPIOB->ODR ^= (1<<14);
//                break;
//            }
			
		}	
		 	
	}

    void USART6_IRQHandler(void){

		while (USART6->ISR & USART_ISR_RXNE){
            uint8_t rx = (uint8_t)(USART6->RDR & 0xFF);
            if (USART6->ISR & USART_ISR_ORE) {
                volatile uint8_t dummy = USART6->RDR;
                (void)dummy;
                USART6->ICR |= USART_ICR_ORECF;
            }else if (rx > 180){
                if (rx == 254){
                    adc_seq_index = 0;
                }else if (rx == 200){
                    theta_garra = 180; //abierta
                }else if (rx == 250){
                    theta_garra = 0; //cerrada
                }
                
            }else if(rx <=180 && adc_seq_index < angles2use){
                    angle_array[adc_seq_index] = rx;
                    adc_seq_index++;
            }
        
//						theta1 = angle_array[0]+angle_array[1]/100.0;
//						theta2 = angle_array[2]+angle_array[3]/100.0;
//						theta3 = angle_array[4]+angle_array[5]/100.0;
//						theta4 = angle_array[6]+angle_array[7]/100.0;
//						theta5 = angle_array[8]+angle_array[9]/100.0;
						
							
					
			
		}	
		 	
	}

    void TIM2_IRQHandler(void){
        TIM2->SR &=~(1UL<<0);
				ang2_3^=1;			
        
    }
		void TIM3_IRQHandler(void){
        TIM3->SR &=~(1UL<<0);
        send_lcd = lcd_ready;
    }

	void TIM4_IRQHandler(void){
        TIM4->SR &=~(1UL<<0);
        GPIOB->ODR^=(3<<0); //Bascula cada 500Hz


        //limpiar override del usart por si acaso
        uint32_t isr = USART6->ISR;
        if (isr & (USART_ISR_ORE | USART_ISR_FE)) {
            // Leer RDR para limpiar ORE/FE
            volatile uint32_t temp = USART6->RDR;
            (void)temp; // evitar warnings

            // Rehabilitar la recepción si se detuvo
            USART6->ICR = USART_ICR_ORECF | USART_ICR_FECF;

            // (opcional) reiniciar alguna variable de control
            // o indicar que hubo un reinicio del puerto
        }

    }

    void TIM5_IRQHandler(void){
        TIM5->SR &=~(1UL<<0);
        GPIOB->ODR^=(3<<7); //Bascula cada 500Hz
    }

    void SysTick_Handler(void) {
        dt = 0.01;
	}

    void TIM7_IRQHandler(void){
        TIM7->SR &=~(1UL<<0);
        sprintf(txt, "%d,%d,%d,%d,%d\n", measured_angle_array[0], measured_angle_array[1], measured_angle_array[2], measured_angle_array[3], measured_angle_array[4]);
        DMA_Usart(txt);
			//DMA_Usart6(usart6_rx);
    }

    void ADC_IRQHandler(void){
        uint32_t sr = ADC1->SR; // leer SR limpia flags cuando se combine con lectura de DR

        if (sr & (1<<5)) {
            ADC1->DR &= ~(1<<5);

            DMA2_Stream0->CR &= ~1;
            DMA2_Stream0->M0AR = (uint32_t)measured_angle_array;
            DMA2_Stream0->NDTR = angles2use; //# de transacciones
            DMA2_Stream0->CR |= 1;
            ADC1->CR2 |= (1<<30); //start convertion
            
        }


        //adc = ADC1->DR;
	


    }
    void DMA2_Stream0_IRQHandler(void) {
    if (DMA2->LISR & DMA_LISR_TCIF0) {
        DMA2->LIFCR = DMA_LIFCR_CTCIF0; // Limpiar flag
        // Aqu� ya tienes los 6 valores en adc_buffer[]       
    }
}    
}