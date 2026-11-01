#include <stm32f7xx.h>
#include <stdio.h>

void Enable();
void LCD_Init();
void LCD_Write(int word);
void LCD_Clear();
void LCD_Fila1();
void LCD_Fila2();

void Enable(){
	GPIOG-> ODR |= 1;
	for(int i = 0; i < 1600; i++); //Configurar de acuerdo al tiempo necesario
	GPIOG-> ODR &= ~1;
	GPIOD-> ODR &= ~0xFF;
}

void LCD_Write(int word){
	GPIOG->ODR |= 0x2;
    GPIOD-> ODR = word;
    Enable();
}

void LCD_Clear(){
    GPIOG-> ODR &= ~2;
    GPIOD-> ODR &= ~0xFF;
    GPIOD-> ODR |= 1;
    Enable();
}

void LCD_Fila1(){
    GPIOG-> ODR &= ~2;
    GPIOD-> ODR &= ~0xFF;
    GPIOD-> ODR |= 0x80;
    Enable();
}

void LCD_Fila2(){
    GPIOG-> ODR &= ~2;
    GPIOD-> ODR &= ~0xFF;
    GPIOD-> ODR |= 0xC0;
    Enable();
}

void posicionCursor(uint16_t fila, uint16_t columna){
    uint16_t comando = (fila == 2) ? 0xC0:0x80;
    comando |= (columna&0xF);
    GPIOG-> ODR &= ~2;
    GPIOD-> ODR &= ~0xFF;
    GPIOD->ODR |= comando;
    Enable();
}

void LCD_Init(){
    RCC->AHB1ENR |= (0x48); //PUERTO D y G
    GPIOD->MODER |= 0x5555;
    GPIOG->MODER |= 0x5;

    GPIOD->ODR|=(0x30);//Configurar 8 bits 
    Enable();	
	for(int i = 0; i<1600; i++);
    GPIOD->ODR|=(0x38); //Activar ambas filas
	for(int i = 0; i<1600; i++);
    Enable();		
    GPIOD->ODR |= 0x1;
	for(int i = 0; i<1600; i++);
    Enable();

    GPIOD->ODR |= 0x6;
	for(int i = 0; i<1600; i++);
    Enable();

    GPIOD->ODR |= 0xf;
	for(int i = 0; i<1600; i++);
    Enable();
}