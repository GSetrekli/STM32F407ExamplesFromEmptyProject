#include <stdint.h>
#include <stdio.h>

#define GPIOD_BASE_ADDR			0x40020C00UL
#define GPIO_MODER_OFFSET		0x00
#define GPIO_TYPER_OFFSET		0x04
#define GPIO_OSPEEDR_OFFSET		0x08
#define GPIO_PUPDR_OFFSET		0x0C
#define GPIO_ODR_OFFSET			0x14

#define RCC_BASE_ADDR			0x40023800UL
#define RCC_AHB1ENR_OFFSET		0x30
#define RCC_APB2ENR_OFFSET		0x44

#define EXTI_BASE_ADDR			0x40013C00UL
#define EXTI_IMR_OFFSET			0x00
#define EXTI_RISER_OFFSET		0x08
#define EXTI_FALLR_OFFSET		0x0C
#define EXTI_PENDR_OFFSET		0x14

#define SYSCFG_BASE_ADDR		0x40013800UL
#define SYSCFG_EXTICR2_OFFSET	0x0C


#define NVIC_ISER0				(volatile uint32_t*)0xE000E100UL
#define NVIC_ISER1				(volatile unint32_t*)0xE000E104UL
#define NVIC_ISER2				(volatile unint32_t*)0xE000E108UL
#define NVIC_ISER3				(volatile unint32_t*)0xE000E10CUL


uint32_t *EXTIPendReg=(uint32_t*)(EXTI_BASE_ADDR+EXTI_PENDR_OFFSET);
uint32_t *pGPIODOdrReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_ODR_OFFSET);

void delay(void)
{
	// this will introduce ~200ms delay when system clock is 16MHz
	for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


int main(void)
{

  uint32_t *pRccAhb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET);
  *pRccAhb1EnrReg |=(1<<3);

  uint32_t *pGPIODModeReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_MODER_OFFSET);
  *pGPIODModeReg &= ~(0x3<<24);
  *pGPIODModeReg |= (0x1<<24); //PD 12 as Output.

  uint32_t *pGPIODTypeReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_TYPER_OFFSET);
  *pGPIODTypeReg &= ~(1<<12); //PD12- push pull

  uint32_t *pGPIODOSpeedReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_OSPEEDR_OFFSET);
  *pGPIODOSpeedReg &= ~(0x3<<24);
  *pGPIODOSpeedReg |= (0x2<<24); // D12-Fast

  uint32_t *pGPIODPupdrReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_PUPDR_OFFSET);
  *pGPIODPupdrReg &= ~(0x3<<24); // PD12- no pull-up pull-down



 /*************Button ****************/

  *pGPIODOSpeedReg &= ~(0x3<<10);
  *pGPIODOSpeedReg |= (0x2<<10); // D5-Fast

  *pGPIODPupdrReg &= ~(0x3<<10); //PD5- PU
  *pGPIODPupdrReg |=(0x1<<10);
/****************************************/

  uint32_t *pEXTIFallReg=(uint32_t*)(EXTI_BASE_ADDR+EXTI_FALLR_OFFSET);
  uint32_t *pEXTIRiseReg=(uint32_t*)(EXTI_BASE_ADDR+EXTI_RISER_OFFSET);
  *pEXTIFallReg |=(1<<5);
  *pEXTIRiseReg &= ~(1<<5);  //Falling Edge Trigger

  uint32_t *pRccApb2EnReg =(uint32_t*)(RCC_BASE_ADDR+RCC_APB2ENR_OFFSET);
  *pRccApb2EnReg |=(1<<14);  //Clock Enable for SYSCFG Reg -> SYSCFG_EXTICR2

  uint32_t *pEXTIConfigReg=(uint32_t*)(SYSCFG_BASE_ADDR+SYSCFG_EXTICR2_OFFSET);
  *pEXTIConfigReg &= ~(0xF<<4);
  *pEXTIConfigReg |=(0x3<<4);	// Config. SYSCFG_EXTICR2: PD5:EXTI line 5,0011 for D

  uint32_t *pEXTIImrReg=(uint32_t*)(EXTI_BASE_ADDR+EXTI_IMR_OFFSET);
  *pEXTIImrReg &= ~(1<<5);
  *pEXTIImrReg |=(1<<5);

  uint8_t IRQ_NO_EXTI9_5=23;	 //PD5: EXTI 5th Line ---> 23rd (IRQ Number) position in NVIC.

  *NVIC_ISER0 &= ~(1<<IRQ_NO_EXTI9_5);
  *NVIC_ISER0 |=(1<<IRQ_NO_EXTI9_5);   //Enable Interrupt to NVIC.

  while(1);

}
void EXTI9_5_IRQHandler(void)
  {
	delay();

	if (*EXTIPendReg & (1<<5))
	{
		*EXTIPendReg |=(1<<5); //clear
	}

	*pGPIODOdrReg ^= (1<<12);
  }

