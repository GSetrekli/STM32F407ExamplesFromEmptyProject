#include <stdint.h>
#include <stdio.h>

#define GPIOD_BASE_ADDR			0x40020C00UL
#define GPIO_MODER_OFFSET		0x00
#define GPIO_TYPER_OFFSET		0x04
#define GPIO_OSPEEDR_OFFSET		0x08
#define GPIO_PUPDR_OFFSET		0x0C
#define GPIO_ODR_OFFSET			0x14
#define RCC_BASE_ADDR			0x40023800UL
#define RCC_AHB1RSTR_OFFSET		0x10
#define RCC_AHB1ENR_OFFSET		0x30

void delay(void)
{
	for(uint32_t i=0; i<50000; i++);
}

int main(void)
{


  uint32_t *pRccAhb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET);
  *pRccAhb1EnrReg |=(1<<3); //First of all ,enable the clock perip. on the bus.

  uint32_t *pGPIODModeReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_MODER_OFFSET);
  *pGPIODModeReg &= ~(0x3<<24);
  *pGPIODModeReg |= (0x1<<24); //PD 12 as Output.

  uint32_t *pGPIODTypeReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_TYPER_OFFSET);
  *pGPIODTypeReg &= ~(1<<12); //push pull

  uint32_t *pGPIODOSpeedReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_OSPEEDR_OFFSET);
  *pGPIODOSpeedReg &= ~(0x3<<24);
  *pGPIODOSpeedReg |= (0x2<<24); //Fast

  uint32_t *pGPIODPupdrReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_PUPDR_OFFSET);
  *pGPIODPupdrReg &= ~(0x3<<24); //no pull-up pull-down

  uint32_t *pGPIODOdrReg=(uint32_t*)(GPIOD_BASE_ADDR+GPIO_ODR_OFFSET); //Output Data Reg.


  while(1)
  {
	  *pGPIODOdrReg ^= (1<<12); //toggle the led, which is PD12.
	  delay();
  }

	return 0;
}
