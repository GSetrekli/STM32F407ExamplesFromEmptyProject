
#define GPIOA_BASEADDR			0x40020000UL
#define GPIO_MODER_OFFSET		0x00
#define GPIO_TYPER_OFFSET		0x04
#define GPIO_OSPEEDR_OFFSET		0x08
#define GPIO_PUPDR_OFFSET		0x0C
#define GPIO_IDR_OFFSET			0x10
#define GPIO_ODR_OFFSET			0x14
#define GPIO_AFLR_OFFSET		0x20

#define RCC_BASE_ADDR			0x40023800UL
#define RCC_AHB1ENR_OFFSET		0x30
#define RCC_APB2ENR_OFFSET		0x44

#define ABP2_BASEADDR			0x40010000UL
#define SPI1_BASEADDR			0x40013000UL
#define SPI1_CR1_OFFSET			0x00
#define SPI1_CR2_OFFSET			0x04
#define SPI1_SR_OFFSET			0x08
#define SPI1_DR_OFFSET			0x0C

#include <stdint.h>
#include <string.h>


uint32_t *pGPIOAModeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_MODER_OFFSET);
uint32_t *pGPIOATypeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_TYPER_OFFSET);
uint32_t *pGPIOAOSpeedReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_OSPEEDR_OFFSET);
uint32_t *pGPIOAPupdReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_PUPDR_OFFSET);
uint32_t *pGPIOAAFLReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_AFLR_OFFSET);

uint32_t *pSPI1SRReg=(uint32_t*)(SPI1_BASEADDR+SPI1_SR_OFFSET);
uint32_t *pSPI1DRReg=(uint32_t*)(SPI1_BASEADDR+SPI1_DR_OFFSET);


void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}

void SPI_SendData(uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0)
	{
		//1. wait until TXE is set(empty)
		while( ! (*pSPI1SRReg & (1<<1)) );

		*pSPI1DRReg =*pTxBuffer;
		Len--;
		pTxBuffer++;
	}

}

void SPI1GPIOInit(uint8_t PinNo)
{
	*pGPIOAModeReg &= ~(0x3<< (PinNo*2));
	*pGPIOAModeReg |= (0x2<<(PinNo*2)); //assignment to AF.

	*pGPIOAPupdReg &= ~(0x3<<(PinNo*2));

	*pGPIOATypeReg &= ~(1<<(PinNo*2));

	*pGPIOAOSpeedReg &= ~(0x3<<(PinNo*2));
	*pGPIOAOSpeedReg |= (0x2<<(PinNo*2));

	*pGPIOAAFLReg &=~(0xF<<(PinNo*4));
	*pGPIOAAFLReg |=(0x5<<(PinNo*4)); // assign PA5 to AF5
}


/*
 * PA4 --> SPI1_NSS
 * PA5 --> SPI1_CLK
 * PA6 -> SPI1_MISO
 * PA7 --> SPI1_MOSI
 * ALT function mode : 5
 * SPI1-->APB2 (84 MHz)
 */


int main(void)
{
	char user_data[] = "HELLO";

	uint32_t *pRccAhb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET);
	*pRccAhb1EnrReg |=(1<<0);  //GPIOA clock Enable

	uint8_t NSSpin=4;
	uint8_t CLKpin=5;
	uint8_t MOSIpin=7;

	/**** Button PA0*****/
	*pGPIOAModeReg &= ~(0x3<<0);  //PA0 -Input

	*pGPIOAOSpeedReg &= ~(0x3<<0);
	*pGPIOAOSpeedReg |= (0x2<<0); // PA0-Fast

	*pGPIOAPupdReg &= ~(0x3<<0); // PA0- no pull-up pull-down
	//*pGPIOAPupdReg |= (0x1<<0);
	/*******************/

	SPI1GPIOInit(CLKpin);	//CLK--PA5
	SPI1GPIOInit(MOSIpin);
	SPI1GPIOInit(NSSpin);

	uint32_t *pRccApb2EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_APB2ENR_OFFSET);
	*pRccApb2EnrReg |=(1<<12);  //SPI1 Clock Enable

	uint32_t TempReg=0;
	uint32_t *pSPI1CR1Reg=(uint32_t*)(SPI1_BASEADDR+SPI1_CR1_OFFSET);
	TempReg |= (1<<2);  //Master
	TempReg &= ~(1<<15); //Full Dublex
	TempReg |= (0x2<<3);  //BaudRate (16 MHz/8)
	TempReg &= ~(1<<15);  //DFF-8 bit
	TempReg &= ~(1<<1);  //CPOL-0
	TempReg &= ~(1<<0);  //CPHA-0
	TempReg &= ~(1<<9);  //SSM -Disable (Hardware Slave Management Enabled )
	*pSPI1CR1Reg =TempReg;

	uint32_t *pSPI1CR2Reg=(uint32_t*)(SPI1_BASEADDR+SPI1_CR2_OFFSET);
	*pSPI1CR2Reg &= ~(1<<2);
	*pSPI1CR2Reg |= (1<<2);  //HardwareMode, if in master,SSOE=1 SSM=0

	uint32_t *GPIOAIdrReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_IDR_OFFSET);

	while(1)
	{
		while (!((*GPIOAIdrReg  >>  0) & 0x00000001)); //wait until press the Btn.
		delay();
		*pSPI1CR1Reg &= ~(1<<6);
		*pSPI1CR1Reg |= (1<<6); //SPE=1,So NSS=0

		uint8_t dataLen = strlen(user_data);
		SPI_SendData(&dataLen,1);
		SPI_SendData((uint8_t*)user_data,strlen(user_data));

		while ((*pSPI1SRReg >>  7) & 0x00000001); //wait until Busy flag is available.
		*pSPI1CR1Reg &= ~(1<<6);

	}
}
