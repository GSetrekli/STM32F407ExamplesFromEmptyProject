#include <stdio.h>
#include <stdint.h>
#include <string.h>

/*
 AF7, UART2
 TX--PA2
 RX--PA3
 */

#define GPIOA_BASEADDR 				0x40020000UL
#define GPIO_MODER_OFFSET 			0x00
#define GPIO_TYPER_OFFSET 			0x04
#define GPIO_OSPEEDR_OFFSET 		0x08
#define GPIO_PUPDR_OFFSET    		0x0C
#define GPIO_IDR_OFFSET    			0x10
#define GPIO_ODR_OFFSET 			0x14
#define GPIO_AFLR_OFFSET 			0x20
#define GPIO_AFHR_OFFSET 			0x24

#define RCC_BASE_ADDR 				0x40023800UL
#define RCC_AHB1ENR_OFFSET 			0x30
#define RCC_APB1ENR_OFFSET 			0x40
#define RCC_APB2ENR_OFFSET 			0x44

#define APB2_BASEADDR 				0x40010000UL
#define USART2_BASEADDR 			0x40004400UL
#define USART_SR_OFFSET   			0x00
#define USART_DR_OFFSET 			0x04
#define USART_BRR_OFFSET 			0x08
#define USART_CR1_OFFSET 			0x0C
#define USART_CR2_OFFSET 			0x10
#define USART_CR3_OFFSET 			0x14


uint8_t *msg = "trb...\n\r";

//reply from arduino will be stored here
uint8_t *rx_buf ;
uint8_t RXCOMLETED=0;

//extern void initialise_monitor_handles();

uint32_t *pRccAhb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_AHB1ENR_OFFSET);
uint32_t *pRccApb1EnrReg=(uint32_t*)(RCC_BASE_ADDR+RCC_APB1ENR_OFFSET);

uint32_t *pGPIOAModeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_MODER_OFFSET);
uint32_t *pGPIOATypeReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_TYPER_OFFSET);
uint32_t *pGPIOAOSpeedReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_OSPEEDR_OFFSET);
uint32_t *pGPIOAPupdReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_PUPDR_OFFSET);
uint32_t *pGPIOAAFHReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_AFHR_OFFSET);
uint32_t *pGPIOAAFLReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_AFLR_OFFSET);
uint32_t *GPIOAIdrReg=(uint32_t*)(GPIOA_BASEADDR+GPIO_IDR_OFFSET);

uint32_t *pUSART2CR1Reg=(uint32_t*)(USART2_BASEADDR+USART_CR1_OFFSET);
uint32_t *pUSART2CR2Reg=(uint32_t*)(USART2_BASEADDR+USART_CR2_OFFSET);
uint32_t *pUSART2CR3Reg=(uint32_t*)(USART2_BASEADDR+USART_CR3_OFFSET);
uint32_t *pUSART2SRReg=(uint32_t*)(USART2_BASEADDR+USART_SR_OFFSET);
uint32_t *pUSART2DRReg=(uint32_t*)(USART2_BASEADDR+USART_DR_OFFSET);
uint32_t *pUSART2BRReg=(uint32_t*)(USART2_BASEADDR+USART_BRR_OFFSET);

uint8_t convert_to_capital(uint8_t data);

void delay(void)
{
for(uint32_t i = 0 ; i < 500000/2 ; i ++);
}


void USART1GPIOInit(uint8_t PinNo)
{
*pGPIOAModeReg &= ~(0x3<< (PinNo*2));
*pGPIOAModeReg |= (0x2<<(PinNo*2)); 	//assignment to AF.

*pGPIOAPupdReg &= ~(0x3<<(PinNo*2));
*pGPIOAPupdReg |= (0x1<<(PinNo*2));

*pGPIOATypeReg &= ~(1<< PinNo);

*pGPIOAOSpeedReg &= ~(0x3<<(PinNo*2));
*pGPIOAOSpeedReg |= (0x2<<(PinNo*2));

if(PinNo<8)
{
*pGPIOAAFLReg &=~(0xF<<(PinNo*4));		 // assign PA9-10 to AF7
*pGPIOAAFLReg |=(0x7<<(PinNo*4));
}
else
{
*pGPIOAAFHReg &=~(0xF<<((PinNo-8)*4));
*pGPIOAAFHReg |=(0x7<<((PinNo-8)*4));
}
}


void USART_SetBaudRate(uint32_t BaudRate)
{
uint32_t usartdiv;

if(BaudRate==115200) 	//-- PCLK=16MHz, //Over sampling by 16
{
usartdiv=86875;
}
else if(BaudRate==9600)
{
usartdiv=1041875;
}

//variables to hold Mantissa and Fraction values
uint32_t M_part=(usartdiv/10000);
uint32_t F_part=(usartdiv-(M_part*10000));

uint32_t tempreg=0;

tempreg |= (M_part << 4);
F_part=(((F_part*16)/10000) & (uint8_t)0x0F);

tempreg |=F_part;
*pUSART2BRReg=tempreg;

}

void USART_ReadData(uint8_t *pRxBuffer, uint32_t Len)
{

	while(! (*pUSART2SRReg & (1<<5)));

	for (uint32_t i=0; i<Len; i++)
	{
	//Implement the code to wait until RXNE flag is set in the SR

		*pRxBuffer = (uint8_t) (*pUSART2DRReg & (uint8_t)0xFF);
		pRxBuffer++;

	if (i==(Len-1))
	{
		RXCOMLETED = 1;
	}
	}

}
void USART_SendData(uint8_t *pTxBuffer, uint32_t Len)
{

for(uint32_t i = 0 ; i < Len; i++)
{
	//Implement the code to wait until TXE flag is set in the SR
	while(! (*pUSART2SRReg & (1<<7)));

	*pUSART2DRReg = (*pTxBuffer  & (uint8_t)0xFF);

	pTxBuffer++;

//Implement the code to wait till TC flag is set in the SR
	while( ! (*pUSART2SRReg & (1<<6)));

}
}



int main(void)
{

*pRccAhb1EnrReg &=~(1<<0);
*pRccAhb1EnrReg |=(1<<0);  		//GPIOA clock Enable

/******** Button PA0*********/
*pGPIOAModeReg &= ~(0x3<<0);  	//PA0 -Input

*pGPIOAOSpeedReg &= ~(0x3<<0);
*pGPIOAOSpeedReg |= (0x2<<0); 	// PA0-Fast

*pGPIOAPupdReg &= ~(0x3<<0); 	// PA0- no pull-up pull-down
//*pGPIOAPupdReg |= (0x1<<0);
/***************************/

uint8_t TXpin=2;
uint8_t RXpin=3;

//initialise_monitor_handles(); //**********************************************

USART1GPIOInit(TXpin); 		//TX--PA2
USART1GPIOInit(RXpin); 		//RX--PA3

*pRccApb1EnrReg &= ~(1<<17);
*pRccApb1EnrReg |= (1<<17);

uint32_t tempreg=0;
tempreg |= (( 1 << 3 )| (1<<2));		// TX ,RX enable
//tempreg |= ( 1 << 3 );

tempreg &= ~( 1 << 12 );	//word length 8 bits

tempreg &= ~(1<<10);		//parity bit disable

*pUSART2CR1Reg=tempreg;		//cr1 register (( OVER8=0)-->Over sampling by 16)

tempreg=0;
tempreg &= ~(0x3<<12);		//Stop bit 1

*pUSART2CR2Reg=tempreg;		//Cr2 register

tempreg=0;
*pUSART2CR3Reg=tempreg;		//NO RTS - CTS

USART_SetBaudRate(115200);

*pUSART2CR1Reg |= (1<<13);	//UART ENABLE


//uint8_t rcvd_data;
uint8_t data_buffer[100];
uint32_t count=0;

while(1)
  {

		USART_SendData(msg,8);
//		printf("Transmitted    : %s\n",msg);

	//wait till button is pressed
	//while (!((*GPIOAIdrReg ) & 0x00000001)); //wait until press the Btn

	//delay();

	while(1)
	{
		USART_ReadData(rx_buf,1);

				if(*rx_buf == '\r')
				{
					break;
				}
				else
				{
					data_buffer[count++]=convert_to_capital(*rx_buf);
					data_buffer[count++]= '\r';
				}

				//USART_SendData((uint8_t*)data_buffer, count);
				//count=0;
	}
	USART_SendData((uint8_t*)data_buffer, count);
	count=0;

	//just make sure that last byte should be null otherwise %s fails while printing
	//rx_buf[strlen(msg)+ 1] = '\0';

//	printf("Received    : %s\n",rx_buf);

	RXCOMLETED=0;
  }

return 0;
}

uint8_t convert_to_capital(uint8_t data)
{
	if( data >= 'a' && data <= 'z')
	{
		data = data - ( 'a'- 'A');
	}

	return data;
}


