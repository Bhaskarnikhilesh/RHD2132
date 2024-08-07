#include "drivers.h"
#include "rhdregisters.h"

uint16_t data_rec[6];
uint16_t dataBuffer[BUFFER_SIZE];
volatile uint16_t dataIndex = 0;
uint16_t data;
uint16_t rxdata;
uint16_t dummy;
uint16_t calibrate;
uint16_t data3[10];


void error_led_init(void){
    /* Enable clock access to GPIOA */
    RCC->AHBENR |= GPIOAEN;
    /* Set PA5 to output mode */
    GPIOA->MODER |= (1U<<10);
    GPIOA->MODER &=~ (1U<<11);
}

void error_led_on(void){
    GPIOA->ODR |= (1U<<5);
}

void error_led_off(void){
    GPIOA->ODR &=~ (1U<<5);
}

void SystickDelay_Microsec(uint16_t delay)
{
	/*RELOAD WITH NUMBERS OF CLOCKS PER MICRO SECONDS*/
	SysTick->LOAD = SYSTICK_LOAD_VAL;

	/*CLEAR SYSTICK CURRENT VALUE REGISTER*/
	SysTick->VAL = 0;

	/*ENABLE SYSTICK AND SELECT INTERNAL CLK SRC*/
	SysTick->CTRL = SYSTICK_CTRL_EN | SYSTICK_CLKSRC;

	for(uint16_t i = 0; i < delay; i++ ){
		/*WAIT UNTIL THE COUNTFLAG IS SET*/
		while((SysTick->CTRL & SYSTICK_COUNTFLAG)==0){}
	}
	SysTick->CTRL = 0;
}
void spi_gpio_init(void){
	/*enable clock access to GPIOA AND GPIOB*/
	RCC->AHBENR |= GPIOAEN;
	//RCC->AHBENR |= GPIOBEN;
	/* SET PA5, PA6, PA7, PB4 MODE TO ALTERNATE FUNCTION MODE*/
	/* PA5 SPI1_SCLK*/
	GPIOA->MODER &=~ (1U<<10);
	GPIOA->MODER |= (1U<<11);
	/* PA6 SPI1_MISO*/
	GPIOA->MODER &=~ (1U<<12);
	GPIOA->MODER |= (1U<<13);
	/* PA7 SPI1_MOSI*/
	GPIOA->MODER &=~ (1U<<14);
	GPIOA->MODER |= (1U<<15);
	/* PA9 CS*/
	GPIOA->MODER |= (1U<<18);
	GPIOA->MODER &=~ (1U<<19);
	/*PB4*/
	//GPIOB->MODER &=~ (1U<<8);
	//GPIOB->MODER |=  (1U<<9);
	/* SET PA5,6,7 ALTERNATE FUNCTION TYPE TO SPI AF0*/

	/*PA5 Alternate function mapping*/
	GPIOA->AFR[0] &=~ (1U<<20);
	GPIOA->AFR[0] &=~ (1U<<21);
	GPIOA->AFR[0] &=~ (1U<<22);
	GPIOA->AFR[0] &=~ (1U<<23);
	/*PA6 Alternate function mapping*/
	GPIOA->AFR[0] &=~ (1U<<24);
	GPIOA->AFR[0] &=~ (1U<<25);
	GPIOA->AFR[0] &=~ (1U<<26);
	GPIOA->AFR[0] &=~ (1U<<27);
	/*PA7 ALternate function mapping*/
	GPIOA->AFR[0] &=~ (1U<<28);
	GPIOA->AFR[0] &=~ (1U<<29);
	GPIOA->AFR[0] &=~ (1U<<30);
	GPIOA->AFR[0] &=~ (1U<<31);
	/*PB9*/
	/*GPIOB->AFR[0] &=~ (1U<<16);
	GPIOB->AFR[0] &=~ (1U<<17);
	GPIOB->AFR[0] &=~ (1U<<18);
	GPIOB->AFR[0] &=~ (1U<<19);*/

}

void spi_configure(void){
	/*ENABLE CLOCK ACCESS TO SPI1 MODULE*/
	RCC->APB2ENR |= (1U<<12);
	/*SET CLOCK TO FPCLK/2 i.e 48Mhz/4 = 12Mhz when we divide 1/12Mhz= 83.33 ns which is specified by RHD2132 Intan datasheet*/
	SPI1->CR1 |= (1U<<3);
	SPI1->CR1 &=~ (1U<<4);
	SPI1->CR1 &=~ (1U<<5);
	/*SET CPOL TO 0 CPHA TO 0*/
	//SPI1->CR1 |= (1U<<0);
	//SPI1->CR1 |= (1U<<1);
	SPI1->CR1 &=~ (1U<<0);
	SPI1->CR1 &=~ (1U<<1);
	/*ENABLE FULL DUPLEX*/
	SPI1->CR1 &=~ (1U<<10);
	/*SET MODE TO MASTER*/
	SPI1->CR1 |= (1U<<2);
	/*SET 16 BIT DATA MODE*/
	//SPI1->CR1 &=~ (1U<<11);
	SPI1->CR1 |= (1U<<11);
	/*SELECT SOFTWARE SLAVE MANAGEMENT BY SELECTING SSM =1 AND SSI = 1*/
	SPI1->CR1 |= (1U<<8);
	SPI1->CR1 |= (1U<<9);
	/*MSB FIRST*/
	SPI1->CR1 &=~ (1U<<7);
	/*ENABLE SPI MODULE*/
	SPI1->CR1 |= (1U<<6);
}

uint16_t spi_transmit_receive(uint16_t txdata1){
	cs_enable();
	//SystickDelay_Microsec(20);
	uint16_t rxdata;
	uint16_t temp = 0;
	//error_led_on();
    /*wait for until TXE is set*/
    while(!(SPI1->SR & (1U<<1))){}

    /*WRITE THE DATA TO THE DATA REGISTER*/
    SPI1->DR = txdata1;

    //SPI1->DR = 0;

    while (!(SPI1->SR & (1U<<0))) {}
    rxdata = (SPI1->DR);

    /*WAIT FOR BUSY FLAG TO RESET*/

    while((SPI1->SR & (1U<<7))){}

    /*CLEAR OVR FLAG*/
    temp = SPI1->DR;
    temp = SPI1->SR;

    //SystickDelay_Microsec(20);

    cs_disable();
    //error_led_off();
    return rxdata;


}
/*uint16_t spi1_receive(void) {
	// Wait for RXNE flag to be set
	    while (!(SPI1->SR & SPI_SR_RXNE)) {}

	    return SPI1->DR;
}*/

/*PULL PA9 LOW TO ENABLE*/
void cs_enable(void){
	GPIOA->ODR &=~ (1U<<9);
}
/*PULL PA9 HIGH TO DISABLE*/
void cs_disable(void){
	GPIOA->ODR |= (1U<<9);
}

void tim3_20kS_init_interrupt(void){

	/*ENABLE CLOCK ACESS FOR TIMER 3*/
	RCC->APB1ENR |= TIM3ENR;

	/*SET PRESCALAR VALUE TO 2 BECAUSE 48MHZ/2 = 24MHZ AND
	 * AUTO RELOAD VALUE TO 50 BECAUSE 24MHZ/1200 = 20KHZ*/
	TIM3->PSC = 4-1;
	TIM3->ARR = 400-1;

	/*CLEAR COUNTER*/
	TIM3->CNT = 0;

	/*ENABLE TIMER3*/
	TIM3->CR1 = CR1EN;

	/*ENABLE TIMER3 INTERRUPT*/
	TIM3->DIER |= UIE;

	/*ENABLE TIM INTERRUPT IN NVIC*/
	NVIC_EnableIRQ(TIM3_IRQn);
}

void TIM3_IRQHandler(void){

	if(TIM3->SR & UIF){
		TIM3->SR &=~ UIF; // Clear interrupt flag
		//uint16_t command = convert_command(24, 0);
		uint16_t command = convert_command(1, 0);
		//uint16_t command = read_command(40);
		data = spi_transmit_receive(command);
		//spi_transmit_receive(command2);

		dataBuffer[dataIndex++] = data;

		if (dataIndex >= BUFFER_SIZE) {
			dataIndex = 0;
			//START DMA TRANSFER
			/*DISABLE DMA CHANNEL 4*/
			DMA1_Channel4->CCR &=~ DMA_EN;
			DMA1_Channel4->CNDTR = BUFFER_SIZE; // Number of data items
			DMA1_Channel4->CCR |= DMA_CCR_EN; // Enable DMA channel
		}

		//error_led_off();
	}
}

void uart2_tx_DMA_init(void){
	/*CONFIGURE UART GPIO PIN
	 * ENABLE CLOCK ACCESS TO GPIO A
	 */
	RCC->AHBENR |= GPIOAEN;

	/*SET PA2 TO ALTERNATE FUNCTION MODE*/
	GPIOA->MODER &=~ (1U<<4);
	GPIOA->MODER |=  (1U<<5);

	/*SET PA2 ALTERNATE FUNCTION TYPE TO UART2_TX (AF1)(UART_TX)*/
	GPIOA->AFR[0] |=  (1U<<8);
	GPIOA->AFR[0] &=~ (1U<<9);
	GPIOA->AFR[0] &=~ (1U<<10);
	GPIOA->AFR[0] &=~ (1U<<11);

	/*ENABLE CLOCK ACCESS TO UART2*/
	RCC->APB1ENR |= UART2EN;

	/*SET BAUDRATE*/
	USART2->BRR = SYS_CLK_FREQ / USART_BAUDRATE;

	/*SELECT TO USE DMA FOR TX*/
	USART2->CR3 |= DMAT;

	/*SET TRANSFER DIRECTION*/
	USART2->CR1 |= CR1_TE;

	/*ENABLE UART2 MODULE*/
	USART2->CR1 |= UART2_UE;

	/*ENABLE CLOCK ACCESS TO DMA*/
	RCC->AHBENR |= DMAEN;

	/*DISABLE DMA CHANNEL 4*/
	DMA1_Channel4->CCR &=~ DMA_EN;

	/*WAIT TILL DMA CHANNEL IS DISABLED*/
	while(DMA1_Channel4->CCR & DMA_EN){}

	/*CLEAR INTERRUPT FLAGS FOR CHANNEL 4*/
	DMA1->IFCR |= CTCIF4 | CHTIF4 | CTEIF4 | CGIF4;

	/*SET PERIPHERAL ADDRESS*/
	DMA1_Channel4->CPAR = (uint32_t)&USART2->TDR;

	/*SET MEMORY ADDRESS*/
	DMA1_Channel4->CMAR = (uint32_t)dataBuffer; // Memory address

	/*ENABLE MEMORY ADDRESS INCREMENT AND READ FROM MEMORY*/
	DMA1_Channel4->CCR |= MINC | DIR;

}

void dummy_command_list(void){
	for(uint16_t start = 0; start<9; start++){
		spi_transmit_receive(read_command(40));
	}
}

// RHD2000 ADC Initialization
void adc_init(void) {
	uint16_t init_commands[18] = {
	    write_command(0, REGISTER_0), // Command 1
	    write_command(1, REGISTER_1), // Command 2
	    write_command(2, REGISTER_2), // Command 3
	    write_command(3, REGISTER_3), // Command 4
	    write_command(4, REGISTER_4), // Command 5
	    write_command(5, REGISTER_5), // Command 6
            write_command(6, REGISTER_6), // Command 7
            write_command(7, REGISTER_7), // Command 8
	    write_command(8, REGISTER_8), // Command 9
	    write_command(9, REGISTER_9), // Command 10
	    write_command(10, REGISTER_10),// Command 11
	    write_command(11, REGISTER_11),// Command 12
	    write_command(12, REGISTER_12),// Command 13
	    write_command(13, REGISTER_13),// Command 14
	    write_command(14, REGISTER_14),// Command 15
	    write_command(15, REGISTER_15),// Command 16
	    write_command(16, REGISTER_16),// Command 17
            write_command(17, REGISTER_17)// Command 18
	};

    // Send initial configuration commands
    for(uint16_t i = 0; i<=18; i++){
    	spi_transmit_receive((init_commands[i]));
    	spi_transmit_receive(read_command(i));
    }

    // Send a few dummy commands in case chip is still powering up.
    spi_transmit_receive(read_command(40));
	spi_transmit_receive(read_command(40));

    /*DELAY FOR ATLEAST 100 MICROSECOND AFTER amp Vref enable i.e. D[4] bit IS SET IN REGISTER 0 */
    SystickDelay_Microsec(MICROSEC_DELAY);

    // Perform Calibration
    uint16_t calibrate = calibrate_command();
    spi_transmit_receive(calibrate);

    /*SEND 9 DUMMY COMMANDS AFTER CALIBRATE COMMAND SINCE 9 COMMANDS FOLLWING CALIBRATE COMMAND WILL NOT EXECUTE*/
    dummy_command_list();

    }

void RHD_Read(void){
	uint16_t read_init[10];
	read_init[0]=read_command(40);
	read_init[1]=read_command(41);
	read_init[2]=read_command(42);
	read_init[3]=read_command(43);
	read_init[4]=read_command(44);
	read_init[5]=read_command(60);
	read_init[6]=read_command(61);
	read_init[7]=read_command(62);
	read_init[8]=read_command(63);
	read_init[9]=read_command(48);

	//read the intan chip
	for(uint16_t j= 0; j<=10; j++){
		data3[j] = spi_transmit_receive((read_init[j]));
        //printf("Register %d Data: %02X\n", 60 + j, received_data[j] & 0xFF); // Print received data for verification
	}


}


