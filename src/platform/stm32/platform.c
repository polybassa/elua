// Platform-dependent functions

#include "platform.h"
#include "type.h"
#include "devman.h"
#include "genstd.h"
#include <reent.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include "uip_arp.h"
#include "elua_uip.h"
#include "elua_adc.h"
#include "uip-conf.h"
#include "platform_conf.h"
#include "diskio.h"
#include "common.h"
#include "buf.h"
#include "utils.h"
#include "lua.h"
#include "lauxlib.h"
#include "lrotable.h"

// Platform specific includes
#include "stm32f10x.h"

// Clock data
// IMPORTANT: if you change these, make sure to modify RCC_Configuration() too!
#define HCLK        ( HSE_Value * 9 )
#define PCLK1_DIV   2
#define PCLK2_DIV   1

// SysTick Config Data
// NOTE: when using virtual timers, SYSTICKHZ and VTMR_FREQ_HZ should have the
// same value, as they're served by the same timer (the systick)
// Max SysTick preload value is 16777215, for STM32F103RET6 @ 72 MHz, lowest acceptable rate would be about 5 Hz
#define SYSTICKHZ               10  
#define SYSTICKMS               (1000 / SYSTICKHZ)
// ****************************************************************************
// Platform initialization

// forward dcls
static void RCC_Configuration(void);
static void NVIC_Configuration(void);

static void timers_init();
static void uarts_init();
static void pios_init();
static void cans_init();
static void seco_gpios_init();

int platform_init()
{
	// Set the clocking to run from PLL
	RCC_Configuration();

	// Setup IRQ's
	NVIC_Configuration();

	// Setup PIO
	pios_init();

	// Setup UARTs
	uarts_init();

	// Setup timers
	timers_init();

	// Setup CANs
	cans_init();

	// Setup system timer
	cmn_systimer_set_base_freq( HCLK );
	cmn_systimer_set_interrupt_freq( SYSTICKHZ );

	// Enable SysTick
	if ( SysTick_Config( HCLK / SYSTICKHZ ) )
	{ 
		/* Capture error */ 
		while (1);
	}

	// Flash initialization (for WOFS)
	FLASH_Unlock();

	cmn_platform_init();

	// Setup Default GPIOs
	seco_gpios_init();

	// All done
	return PLATFORM_OK;
}

// ****************************************************************************
// Clocks
// Shared by all STM32 devices.
// TODO: Fix to handle different crystal frequencies and CPU frequencies.

/*******************************************************************************
 * Function Name  : RCC_Configuration
 * Description    : Configures the different system clocks.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/
static void RCC_Configuration(void)
{
	SystemInit();

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
}

// ****************************************************************************
// NVIC
// Shared by all STM32 devices.

/*******************************************************************************
 * Function Name  : NVIC_Configuration
 * Description    : Configures the nested vectored interrupt controller.
 * Input          : None
 * Output         : None
 * Return         : None
 *******************************************************************************/

static void NVIC_Configuration(void)
{
	NVIC_InitTypeDef nvic_init_structure;


#ifdef  VECT_TAB_RAM
	/* Set the Vector Table base location at 0x20000000 */
	NVIC_SetVectorTable(NVIC_VectTab_RAM, 0x0);
#else  /* VECT_TAB_FLASH  */
	/* Set the Vector Table base location at 0x08000000 */
	NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0);
#endif

	/* Configure the NVIC Preemption Priority Bits */
	/* Priority group 0 disables interrupt nesting completely */
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);

	// Lower the priority of the SysTick interrupt to let the
	// UART interrupt preempt it
	nvic_init_structure.NVIC_IRQChannel = SysTick_IRQn;
	nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 0; 
	nvic_init_structure.NVIC_IRQChannelSubPriority = 1; 
	nvic_init_structure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&nvic_init_structure);
	
	nvic_init_structure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
	nvic_init_structure.NVIC_IRQChannelPreemptionPriority = 1; 
	nvic_init_structure.NVIC_IRQChannelSubPriority = 1; 
	nvic_init_structure.NVIC_IRQChannelCmd = ENABLE; 
	NVIC_Init(&nvic_init_structure);

	
}

// ****************************************************************************
// PIO
// This is pretty much common code to all STM32 devices.
// todo: Needs updates to support different processor lines.
static GPIO_TypeDef * const pio_port[] = { GPIOA, GPIOB, GPIOC, GPIOD, GPIOE, GPIOF, GPIOG };
static const u32 pio_port_clk[]        = { RCC_APB2Periph_GPIOA, RCC_APB2Periph_GPIOB, RCC_APB2Periph_GPIOC, RCC_APB2Periph_GPIOD, RCC_APB2Periph_GPIOE, RCC_APB2Periph_GPIOF, RCC_APB2Periph_GPIOG };

static void pios_init()
{
	GPIO_InitTypeDef GPIO_InitStructure;
	int port;

	for( port = 0; port < NUM_PIO; port++ )
	{
		// Enable clock to port.
		RCC_APB2PeriphClockCmd(pio_port_clk[port], ENABLE);

		// Default all port pins to input and enable port.
		GPIO_InitStructure.GPIO_Pin  = GPIO_Pin_All;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

		GPIO_Init(pio_port[port], &GPIO_InitStructure);
	}
}

pio_type platform_pio_op( unsigned port, pio_type pinmask, int op )
{
	pio_type retval = 1;
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_TypeDef * base = pio_port[ port ];

	switch( op )
	{
		case PLATFORM_IO_PORT_SET_VALUE:
			GPIO_Write(base, pinmask);
			break;

		case PLATFORM_IO_PIN_SET:
			GPIO_SetBits(base, pinmask);
			break;

		case PLATFORM_IO_PIN_CLEAR:
			GPIO_ResetBits(base, pinmask);
			break;

		case PLATFORM_IO_PORT_DIR_INPUT:
			pinmask = GPIO_Pin_All;
		case PLATFORM_IO_PIN_DIR_INPUT:
			GPIO_InitStructure.GPIO_Pin  = pinmask;
			GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

			GPIO_Init(base, &GPIO_InitStructure);
			break;

		case PLATFORM_IO_PORT_DIR_OUTPUT:
			pinmask = GPIO_Pin_All;
		case PLATFORM_IO_PIN_DIR_OUTPUT:
			GPIO_InitStructure.GPIO_Pin   = pinmask;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_Out_PP;
			GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

			GPIO_Init(base, &GPIO_InitStructure);
			break;

		case PLATFORM_IO_PORT_GET_VALUE:
			retval = pinmask == PLATFORM_IO_READ_IN_MASK ? GPIO_ReadInputData(base) : GPIO_ReadOutputData(base);
			break;

		case PLATFORM_IO_PIN_GET:
			retval = GPIO_ReadInputDataBit(base, pinmask);
			break;

		case PLATFORM_IO_PIN_PULLUP:
			GPIO_InitStructure.GPIO_Pin   = pinmask;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPU;

			GPIO_Init(base, &GPIO_InitStructure);
			break;

		case PLATFORM_IO_PIN_PULLDOWN:
			GPIO_InitStructure.GPIO_Pin   = pinmask;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IPD;

			GPIO_Init(base, &GPIO_InitStructure);
			break;

		case PLATFORM_IO_PIN_NOPULL:
			GPIO_InitStructure.GPIO_Pin   = pinmask;
			GPIO_InitStructure.GPIO_Mode  = GPIO_Mode_IN_FLOATING;

			GPIO_Init(base, &GPIO_InitStructure);
			break;

		default:
			retval = 0;
			break;
	}
	return retval;
}

// ****************************************************************************
// GPIOS FOR SECO


static void seco_gpios_init( void ) {
	/* Initialize all configured peripherals */

	GPIO_InitTypeDef GPIO_InitStruct;

	/*
	 * GPIO A @ 0x40010800
	 PA 0: Floating Input (0)
	 PA 1: Floating Input (0)
	 PA 2: Output 0 (Speed 50MHz)                   // done
	 PA 3: Floating Input (1)
	 PA 4: Floating Input (0)
	 PA 5: Floating Input (0)
	 PA 6: Floating Input (0)
	 PA 7: Floating Input (0)
	 PA 8: Floating Input (0)
	 PA 9: Output alternate 0 (Speed 50MHz)   // done in stm32f1xx_hal_msp.c for USART1
	 PA10: Floating Input (1)				        // done in stm32f1xx_hal_msp.c for USART1
	 PA11: Input with pull-up (1)				// done in stm32f1xx_hal_msp.c for CAN
	 PA12: Floating Input (1)					// done in stm32f1xx_hal_msp.c for CAN
	 PA13: Input with pull-up (0)				// done
	 PA14: Input with pull-down (0)			// done
	 PA15: Input with pull-up (1)				// done
	*/

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_14;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*Configure GPIO pin Output Level */
	GPIO_WriteBit(GPIOA, GPIO_Pin_2, Bit_RESET);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStruct);

	/*
	 *   GPIO B @ 0x40010c00
	PB 0: Output 0 (Speed 50MHz)			//done
	PB 1: Floating Input (0)
	PB 2: Floating Input (0)
	PB 3: Output 1 (Speed 50MHz)			//done
	PB 4: Floating Input (0)
	PB 5: Output 0 (Speed 50MHz)			//done
	PB 6: Floating Input (0)
	PB 7: Floating Input (0)
	PB 8: Input with pull-up (1)			  // done in stm32f1xx_hal_msp.c for CAN_CONFIG 2
	PB 9: Output alternate 0 (Speed 50MHz) // done in stm32f1xx_hal_msp.c for CAN_CONFIG 2
	PB10: Output 0 (Speed 50MHz)			//done
	PB11: Floating Input (0)
	PB12: Output 0 (Speed 50MHz)			//done
	PB13: Output 0 (Speed 50MHz)			//done
	PB14: Output 0 (Speed 50MHz)			//done
	PB15: Output 1 (Speed 50MHz)			//done
	*/
	/*Configure GPIO pin Output Level */
	GPIO_WriteBit(GPIOB,
			GPIO_Pin_0 | GPIO_Pin_5 | GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14,
			Bit_RESET);
	GPIO_WriteBit(GPIOB,
			GPIO_Pin_3 | GPIO_Pin_15,
			Bit_SET);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_3 | GPIO_Pin_5 |
		GPIO_Pin_10 | GPIO_Pin_12 | GPIO_Pin_13 |
		GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStruct);
	/*
	   GPIO C @ 0x40011000
	   PC 0: Output 0 (Speed 50MHz) //done
	   PC 1: Floating Input (0)
	   PC 2: Output 0 (Speed 50MHz) //done
	   PC 3: Floating Input (1)
	   PC 4: Output 0 (Speed 50MHz) //done
	   PC 5: Output 0 (Speed 50MHz) //done
	   PC 6: Output 0 (Speed 50MHz) //done
	   PC 7: Floating Input (1)
	   PC 8: Output 0 (Speed 50MHz) //done
	   PC 9: Output 0 (Speed 50MHz) //done
	PC10: Output 1 (Speed 50MHz) //done
	PC11: Output 1 (Speed 50MHz) //done
	PC12: Output 0 (Speed 50MHz) //done
	PC13: Floating Input (0)
	PC14: Floating Input (0)
	PC15: Floating Input (0)
	*/
	/*Configure GPIO pin Output Level */
	GPIO_WriteBit(GPIOC,
			GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 |
			GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 |
			GPIO_Pin_9 | GPIO_Pin_12,
			Bit_RESET);
	GPIO_WriteBit(GPIOC,
			GPIO_Pin_10 | GPIO_Pin_11,
			Bit_SET);

	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_2 | GPIO_Pin_4 |
		GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_8 |
		GPIO_Pin_9 | GPIO_Pin_10 | GPIO_Pin_11 |
		GPIO_Pin_12;
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStruct);
}

// ****************************************************************************
// CAN
// TODO: Many things



void cans_init( void )
{
	// Remap CAN to PB8/9
//	GPIO_PinRemapConfig( GPIO_Remap1_CAN1, ENABLE );

	// CAN Periph clock enable
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);
}

/*       BS1 BS2 SJW Pre
1M:      5   3   1   4
500k:    7   4   1   6
250k:    9   8   1   8
125k:    9   8   1   16
100k:    9   8   1   20 */

#define CAN_BAUD_COUNT 5
static const u8 can_baud_bs1[]    = { CAN_BS1_9tq, CAN_BS1_9tq, CAN_BS1_9tq, CAN_BS1_7tq, CAN_BS1_5tq };
static const u8 can_baud_bs2[]    = { CAN_BS1_8tq, CAN_BS1_8tq, CAN_BS1_8tq, CAN_BS1_4tq, CAN_BS1_3tq };
static const u8 can_baud_sjw[]    = { CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq, CAN_SJW_1tq };
static const u8 can_baud_pre[]    = { 20, 16, 8, 6, 4 };
static const u32 can_baud_rate[]  = { 100000, 125000, 250000, 500000, 1000000 };

u32 platform_can_setup( unsigned id, u32 clock )
{
	CAN_InitTypeDef        CAN_InitStructure;
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	int cbaudidx = -1;

	// Configure IO Pins -- This is for STM32F103RE
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init( GPIOA, &GPIO_InitStructure );

	// Select baud rate up to requested rate, except for below min, where min is selected
	if ( clock >= can_baud_rate[ CAN_BAUD_COUNT - 1 ] ) // round down to peak rate if >= peak rate
		cbaudidx = CAN_BAUD_COUNT - 1;
	else
	{
		for( cbaudidx = 0; cbaudidx < CAN_BAUD_COUNT - 1; cbaudidx ++ )
		{
			if( clock < can_baud_rate[ cbaudidx + 1 ] ) // take current idx if next is too large
				break;
		}
	}

	/* Deinitialize CAN Peripheral */
	CAN_DeInit( CAN1 );
	CAN_StructInit( &CAN_InitStructure );

	/* CAN cell init */
	CAN_InitStructure.CAN_TTCM=DISABLE;
	CAN_InitStructure.CAN_ABOM=DISABLE;
	CAN_InitStructure.CAN_AWUM=DISABLE;
	CAN_InitStructure.CAN_NART=DISABLE;
	CAN_InitStructure.CAN_RFLM=DISABLE;
	CAN_InitStructure.CAN_TXFP=DISABLE;
	CAN_InitStructure.CAN_Mode=CAN_Mode_Normal;
	CAN_InitStructure.CAN_SJW=can_baud_sjw[ cbaudidx ];
	CAN_InitStructure.CAN_BS1=can_baud_bs1[ cbaudidx ];
	CAN_InitStructure.CAN_BS2=can_baud_bs2[ cbaudidx ];
	CAN_InitStructure.CAN_Prescaler=can_baud_pre[ cbaudidx ];
	CAN_Init( CAN1, &CAN_InitStructure );

	/* CAN filter init */
	CAN_FilterInitStructure.CAN_FilterNumber=0;
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE;
	CAN_FilterInit(&CAN_FilterInitStructure);

	return can_baud_rate[ cbaudidx ];
}

void platform_can_off( unsigned id )
{
	/* Deinitialize CAN Peripheral */
	CAN_DeInit( CAN1 );
}

/*
   u32 platform_can_op( unsigned id, int op, u32 data )
   {
   u32 res = 0;
   TIM_TypeDef *ptimer = timer[ id ];
   volatile unsigned dummy;

   data = data;
   switch( op )
   {
   case PLATFORM_TIMER_OP_READ:
   res = TIM_GetCounter( ptimer );
   break;
   }
   return res;
   }
   */

void platform_can_interrupt( unsigned id, u8 state) {
	if(state)
		CAN_ITConfig(CAN1, CAN_IT_FMP0, ENABLE);
	else
		CAN_ITConfig(CAN1, CAN_IT_FMP0, DISABLE);
}

void platform_can_filter( unsigned id, u8 num, u8 enable, u32 filterId, u32 mask) {
	CAN_FilterInitTypeDef  CAN_FilterInitStructure;

	CAN_FilterInitStructure.CAN_FilterNumber=num;
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask;
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit;
		CAN_FilterInitStructure.CAN_FilterIdHigh=(filterId & 0xffff0000) >> 16;
		CAN_FilterInitStructure.CAN_FilterIdLow=filterId & 0xffff;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=(mask & 0xffff0000) >> 16;
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=mask & 0xffff;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_FIFO0;
		CAN_FilterInitStructure.CAN_FilterActivation=enable;
		CAN_FilterInit(&CAN_FilterInitStructure);
}

int platform_can_send( unsigned id, u32 canid, u8 idtype, u8 len, const u8 *data )
{
	CanTxMsg TxMessage;
	const char *s = ( char * )data;
	char *d;

	switch( idtype )
	{
		case ELUA_CAN_ID_STD:
			TxMessage.IDE = CAN_ID_STD;
			TxMessage.StdId = canid;
			break;
		case ELUA_CAN_ID_EXT:
			TxMessage.IDE = CAN_ID_EXT;
			TxMessage.ExtId = canid;
			break;
	}

	TxMessage.RTR=CAN_RTR_DATA;
	TxMessage.DLC=len;

	d = ( char * )TxMessage.Data;
	DUFF_DEVICE_8( len,  *d++ = *s++ );

	if( CAN_Transmit( CAN1, &TxMessage ) == CAN_NO_MB )
		return PLATFORM_ERR;

	return PLATFORM_OK;
}

int platform_can_overrun( unsigned id )
{
	return CAN_GetFlagStatus(CAN1, CAN_IT_FOV0);

}

int platform_can_message_pending( unsigned id )
{
	return CAN_GetFlagStatus(CAN1, CAN_IT_FMP0);
}

int platform_can_fifo_full( unsigned id )
{
	return CAN_GetFlagStatus(CAN1, CAN_IT_FF0);
}

int platform_can_recv( unsigned id, u32 *canid, u8 *idtype, u8 *len, u8 *data )
{
	CanRxMsg RxMessage;
	const char *s;
	char *d;

	if( CAN_MessagePending( CAN1, CAN_FIFO0 ) > 0 )
	{
		CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);

		if( RxMessage.IDE == CAN_ID_STD )
		{
			*canid = ( u32 )RxMessage.StdId;
			*idtype = ELUA_CAN_ID_STD;
		}
		else
		{
			*canid = ( u32 )RxMessage.ExtId;
			*idtype = ELUA_CAN_ID_EXT;
		}

		*len = RxMessage.DLC;

		s = ( const char * )RxMessage.Data;
		d = ( char* )data;
		DUFF_DEVICE_8( RxMessage.DLC,  *d++ = *s++ );
		return PLATFORM_OK;
	}
	else
		return PLATFORM_UNDERFLOW;
}


// ****************************************************************************
// UART
// TODO: Support timeouts.

// All possible STM32 uarts defs
USART_TypeDef *const stm32_usart[] =          { USART1, USART2, USART3, UART4, UART5 };
static GPIO_TypeDef *const usart_gpio_rx_port[] = { GPIOA, GPIOA, GPIOB, GPIOC, GPIOD };
static GPIO_TypeDef *const usart_gpio_tx_port[] = { GPIOA, GPIOA, GPIOB, GPIOC, GPIOC };
static const u16 usart_gpio_rx_pin[] = { GPIO_Pin_10, GPIO_Pin_3, GPIO_Pin_11, GPIO_Pin_11, GPIO_Pin_2 };
static const u16 usart_gpio_tx_pin[] = { GPIO_Pin_9, GPIO_Pin_2, GPIO_Pin_10, GPIO_Pin_10, GPIO_Pin_12 };
static GPIO_TypeDef *const usart_gpio_hwflow_port[] = { GPIOA, GPIOA, GPIOB };
static const u16 usart_gpio_cts_pin[] = { GPIO_Pin_11, GPIO_Pin_0, GPIO_Pin_13 };
static const u16 usart_gpio_rts_pin[] = { GPIO_Pin_12, GPIO_Pin_1, GPIO_Pin_14 };

static void usart_init(u32 id, USART_InitTypeDef * initVals)
{
	/* Configure USART IO */
	GPIO_InitTypeDef GPIO_InitStructure;

	/* Configure USART Tx Pin as alternate function push-pull */
	GPIO_InitStructure.GPIO_Pin = usart_gpio_tx_pin[id];
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_Init(usart_gpio_tx_port[id], &GPIO_InitStructure);

	/* Configure USART Rx Pin as input floating */
	GPIO_InitStructure.GPIO_Pin = usart_gpio_rx_pin[id];
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_Init(usart_gpio_rx_port[id], &GPIO_InitStructure);

	/* Configure USART */
	USART_Init(stm32_usart[id], initVals);

	/* Enable USART */
	USART_Cmd(stm32_usart[id], ENABLE);
}

static void uarts_init()
{
	// Enable clocks.
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4, ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5, ENABLE);
}

u32 platform_uart_setup( unsigned id, u32 baud, int databits, int parity, int stopbits )
{
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = baud;

	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

	switch( databits )
	{
		case 5:
		case 6:
		case 7:
		case 8:
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			break;
		case 9:
			USART_InitStructure.USART_WordLength = USART_WordLength_9b;
			break;
		default:
			USART_InitStructure.USART_WordLength = USART_WordLength_8b;
			break;
	}

	switch (stopbits)
	{
		case PLATFORM_UART_STOPBITS_1:
			USART_InitStructure.USART_StopBits = USART_StopBits_1;
			break;
		case PLATFORM_UART_STOPBITS_2:
			USART_InitStructure.USART_StopBits = USART_StopBits_2;
			break;
		default:
			USART_InitStructure.USART_StopBits = USART_StopBits_2;
			break;
	}

	switch (parity)
	{
		case PLATFORM_UART_PARITY_EVEN:
			USART_InitStructure.USART_Parity = USART_Parity_Even;
			break;
		case PLATFORM_UART_PARITY_ODD:
			USART_InitStructure.USART_Parity = USART_Parity_Odd;
			break;
		default:
			USART_InitStructure.USART_Parity = USART_Parity_No;
			break;
	}

	usart_init(id, &USART_InitStructure);

	return TRUE;
}

void platform_s_uart_send( unsigned id, u8 data )
{
	while(USART_GetFlagStatus(stm32_usart[id], USART_FLAG_TXE) == RESET)
	{
	}
	USART_SendData(stm32_usart[id], data);
}

int platform_s_uart_recv( unsigned id, timer_data_type timeout )
{
	if( timeout == 0 )
	{
		if (USART_GetFlagStatus(stm32_usart[id], USART_FLAG_RXNE) == RESET)
			return -1;
		else
			return USART_ReceiveData(stm32_usart[id]);
	}
	// Receive char blocking
	while(USART_GetFlagStatus(stm32_usart[id], USART_FLAG_RXNE) == RESET);
	return USART_ReceiveData(stm32_usart[id]);
}

int platform_s_uart_set_flow_control( unsigned id, int type )
{
	USART_TypeDef *usart = stm32_usart[ id ]; 
	int temp = 0;
	GPIO_InitTypeDef GPIO_InitStructure;

	if( id >= 3 ) // on STM32 only USART1 through USART3 have hardware flow control ([TODO] but only on high density devices?)
		return PLATFORM_ERR;

	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;

	if( type == PLATFORM_UART_FLOW_NONE )
	{
		usart->CR3 &= ~USART_HardwareFlowControl_RTS_CTS;
		GPIO_InitStructure.GPIO_Pin = usart_gpio_rts_pin[ id ] | usart_gpio_cts_pin[ id ];
		GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );      
		return PLATFORM_OK;
	}
	if( type & PLATFORM_UART_FLOW_CTS )
	{
		temp |= USART_HardwareFlowControl_CTS;
		GPIO_InitStructure.GPIO_Pin = usart_gpio_cts_pin[ id ];
		GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
	}
	if( type & PLATFORM_UART_FLOW_RTS )
	{
		temp |= USART_HardwareFlowControl_RTS;
		GPIO_InitStructure.GPIO_Pin = usart_gpio_rts_pin[ id ];
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
		GPIO_Init( usart_gpio_hwflow_port[ id ], &GPIO_InitStructure );
	}
	usart->CR3 |= temp;
	return PLATFORM_OK;
}


// ****************************************************************************
// Timers

u8 stm32_timer_int_periodic_flag[ NUM_PHYS_TIMER ];

// We leave out TIM6/TIM for now, as they are dedicated
TIM_TypeDef * const timer[] = { TIM1, TIM2, TIM3, TIM4, TIM5 };
#define TIM_GET_PRESCALE( id ) ( ( id ) == 0 || ( id ) == 5 ? ( PCLK2_DIV ) : ( PCLK1_DIV ) )
#define TIM_GET_BASE_CLK( id ) ( TIM_GET_PRESCALE( id ) == 1 ? ( HCLK / TIM_GET_PRESCALE( id ) ) : ( HCLK / ( TIM_GET_PRESCALE( id ) / 2 ) ) )
#define TIM_STARTUP_CLOCK       50000

static u32 platform_timer_set_clock( unsigned id, u32 clock );

void SysTick_Handler( void )
{
	// Handle virtual timers
	cmn_virtual_timer_cb();

	// Handle system timer call
	cmn_systimer_periodic();
}

static void timers_init()
{
	unsigned i;

	// Enable clocks.
	RCC_APB2PeriphClockCmd( RCC_APB2Periph_TIM1, ENABLE );  
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM2, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM3, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM4, ENABLE );
	RCC_APB1PeriphClockCmd( RCC_APB1Periph_TIM5, ENABLE );

	// Configure timers
	for( i = 0; i < NUM_TIMER; i ++ )
		platform_timer_set_clock( i, TIM_STARTUP_CLOCK );
}

static u32 platform_timer_get_clock( unsigned id )
{
	TIM_TypeDef* ptimer = timer[ id ];

	return TIM_GET_BASE_CLK( id ) / ( TIM_GetPrescaler( ptimer ) + 1 );
}

static u32 platform_timer_set_clock( unsigned id, u32 clock )
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	TIM_TypeDef *ptimer = timer[ id ];
	u16 pre = ( TIM_GET_BASE_CLK( id ) / clock ) - 1;

	TIM_TimeBaseStructure.TIM_Period = 0xFFFF;
	TIM_TimeBaseStructure.TIM_Prescaler = pre;
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
	TIM_TimeBaseStructure.TIM_RepetitionCounter = 0x0000;
	TIM_TimeBaseInit( timer[ id ], &TIM_TimeBaseStructure );
	TIM_Cmd( ptimer, ENABLE );

	return TIM_GET_BASE_CLK( id ) / ( pre + 1 );
}

void platform_s_timer_delay( unsigned id, timer_data_type delay_us )
{
	TIM_TypeDef *ptimer = timer[ id ];
	volatile unsigned dummy;
	timer_data_type final;

	final = ( ( u64 )delay_us * platform_timer_get_clock( id ) ) / 1000000;
	TIM_SetCounter( ptimer, 0 );
	for( dummy = 0; dummy < 200; dummy ++ );
	while( TIM_GetCounter( ptimer ) < final );
}

timer_data_type platform_s_timer_op( unsigned id, int op, timer_data_type data )
{
	u32 res = 0;
	TIM_TypeDef *ptimer = timer[ id ];
	volatile unsigned dummy;

	data = data;
	switch( op )
	{
		case PLATFORM_TIMER_OP_START:
			TIM_SetCounter( ptimer, 0 );
			for( dummy = 0; dummy < 200; dummy ++ );
			break;

		case PLATFORM_TIMER_OP_READ:
			res = TIM_GetCounter( ptimer );
			break;

		case PLATFORM_TIMER_OP_SET_CLOCK:
			res = platform_timer_set_clock( id, data );
			break;

		case PLATFORM_TIMER_OP_GET_CLOCK:
			res = platform_timer_get_clock( id );
			break;

		case PLATFORM_TIMER_OP_GET_MAX_CNT:
			res = 0xFFFF;
			break;
	}
	return res;
}

int platform_s_timer_set_match_int( unsigned id, timer_data_type period_us, int type )
{
	TIM_TypeDef* base = ( TIM_TypeDef* )timer[ id ];
	u32 period, prescaler, freq;
	u64 final;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	if( period_us == 0 )
	{
		TIM_ITConfig( base, TIM_IT_CC1, DISABLE );
		base->CR1 = 0; // Why are we doing this?
		base->CR2 = 0;
		return PLATFORM_TIMER_INT_OK;
	}

	period = ( ( u64 )TIM_GET_BASE_CLK( id ) * period_us ) / 1000000;

	prescaler = ( period / 0x10000 ) + 1;
	period /= prescaler;

	platform_timer_set_clock( id, TIM_GET_BASE_CLK( id  ) / prescaler );
	freq = platform_timer_get_clock( id );
	final = ( ( u64 )period_us * freq ) / 1000000;

	if( final == 0 )
		return PLATFORM_TIMER_INT_TOO_SHORT;
	if( final > 0xFFFF )
		return PLATFORM_TIMER_INT_TOO_LONG;

	TIM_Cmd( base, DISABLE );

	TIM_OCStructInit( &TIM_OCInitStructure );
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_Timing;
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
	TIM_OCInitStructure.TIM_Pulse = ( u16 )final;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
	TIM_OC1Init( base, &TIM_OCInitStructure );

	// Patch timer configuration to reload when period is reached
	TIM_SetAutoreload( base, ( u16 )final );

	TIM_OC1PreloadConfig( base, TIM_OCPreload_Enable );

	stm32_timer_int_periodic_flag[ id ] = type;

	TIM_SetCounter( base, 0 );
	TIM_Cmd( base, ENABLE );
	//TIM_ITConfig( base, TIM_IT_CC1, ENABLE );

	return PLATFORM_TIMER_INT_OK;
}

u64 platform_timer_sys_raw_read()
{
	return SysTick->LOAD - SysTick->VAL;
}

void platform_timer_sys_disable_int()
{
	SysTick->CTRL &= ~( 1 << SYSTICK_TICKINT );
}

void platform_timer_sys_enable_int()
{
	SysTick->CTRL |= 1 << SYSTICK_TICKINT;
}

timer_data_type platform_timer_read_sys()
{
	return cmn_systimer_get();
}

// *****************************************************************************
// CPU specific functions

u32 platform_s_cpu_get_frequency()
{
	return HCLK;
}


#ifdef BUILD_WOFS
u32 platform_s_flash_write( const void *from, u32 toaddr, u32 size )
{
  u32 ssize = 0;
  const u16 *psrc = ( const u16* )from;
  FLASH_Status flstat;

  while( ssize < size )
  {
    if( ( flstat = FLASH_ProgramHalfWord( toaddr, *psrc ++ ) ) != FLASH_COMPLETE )
    {
      printf( "ERROR in platform_s_flash_write: stat=%d at %08X\n", ( int )flstat, ( unsigned )toaddr );
      break;
    }
    toaddr += 2;
    ssize += 2;
  }
  return ssize;
}

int platform_flash_erase_sector( u32 sector_id )
{
  return FLASH_ErasePage( sector_id * INTERNAL_FLASH_SECTOR_SIZE + INTERNAL_FLASH_START_ADDRESS ) == FLASH_COMPLETE ? PLATFORM_OK : PLATFORM_ERR;
}

#endif // #ifdef BUILD_WOFS

