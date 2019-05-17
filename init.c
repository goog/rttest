#include <stdio.h>
#include <bsp.h>
#include <bsp/irq.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <sys/time.h>
#include <errno.h>

#include "oled.h"
#include "led.h"
#include "pins.h"
#include "time_rtc.h"
#include "adc.h"
#include "I2C_Software_Master.h"
#include "spi.h"
#include "w25qxx.h"

#define MPU6050_ADDRESS_AD0_LOW     0x68 // address pin low (GND), default for InvenSense evaluation board
//#define MPU6050_ADDRESS_AD0_HIGH    0x69 // address pin high (VCC)
#define MPU6050_DEFAULT_ADDRESS (MPU6050_ADDRESS_AD0_LOW<<1)

// Minor number
#define USART6_TLB_INDEX 3

typedef enum
{
	
	GPRS_Init=0,               
	GPRS_IdlePWRKEYOFF,            
	GPRS_IdleDelayPowerOFF,        
	GPRS_IdlePowerOFF,            
	GPRS_Idle,                    
	
	
	GPRS_PowerInit,                
	GPRS_InitOFF,                  
	GPRS_PWRKEYOFF,                
	GPRS_DelayPowerOFF,           
	GPRS_PowerOFF,                
	GPRS_DelayRestart,            
	GPRS_PowerUp,                  
	GPRS_PWRKEYON,                 
	GPRS_DelayInit,                
    GPRS_NET_ATTACHED,
	
	GPRS_GET_CIMI,
	GPRS_SET_APN,		
	GPRS_CHECK_CSQ,	
	
	GPRS_CHECK_OPERATOR,
	
	GPRS_CHECK_CPSI,
	GPRS_SET_PDP,		
	GPRS_GET_IPADDR,
	
	GPRS_WAIT_CIPOPEN,		
	GPRS_SET_CIPOPEN,		
	GPRS_SET_CIPHEAD,	
	GPRS_WAIT_NETCLOSE,		
	
	
	GPRS_SET_NETCLOSE,			
	GPRS_CONNECT     
	
}
GPRSStatus;


typedef unsigned char uint8;


//
extern unsigned char g_GPRSSendBuffer[500];        
extern int Send_len;


volatile GPRSStatus netstat = 0;
volatile uint8 timer_timeout = 0;
volatile uint8 f_key_pressed = 0;

volatile uint8 f_wait_ipsend_timeout = 0;
rtems_id Timer1;
rtems_id timer_uart;

// usart 2
#define UART2_BUF_LEN  400
volatile uint8_t uart2_buf[UART2_BUF_LEN] = {0};
volatile int uart2_index = 0;
volatile uint8_t uart2_rev_flag = 0;


char gps_buffer[220] = {0};
// usart 1 

#define USART_BUF_LEN  300
volatile uint8 uart1_buf[USART_BUF_LEN] = {0};
volatile int uart1_index = 0;
volatile uint8 uart1_rev_flag = 0;


// usart 6
volatile uint8 uart6_buf[USART_BUF_LEN] = {0};
volatile int uart6_index = 0;
volatile uint8 uart6_rev_flag = 0;
volatile uint8 uart6_send_sign_flag = 0;


static uint8_t mnc = 0;
static uint32_t cpsi_lac = 0;
static uint32_t cpsi_cid = 0;

//io define
// pd2
stm32f4_gpio_config pwrkeyconfig =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(3, 2),
	.pin_last = STM32F4_GPIO_PIN(3, 2),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 0,
	.af = 0
   }
};


// pc5
stm32f4_gpio_config pwrenconfig =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(2, 5),
	.pin_last = STM32F4_GPIO_PIN(2, 5),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};



//pc3
stm32f4_gpio_config gps_enconfig =
{
	.fields={
        .pin_first = STM32F4_GPIO_PIN(2, 3),
	    .pin_last = STM32F4_GPIO_PIN(2, 3),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};


//pc8
stm32f4_gpio_config stm32_io_dtr =
{
	.fields={
        .pin_first = STM32F4_GPIO_PIN(2, 8),
	    .pin_last = STM32F4_GPIO_PIN(2, 8),
	    .mode = STM32F4_GPIO_MODE_OUTPUT,
	    .otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	    .ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	    .pupd = STM32F4_GPIO_NO_PULL,
	    .output = 0,
	    .af = 0
    }
};


// pc4
stm32f4_gpio_config adc_pin =
{
	.fields={
        .pin_first = STM32F4_GPIO_PIN(2, 4),
	    .pin_last = STM32F4_GPIO_PIN(2, 4),
	.mode = STM32F4_GPIO_MODE_ANALOG,
	//.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 0,
	.af = 0
   }
};

//pa 0
stm32f4_gpio_config adc_enconfig =
{
	.fields={
        .pin_first = STM32F4_GPIO_PIN(0, 0),
	    .pin_last = STM32F4_GPIO_PIN(0, 0),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};
//pb8
stm32f4_gpio_config iic_scl =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(1, 8),
	.pin_last = STM32F4_GPIO_PIN(1, 8),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};


stm32f4_gpio_config iic_scl_dummy =
{
	.fields={
	.pin_first = STM32F4_GPIO_PIN(1, 8),
	.pin_last = STM32F4_GPIO_PIN(1, 8),
	.mode = STM32F4_GPIO_MODE_ANALOG,
	//.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 0,
	.af = 0
   }
};

stm32f4_gpio_config iic_sda_dummy =
{
	.fields={
	.pin_first = STM32F4_GPIO_PIN(1, 9),
	.pin_last = STM32F4_GPIO_PIN(1, 9),
	.mode = STM32F4_GPIO_MODE_ANALOG,
	//.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 0,
	.af = 0
   }
};



//pb9
stm32f4_gpio_config iic_sda =
{
	.fields={
		.pin_first = STM32F4_GPIO_PIN(1, 9),
	.pin_last = STM32F4_GPIO_PIN(1, 9),
	.mode = STM32F4_GPIO_MODE_OUTPUT,
	.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	.ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	.pupd = STM32F4_GPIO_NO_PULL,
	.output = 1,
	.af = 0
   }
};

stm32f4_gpio_config io_key2 =
{
	.fields={
	    .pin_first = STM32F4_GPIO_PIN(4, 2),
        .pin_last = STM32F4_GPIO_PIN(4, 2),
        .mode = STM32F4_GPIO_MODE_INPUT,
	    //.otype = STM32F4_GPIO_OTYPE_PUSH_PULL,
	    .ospeed = STM32F4_GPIO_OSPEED_2_MHZ,
	    .pupd = STM32F4_GPIO_PULL_UP,
        .output = 0,
        .af = 0
    }
};



#if 0
#define CR_DS_MASK               ((uint32_t)0xFFFFF3FC)
#define CR_PLS_MASK              ((uint32_t)0xFFFFFF1F)
#define CR_VOS_MASK              ((uint32_t)0xFFFF3FFF)
#define SCB_SCR_SLEEPDEEP_Pos               2                                             /*!< SCB SCR: SLEEPDEEP Position */
#define SCB_SCR_SLEEPDEEP_Msk              (1UL << SCB_SCR_SLEEPDEEP_Pos) 

void PWR_EnterSTOPMode(uint32_t PWR_Regulator, uint8_t PWR_STOPEntry)
{
  uint32_t tmpreg = 0;
  
  /* Check the parameters */
  //assert_param(IS_PWR_REGULATOR(PWR_Regulator));
  //assert_param(IS_PWR_STOP_ENTRY(PWR_STOPEntry));
    
  /* Select the regulator state in STOP mode ---------------------------------*/
    uint32_t PWR_CR = *(volatile uint32_t *)0x40007000;
    uint32_t SCB_SCR = *(volatile uint32_t *)0xE000ED10;
    tmpreg = PWR_CR;
  /* Clear PDDS and LPDS bits */
    tmpreg &= CR_DS_MASK;
  
    /* Set LPDS, MRLVDS and LPLVDS bits according to PWR_Regulator value */
    tmpreg |= PWR_Regulator;
  
  /* Store the new value */
    PWR_CR = tmpreg;
  
  /* Set SLEEPDEEP bit of Cortex System Control Register */
    SCB_SCR |= SCB_SCR_SLEEPDEEP_Msk;
  
  /* Select STOP mode entry --------------------------------------------------*/
  //if(PWR_STOPEntry == PWR_STOPEntry_WFI)
  if(PWR_STOPEntry == 1)
  {   
    /* Request Wait For Interrupt */
    //__WFI();
    //__wfi();
    __asm__ volatile ("wfi");
  }
  else
  {
    /* Request Wait For Event */
    //__WFE();
    __asm__ volatile ("wfe");
  }
  /* Reset SLEEPDEEP bit of Cortex System Control Register */
    SCB_SCR &= (uint32_t)~((uint32_t)SCB_SCR_SLEEPDEEP_Msk);  
}


#endif

rtems_timer_service_routine Timer_Routine( rtems_id id, void *ignored)
{
    //printf("time out \n");
    
    int j = 100000;
    timer_timeout = 1;
    //LED_ON();
    #if 0
    (void) rtems_timer_fire_after(
    Timer1,
    5 * rtems_clock_get_ticks_per_second(),
    Timer_Routine,
    NULL
    );
    #endif

}

rtems_timer_service_routine timer_uart_routine(rtems_id id, void *ignored)
{
    //printf("time out \n");
    
    //int j = 100000;
    //timer_timeout = 1;
    f_wait_ipsend_timeout = 1;

}



#if 0
int ux_uart_init(char *dev_name)
{
	printf("ux_uart_init %s begin\n", dev_name);
    int fd = open(dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    //printf("\nOpened COM3, fd=%d \n", fd);
    //LED_ON();
    if(fd < 0)
    {
	    //printf("open error\n");
	    LED_ON();
	    return -1;	
	}
      

  printf("test point 1\n");
  struct termios options;
  tcgetattr(fd, &options);
  bzero(&options, sizeof(options));
	/* setting the baud rate */
  cfsetispeed(&options, B115200);
  cfsetospeed(&options, B115200);

	
  options.c_cflag |= (CLOCAL | CREAD);

  options.c_cflag &= ~PARENB;
  options.c_cflag &= ~CSTOPB;
  options.c_cflag &= ~CSIZE;
  options.c_cflag |= CS8;

  options.c_iflag = IGNPAR| ICRNL;
  options.c_oflag = 0;
  options.c_lflag = ICANON;

  options.c_cc[VEOF] = 4;
  options.c_cc[VMIN] = 1;
  options.c_cc[VTIME] = 0;

  //tcflush(fd, TCIFLUSH);
  tcsetattr(fd, TCSANOW, &options);  
      
  if(0 == isatty(fd))  
  {  
      printf("input is not a terminal device\n");  
      return -1;  
  }
  
    return fd;	
}
#endif

rtems_isr EINT2_handler(rtems_vector_number vector)
{
	//rtems_event_send (taskid, RTEMS_EVENT_1);
    //GPBDAT ^= 1<<7;
    //SRCPND = 1<<vector;
    //INTPND = 1<<vector;
    
    static int cnt = 0;
    uint32_t EXIT_PR = *(uint32_t*)0x40013C14;
    
    if(EXIT_PR & 0x04)
    {
	    int pin = STM32F4_GPIO_PIN(4,2);
		bool val;
		val = stm32f4_gpio_get_input(pin);
		
		//rtems_task_wake_after(1);
        int j = 20000;
        for(; j > 0; j--) j = j;
		if(val == stm32f4_gpio_get_input(pin))
			f_key_pressed = 1;
		
		(*(uint32_t*)0x40013C14) = (1<<2);	
	}
    
}


#pragma GCC push_options
#pragma GCC optimize ("O0")
rtems_isr rtc_wakeup_handler(rtems_vector_number vector)
{

    #if 0
    volatile uint32_t STM32F4_RTC_ISR = (*(volatile uint32_t *)(0x4000280C));
    printk("before clr isr %08x\n", STM32F4_RTC_ISR);
    // clear 10bit
    if(STM32F4_RTC_ISR & (1<<10))
    {
        //printk("wak\n");
        STM32F4_RTC_ISR &= ~(1<<10);
        STM32F4_RTC_ISR &= 0xfffffbff;
        printk("after clr isr %08x\n", STM32F4_RTC_ISR);
    }
    
    volatile uint32_t EXIT_PR = *(volatile uint32_t*)0x40013C14;
    printk("before clr exit_pr %08x\n", EXIT_PR);
    if(EXIT_PR & (1<<22))
    {
        EXIT_PR = 1<<22;
        //EXIT_PR |= 1<<22;
        //EXIT_PR |= 1<<22;
        //int i;
        //for(i = 0; i < 1000; i++) i = i + 1 - 1;
        printk("the test i \n");
        
    }
    #endif
    //printk(" %08x\n", EXIT_PR);
}
#pragma GCC pop_options


// usart 1 read freezer
//0x40011000
#define USART1_SR (*(volatile uint32_t *)0x40011000)
#define USART1_DR (*(volatile uint32_t *)0x40011004)

#pragma GCC push_options
#pragma GCC optimize ("O0")
rtems_isr USART1_handler(rtems_vector_number vector)
{

    uint32_t temp = temp;
    
    if((USART1_SR) & (1<<5))
    {
	    
	    //printk("into int\n");
        if(uart1_index < USART_BUF_LEN)
	        uart1_buf[uart1_index++] = (uint8_t)(USART1_DR)&0xff;
	    else
            temp = USART1_DR;
				    	
	}
    
    if(USART1_SR & (1<<4))  // idle
    {
	    //printk("idl \n");
        
        temp = USART1_SR;
	    temp = USART1_DR;
        
	    
        uart1_rev_flag = 1;   
	}
    
	if(USART1_SR & (1<<3))
	{
        
        temp = USART1_SR;
	    temp = USART1_DR;
    }
    
    
    
}
#pragma GCC pop_options



uint32_t *USART2_SR = (uint32_t *)0x40004400;
uint32_t *USART2_DR = (uint32_t *)0x40004404;

#pragma GCC push_options
#pragma GCC optimize ("O0")
rtems_isr USART2_handler(rtems_vector_number vector)
{

    uint32_t temp = temp;
    //uint32_t SR = *USART2_SR;
    if((*USART2_SR) & (1<<5))
    {
	    
	    //printk("into int\n");
        if(uart2_index < UART2_BUF_LEN)
	        uart2_buf[uart2_index++] = (uint8_t)(*USART2_DR)&0xff;
	    else
            temp = *USART2_DR;
		//printk("%02X  ", uart2_buf[uart2_index - 1]);
		
		
		
		    	
	}
    
    if((*USART2_SR) & (1<<4))  // idle
    {
	    //printk("idle ");
        
        temp = (*USART2_SR);
	    temp = (*USART2_DR);
        //temp = temp;
	    
        uart2_rev_flag = 1;	
        //printk("fla %08X", *USART2_SR);
        
	}
    
	if((*USART2_SR) & (1<<3))
	{
        //int temp = (*USART2_DR)&0xff;
        temp = *USART2_SR;
	    temp = *USART2_DR;
    }
    
    //temp = (*USART2_SR);
    //temp = (*USART2_DR);
    
}
#pragma GCC pop_options


#if 0
uint32_t *USART3_SR = (uint32_t *)0x40004800;
uint32_t *USART3_DR = (uint32_t *)0x40004804;

#pragma GCC push_options
#pragma GCC optimize ("O0")
rtems_isr USART3_handler(rtems_vector_number vector)
{

    uint32_t temp = temp;
    //uint32_t SR = *USART2_SR;
    if((*USART3_SR) & (1<<5))
    {
	    
	    //printk("into int\n");
        if(uart3_index < USART_BUF_LEN)
	        uart3_buf[uart3_index++] = (uint8_t)(*USART3_DR)&0xff;
	    else
            temp = *USART3_DR;
		//printk("%02X  ", uart2_buf[uart2_index - 1]);
		
		
		
		    	
	}
    
    if((*USART3_SR) & (1<<4))  // idle
    {
	    //printk("idle ");
        
        temp = (*USART3_SR);
	    temp = (*USART3_DR);
        
	    
        uart3_rev_flag = 1;	
        
        
	}
    
	if((*USART3_SR) & (1<<3))
	{
        //int temp = (*USART2_DR)&0xff;
        temp = *USART3_SR;
	    temp = *USART3_DR;
    }
    
    
    
}
#pragma GCC pop_options
#endif

// usart 6

#define USART6_SR (*(volatile uint32_t *)0x40011400)
#define USART6_DR (*(volatile uint32_t *)0x40011404)

//#pragma GCC push_options
//#pragma GCC optimize ("O0")
rtems_isr USART6_handler(rtems_vector_number vector)
{

    uint32_t temp = temp;
    
    if((USART6_SR) & (1<<5))
    {
	    
	    //printk("into int\n");
        if(uart6_index < USART_BUF_LEN)
	        uart6_buf[uart6_index++] = (uint8_t)(USART6_DR)&0xff;
	    else
            temp = USART6_DR;
		//printk("%02X  ", uart2_buf[uart2_index - 1]);
		
		
		
		    	
	}
    
    if(USART6_SR & (1<<4))  // idle
    {
	    //printk("idl \n");
        
        temp = USART6_SR;
	    temp = USART6_DR;
        
	    
        uart6_rev_flag = 1;   
	}
    
	if((USART6_SR) & (1<<3))
	{
        
        temp = USART6_SR;
	    temp = USART6_DR;
    }
    
    
    
}
//#pragma GCC pop_options


static void SendCmd(char *cmd)
{

    //TM_USART_Puts(USART1, cmd);
    //TM_USART_Puts(USART1, "\r\n");
}

/*
 * Send AT command to SIMCOM module and check response immediately.
 */
static int SendCmd_Check(char *cmd, char *check)
{
    int result;
    char recv_str[32];

    #if 0
    TM_USART_Puts(USART1, cmd);
    TM_USART_Puts(USART1, "\r\n");

    if(check) {
        RecvResponse(recv_str);

        if(!strstr(recv_str, check)) {
            result = 0;
        }
    }
    #endif

    return result;
}


void keys_init()
{
	stm32f4_gpio_set_config(&io_key2);
	
    //0x4002 3800
	(*(uint32_t*)0x40023844) |= 1<<14;
    
	// it group
	
	(*(uint32_t*)0xE000ED0C) = 0x05FA0000 | 0x400;
	
	//(*(uint32_t*)0x40023830) |= 1<<4; //rcc
	// 0x4002 1000
    (*(uint32_t*)0x4002100c) |= 0x01;  // pull up
	// pe2 exit
    (*(uint32_t*)0x40013808) |= 0b010000000000;
	
    // open it
	(*(uint32_t*)0x40013C00) |= (1<<2);
	// EXTI_FTSR fall
	(*(uint32_t*)0x40013C0C) |= (1<<2);
	
	// ip
	(*(uint8_t*)0xE000E408) = 0xe0;
	
	
	
	
    // nvic enable interrupter number
    // 0xE000E100
    (*(uint32_t*)0xE000E100) |= (1<<8);	
}


void usart2_rxbuf_clear()
{
    memset(uart2_buf, 0, sizeof(uart2_buf));
    
}


int usart2_read(uint8_t *buffer, int len)
{
    if(buffer == NULL || len <= 0)
        return 0;
    
    int read_len = 0;
    
    if(uart2_rev_flag)
    {
        printf("usart2 read buf total len %d\n", uart2_index);
        printf("usart2 buf current len %d\n", len);
        //usart_write_support_polled(minor,uart2_buf,uart2_index);
        //uart2_index -= 2; // remove /r/n
        if(uart2_index < len)
            memcpy(buffer, uart2_buf, uart2_index);
        else
            memcpy(buffer, uart2_buf, len - 1);

        memset(uart2_buf, 0, sizeof(uart2_buf));
        read_len = uart2_index;
        uart2_index = 0;
        uart2_rev_flag = 0;
        
        return read_len;
    }
    
    return 0;
}


void usart2_write(const char *buf, int len)
{
    
    usart_write_support_polled(1, buf, len);
}


int usart6_read(char *buffer, int len)
{
    if(buffer == NULL || len <= 0)
        return 0;
    
    int read_len = 0;
    
    if(uart6_rev_flag)
    {
        printf("usart6 read buf len %d\n", uart6_index);
        //usart_write_support_polled(minor,uart2_buf,uart2_index);
        //uart2_index -= 2; // remove /r/n
        if(uart6_index < len)
            memcpy(buffer, uart6_buf, uart6_index);
        else
            memcpy(buffer, uart6_buf, len - 1);

        memset(uart6_buf, 0, sizeof(uart6_buf));
        read_len = uart6_index;
        uart6_index = 0;
        uart6_rev_flag = 0;
        
        return read_len;
    }
    
    return 0;
}


void usart6_write(const char *buf, int len)
{
    
    usart_write_support_polled(3, buf, len);
}

// pack
void gprs_send(const char *buf, int len)
{
    
    usart6_write(buf, len);
}


void usart1_clear_buf()
{
    memset(uart1_buf, 0, sizeof(uart1_buf));
    uart1_index = 0;   
    
}


int usart1_read(uint8_t *buffer, int len)
{
    if(buffer == NULL || len <= 0)
        return 0;
    
    int read_len = 0;
    
    if(uart1_rev_flag)
    {
        printf("usart1 read buf len %d at %s\n", uart1_index, __FUNCTION__);
        
        
        if(uart1_index < len)
            memcpy(buffer, uart1_buf, uart1_index);
        else
            memcpy(buffer, uart1_buf, len - 1);

        memset(uart1_buf, 0, sizeof(uart1_buf));
        read_len = uart1_index;
        uart1_index = 0;
        uart1_rev_flag = 0;
        
        return read_len;
    }
    
    return 0;
}


void send_at_command(const char *buf)
{
    //const int minor = 3;
    usart_write_support_polled(USART6_TLB_INDEX, buf, strlen(buf));
}


void send_at_cmd(const char *buf, size_t len)
{
    usart_write_support_polled(USART6_TLB_INDEX, buf, len);
}



int extract_cpsi_info(const char *data)
{
    printf("%s begin\n", __FUNCTION__);

    char *token;
    int index = 0;
    char *pc = NULL;

    printf("data : %s\n", data);
    token = strtok(data, ",");
    while(token != NULL)
    {
        index++;
        if(index == 3)
        {
            pc = strchr(token, '-');
            if(pc)
            {
                mnc = (uint8_t) atoi(pc + 1);
            }
        }
        else if(index == 4)
        {
            cpsi_lac = strtol(token, NULL, 16);
        }
        else if(index == 5)
        {
            cpsi_cid = atoi(token);
        }

        token = strtok(NULL, ",");
    }
    

    printf("lac %08x cid %08x\n", cpsi_lac, cpsi_cid);
    printf("mnc %02x \n", mnc);
}

GPRSStatus net_register(int timeout)
{
    int ret = 0;
    
    netstat = 0;
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    // power on   
    // configure pin
    //PWR_KEY_INIT();
    //rtems_task_wake_after(1);
    //PWR_KEY_ON();
    //rtems_task_wake_after(0.5*rtems_clock_get_ticks_per_second());
    //PWR_KEY_OFF();
    // set 0
    
    //power up
    netstat = GPRS_PowerUp;
    //if(netstat == GPRS_Init)
    //{
    //    sleep(20);
    //}
    
    printf("begin to at command\n");
    if(netstat == GPRS_PowerUp)
    {
        printf("begin to at \n");

        #if 0
        snprintf(command, sizeof(command), "AT\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        printf("before read \n");
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cmd reply: %s\n", rx_buf);
        #endif

        snprintf(command, sizeof(command), "AT+IPR=?\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("IPR reply: %s\n", rx_buf);
        
        // baudrate
        //AT+IPR?
        snprintf(command, sizeof(command), "AT+IPR=921600\r\n");
        send_at_command(command);

        //
        
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("IPR reply: %s\n", rx_buf);


        usart_initialize_baud(3); // usart6
        sleep(2);
        
        printf("ATE0\r\n");
        snprintf(command, sizeof(command), "ATE0\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cmd reply: %s\n", rx_buf);


        
        


        // csclk SLEEP MODE
        snprintf(command, sizeof(command), "AT+CSCLK=1\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        //printf("before read \n");
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cmd reply: %s\n", rx_buf);
        
        snprintf(command, sizeof(command), "AT+CIMI\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cmd reply: %s\n", rx_buf);


        //AT+CSPISETCLK=?
        snprintf(command, sizeof(command), "AT+CSPISETCLK=?\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.5*rtems_clock_get_ticks_per_second());
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("spi reply: %s\n", rx_buf);
        
        
        
        snprintf(command, sizeof(command), "AT+CGDCONT=1,\"IP\",\"cmnet\"\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cmd reply: %s\n", rx_buf);
        // strstr and change status
        
        
        snprintf(command, sizeof(command), "AT+CSQ\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("csq reply: %s\n", rx_buf);
        
        
        // AT+CPSI?
        snprintf(command, sizeof(command), "AT+CPSI?\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cpsi reply: %s\n", rx_buf);
 
        // extract mnc lac cid
        extract_cpsi_info(rx_buf);
        
        // AT+COPS?
        snprintf(command, sizeof(command), "AT+COPS?\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("cops reply: %s\n", rx_buf);
        
        // check attached
        int i = 0;
        for(i = 0; i < timeout; i++)
        {
            snprintf(command, sizeof(command), "AT+CEREG?\r\n");
            send_at_command(command);
            rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
            memset(rx_buf, 0, rx_buf_len);
            ret = usart6_read(rx_buf, rx_buf_len);
            printf("cereg reply: %s\n", rx_buf);
            
            if(strstr(rx_buf, "+CEREG: 0,1"))
            {
                netstat = GPRS_NET_ATTACHED;
                break;
            }
            sleep(1);
        }
        
        
        
        
        
        
    }
    
    
    
    
    

    return 0;
}

int network_get_status()
{
    
    // 
    
    return 0;    
}



int gprs_get_imei(char *buf, int len)
{
    int ret = 0;
    char command[100] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    
    snprintf(command, sizeof(command), "AT+CGSN\r\n");
    send_at_command(command);
    rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());

    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("IMEI reply: %s\n", rx_buf);
    strncpy(buf, rx_buf+2, 15);

    int i = 0;
    for(i = 0; i < 15; i++)
        printf("%c", buf[i]);
    printf("\n");
    
    if(ret > 0) ret = 1;

    return ret;
}
// AT+CGSN



int gprs_get_ccid(char *buf, int len)
{

    int ret = 0;
    char command[100] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    usart6_read(rx_buf, rx_buf_len);  // clear rx buf
    snprintf(command, sizeof(command), "AT+CCID\r\n");
    send_at_command(command);
    rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());

    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("CCID reply:%s\n", rx_buf);

    if(ret > 28)
        strncpy(buf, rx_buf+9, 20);


    int i = 0;
    for(i = 0; i < 20; i++)
        printf("%c", buf[i]);
    printf("\n");
    
    if(ret > 0) ret = 1;

    return ret;
}

int gprs_is_registered()
{
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    
    snprintf(command, sizeof(command), "AT+CEREG?\r\n");
    send_at_command(command);
    rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());

    memset(rx_buf, 0, rx_buf_len);
    usart6_read(rx_buf, rx_buf_len);
    //printf("cereg reply: %s\n", rx_buf);
    
    if(strstr(rx_buf, "+CEREG: 0,1"))
    {
        //netstat = GPRS_NET_ATTACHED;
        //break;
        return 1;
    }    

    return 0;
}

// 4g send tcp data
int tcp_send_data(char *data, int len)
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    char command[200] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    snprintf(command, sizeof(command), "AT+CIPSEND=0,%d\r\n", len);
    printf("command %s\n", command);
    send_at_command(command);
    //rtems_task_wake_after(2*rtems_clock_get_ticks_per_second());
    
    
    printf("before timer starts \n");
    rtems_timer_fire_after(timer_uart,
                           10 * rtems_clock_get_ticks_per_second(),
                           timer_uart_routine, NULL
                          );
    
    // wait timeout or data
    while(f_wait_ipsend_timeout == 0 && uart6_rev_flag == 0);
    
    // reset timeout flag
    if(f_wait_ipsend_timeout) 
    {
        printf("wait ipsend timeout\n");
        f_wait_ipsend_timeout = 0;
        
        return -1;
    }
    // cancel timer
    rtems_timer_cancel(timer_uart);
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("%02x %02x\n", rx_buf[0], rx_buf[1]);
    printf("ipsend reply:%s\n", rx_buf);
    if(strstr(rx_buf, ">"))
    {
        printf("ready to send data\n");
        //snprintf(command, len + 2, "%s\r\n", data);
        //send_at_command(command);
        memset(command, 0, sizeof(command));
        memcpy(command, data, len);
        command[len] = '\r';
        command[len+1] = '\n';
        send_at_cmd(command, len + 2);  // dont depence string mode
        rtems_task_wake_after(3.5*rtems_clock_get_ticks_per_second());
    
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("send tcp data reply:%s\n", rx_buf);
        printf("%s function end\n", __FUNCTION__);
        return 0;
        
    }
    else
    {
        printf("ipsend no respond\n");

        // addd gprs cnt plus
        return -1;
    }
    
    
    
}






int tcp_send_1kdata()
{
    //printf("%s function begin\n", __FUNCTION__);
    // check reg status
    
    int ret;
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    int i;
    rtems_status_code rc;

    uint8_t data[1024] = {[0 ... 1023] = 0x3a};

    int len = 1000;

    char *ip = "monitor.iok.la";
        //uxdar.f3322.net
        //char *ip = "uxdar.f3322.net";
    int port = 18327;
    
    snprintf(command, sizeof(command), "AT+CIPSEND=0,%d,\"%s\",%d\r\n", len, ip, port);
    printf("command %s\n", command);
    send_at_command(command);
    uart6_rev_flag = 0;
    //rtems_task_wake_after(2*rtems_clock_get_ticks_per_second());
    
    
    //printf("before timer starts \n");
    rtems_timer_fire_after(timer_uart,
                           10 * rtems_clock_get_ticks_per_second(),
                           timer_uart_routine, NULL
                          );
    
    // wait timeout or data
    printf("before while\n");
    while(f_wait_ipsend_timeout == 0 && uart6_rev_flag == 0);
    printf("after while\r\n");
    
    // reset timeout flag
    if(f_wait_ipsend_timeout) 
    {
        printf("wait ipsend timeout\n");
        f_wait_ipsend_timeout = 0;
        
        return -1;
    }
    // cancel timer
    rc = rtems_timer_cancel(timer_uart);
    if(rc != RTEMS_SUCCESSFUL)
    {
        printf("timer error code %d\n", rc);
    }
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    //printf("%02x %02x %02x \n", rx_buf[0], rx_buf[1], rx_buf[2]);
    printf("ipsend reply:%s\n", rx_buf);
    if(strchr(rx_buf, '>'))
    {
        //printf("ready to send data 1020\n");
        //snprintf(command, len + 2, "%s\r\n", data);
        //send_at_command(command);
        usart_write_support_polled(3, data, len);

        
        //rtems_task_wake_after(1*rtems_clock_get_ticks_per_second());
        //rtems_counter_delay_nanoseconds(1000000 * 0.5); // 0.5ms

        #if 0
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("send tcp data reply:%s\n", rx_buf);
        //printf("%s function end\n", __FUNCTION__);
        #endif
        return 0;
        
    }
    else
    {
        printf("ipsend no respond\n");
        
        //rtems_task_wake_after(1);
        return -1;
    }
    
    
    
}

#if 0
int mqtt_init()
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    snprintf(command, sizeof(command), "AT+CMQTTSTART\r\n");
    printf("command %s\n", command);
    send_at_command(command);
    rtems_task_wake_after(1*rtems_clock_get_ticks_per_second());
    
    
    
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("mqtt start reply:%s\n", rx_buf);
    printf("%s function end\n", __FUNCTION__);

    if(strstr(rx_buf, "+CMQTTSTART: 0") == NULL)
    {
        printf("mqtt start failed \n");
        return -1;
    }


    snprintf(command, sizeof(command), "AT+CMQTTACCQ=0,\"client0\"\r\n");
    printf("command %s\n", command);
    send_at_command(command);
    rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("mqtt start reply:%s\n", rx_buf);


    // create connect 
    // AT+CMQTTCONNECT=0,"tcp://test.mosquitto.org:1883",60,1
    snprintf(command, sizeof(command), 
             "AT+CMQTTCONNECT=0,\"tcp://uxdar.f3322.net:1883\",60,1\r\n");
    printf("command %s\n", command);
    send_at_command(command);
    rtems_task_wake_after(2*rtems_clock_get_ticks_per_second());
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("mqtt connect reply:%s\n", rx_buf);
    

    //if(strstr(rx_buf, "+CMQTTSTART: 0") == NULL)
    {
        //printf("mqtt start failed \n");
        //return -1;
    }

    //set public topic
    ret = mqtt_cmd_then_data("CMQTTTOPIC", "CSM2610/heartbeat", 17);
    printf("the rett value %d\n", ret);

    return 0;    
}



int mqtt_cmd_then_data(const char *cmd, char *data, int len)
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    char command[200] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);

    // AT+CMQTTTOPIC=0,9
    snprintf(command, sizeof(command), "AT+%s=0,%d\r\n", cmd, len);
    printf("command %s\n", command);
    send_at_command(command);
    //rtems_task_wake_after(2*rtems_clock_get_ticks_per_second());
    
    
    //printf("before timer starts \n");
    rtems_timer_fire_after(timer_uart,
                           10 * rtems_clock_get_ticks_per_second(),
                           timer_uart_routine, NULL
                          );
    
    // wait timeout or data
    while(f_wait_ipsend_timeout == 0 && uart6_rev_flag == 0);
    
    // reset timeout flag
    if(f_wait_ipsend_timeout) 
    {
        printf("wait ipsend timeout\n");
        f_wait_ipsend_timeout = 0;
        
        return -1;
    }
    // cancel timer
    rtems_timer_cancel(timer_uart);
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    //printf("%02x %02x\n", rx_buf[0], rx_buf[1]);
    printf("send reply:%s\n", rx_buf);
    if(strchr(rx_buf, '>'))
    {
        printf("ready to send data\n");
        //snprintf(command, len + 2, "%s\r\n", data);
        memset(command, 0, sizeof(command));
        memcpy(command, data, len);
        command[len] = '\r';
        command[len+1] = '\n';
        printf("command[len+1] %02x\n", command[len+1]);
        printf("last two bytes[%d] %02x %02x\n", len-2, command[len-2], command[len-1]);
        printf("the length %d\n", len);
        send_at_cmd(command, len + 2);
        //send_at_command(command);
        rtems_task_wake_after(40*rtems_clock_get_ticks_per_second());
        //printf("first two bytes %02x %02x\n", command[0], command[1]);
        //printf("last two bytes[%d] %02x %02x\n", len-2, command[len-2], command[len-1]);
    
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("usart6 read ret %d\n", ret);
        printf("send data reply:%s\n", rx_buf);
        
        if(strstr(rx_buf, "OK"))
            return 0;
        else
            return -1;
        
    }
    else
    {
        printf("%s no respond\n", __FUNCTION__);

        // addd gprs cnt plus
        return -1;
    }
    
    
}


int mqtt_send_data(char *data, int len)
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);


    ret = mqtt_cmd_then_data("CMQTTPAYLOAD", data, len);
    printf("the rett value %d\n", ret);


    printf("ready to public mqtt msg\n");
    snprintf(command, sizeof(command), "AT+CMQTTPUB=0,1,60\r\n");
    printf("command %s\n", command);
    send_at_command(command);

    
    rtems_timer_fire_after(timer_uart,
                           60 * rtems_clock_get_ticks_per_second(),
                           timer_uart_routine, NULL
                          );
    
    // wait timeout or data
    while(f_wait_ipsend_timeout == 0 && uart6_rev_flag == 0);
    
    // reset timeout flag
    if(f_wait_ipsend_timeout) 
    {
        printf("wait ipsend timeout\n");
        f_wait_ipsend_timeout = 0;
        
        return -1;
    }
    // cancel timer
    rtems_timer_cancel(timer_uart);
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    
    printf("CMQTTPUB reply:%s\n", rx_buf);
    if(strstr(rx_buf, "OK"))
        return 0;
    else
        return -1;
   
}
#endif

uint8_t mq_connect_data[] = {
0x10, 0x2C, 0x00, 0x06, 0x4D, 0x51, 0x49, 0x73, 0x64, 0x70,
0x03, 0xC2, 0x00, 0x3C, 0x00, 0x06, 0x41, 0x42, 0x43, 0x44,
0x45, 0x46, 0x00, 0x08, 0x64, 0x78, 0x78, 0x6B, 0x67, 0x6B,
0x70, 0x70, 0x00, 0x0C, 0x71, 0x41, 0x55, 0x5A, 0x42, 0x64,
0x61, 0x53, 0x49, 0x55, 0x4C, 0x78};

int mqtt_connect()
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    
    //snprintf(command, sizeof(command), "AT+CMQTTSTART\r\n");
    //printf("command %s\n", command);
    //send_at_cmd(mq_connect_data, 46);
    ret = tcp_send_data(mq_connect_data, 46);
    printf("the ret %d\n", ret);
    #if 0
    rtems_task_wake_after(5*rtems_clock_get_ticks_per_second());
    
    
    
    
    memset(rx_buf, 0, rx_buf_len);
    ret = usart6_read(rx_buf, rx_buf_len);
    printf("mqtt connect reply:%s\n", rx_buf);
    printf("%s function end\n", __FUNCTION__);
    #endif

    

    return 0;    
}


int mqtt_publish()
{
    printf("%s function begin\n", __FUNCTION__);
    // check reg status
    if(gprs_is_registered() == 0)
    {
        printf("register status error \n");
        printf("cant send data now \n");
        return -1;    
    }
    
    int ret;
    unsigned char command[600] = {0};

    char *topic = "CSM2610/heartbeat";
    //char *rx_buf = command; 
    
    command[0] = 0x30;
    command[1] = 0;   // RL
    uint16_t topic_len = strlen(topic);
    
    command[2] = topic_len >>8;
    command[3] = topic_len & 0xff;
    memcpy(&command[4], topic, topic_len);

    memcpy(&command[4] + topic_len, g_GPRSSendBuffer, Send_len);

    command[1] = 2 + topic_len + Send_len;   // RL
    printf("%s function end\n", __FUNCTION__);

    int data_len = command[1] + 2;
    ret = tcp_send_data(command, data_len);
    printf("the ret %d\n", ret);

    return 0;    
}



// handle net register and connect
GPRSStatus net_process()
{
    netstat = 0;
    
    char command[160] = {0};
    char *rx_buf = command; 
    int rx_buf_len = sizeof(command);
    int ret;
    // power on   
    
    PWR_KEY_OFF();
    rtems_task_wake_after(1);
    PWR_KEY_ON();
    printf("pd2 up\n");
    rtems_task_wake_after(0.6*rtems_clock_get_ticks_per_second());
    PWR_KEY_OFF();
    // set 0
    
    
    //rtems_task_wake_after(10*rtems_clock_get_ticks_per_second());
    //power up
    netstat = GPRS_PowerUp;
    if(netstat == GPRS_PowerUp)
    {
        printf("wait 16s \n");
        sleep(16);
    }
    
    if(netstat == GPRS_PowerUp)
    {
        // register
        net_register(100);
    }
    
    if(netstat == GPRS_NET_ATTACHED)
    {
        
        snprintf(command, sizeof(command), "AT+NETOPEN\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("netopen reply: %s\n", rx_buf);
        
        // AT+IPADDR
        snprintf(command, sizeof(command), "AT+IPADDR\r\n");
        send_at_command(command);
        rtems_task_wake_after(0.3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("ipaddr reply: %s\n", rx_buf);
        
        
        //AT+CIPOPEN=0,"TCP","116.247.119.165",7015
        
        //char *ip = "monitor.iok.la";
        //uxdar.f3322.net
        char *ip = "uxdar.f3322.net";
        int port = 1883;//18327;


        #if 1  // disable it after mqtt open
        snprintf(command, sizeof(command), "AT+CIPOPEN=0,\"TCP\",\"%s\",%d\r\n",
                 ip, port);
        printf("command %s\n", command);
        send_at_command(command);
        rtems_task_wake_after(3*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("ipopen reply: %s\n", rx_buf);
        #endif

        #if 0
        snprintf(command, sizeof(command), "AT+CIPOPEN=0,\"UDP\",,,5000\r\n");
        printf("command %s\n", command);
        send_at_command(command);
        rtems_task_wake_after(2*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("ipopen reply: %s\n", rx_buf);
        #endif
        
        //"AT+CIPHEAD=0\r\n"
        snprintf(command, sizeof(command), "AT+CIPHEAD=0\r\n");
        send_at_command(command);
        rtems_task_wake_after(5*rtems_clock_get_ticks_per_second());
        
        memset(rx_buf, 0, rx_buf_len);
        ret = usart6_read(rx_buf, rx_buf_len);
        printf("IPHEAD reply: %s\n", rx_buf);
        
        
        
        
        
    }
    
    
    
    
    

    return netstat;
}



#define MPU6050_RA_PWR_MGMT_1 0x6B
#define GYRO_CONFIG 0x1b
#define ACCEL_CONFIG 0x1C
#define LP_CONFIG 0x1A


void mpu_init()
{
    
    
    printf("%s begin\n", __FUNCTION__);
    uint8_t temp = 0x00;

    int ret = I2C_SoftWare_Master_Write(MPU6050_DEFAULT_ADDRESS, MPU6050_RA_PWR_MGMT_1, &temp, 1);
    printf("ret  %d\n", ret);

    // MUST have
    temp = 0x00;
    I2C_SoftWare_Master_Write(MPU6050_DEFAULT_ADDRESS, ACCEL_CONFIG, &temp, 1);

    // Gyroscope Configuration
    temp = 0x00;
    I2C_SoftWare_Master_Write(MPU6050_DEFAULT_ADDRESS, GYRO_CONFIG, &temp, 1);
    temp = 0;
    I2C_SoftWare_Master_Read(MPU6050_DEFAULT_ADDRESS, GYRO_CONFIG, &temp, 1);
    //printf("the read temp %02x \n", temp);

    // div
    temp = 0x07;
    I2C_SoftWare_Master_Write(MPU6050_DEFAULT_ADDRESS, 0x19, &temp, 1);

    // low pass filter freq
    temp = 0x06;
    I2C_SoftWare_Master_Write(MPU6050_DEFAULT_ADDRESS, LP_CONFIG, &temp, 1);
}


float subsecond_to_second(uint32_t ss)
{
    printf("ss %u\n", ss);
    return (float)(255 - (int)ss) / 256.0; 

}


#define UART_NET_SPEED 1
#define RTC_DEBUG 1
char GPRS_IMEI[16] = {0};
char SIM_CCID[20] = {0x30};
uint8_t    GPRS_Mnc  = 0;					
uint32_t   GPRS_Cid  = 0;					//CID
uint32_t   GPRS_Lac  = 0;					//LAC

rtems_task Init(rtems_task_argument argument)
{
    
    rtems_status_code status;
    int usart_rev_len = 0;
    
    uint32_t *ISER1 = (uint32_t *)0xE000E104;
    uint32_t *ISER2 = (uint32_t *)0xE000E108;
    int gprs_err_cnt = 0;

    #if RTC_DEBUG
    rtems_time_of_day r = {0};
    #endif

    //minor = 2;
    usart_initialize(2); // index 2
    *ISER1 |= (1<<(39-32));
    
	//usart_write_support_polled(minor,"tea\n",4);
    printf("debug usart 3 init\n");
    
    keys_init(); // care nvic group
    // (*(uint32_t*)0xE000ED0C) = 0x05FA0000 | 0x400;
	
	
    PWR_KEY_INIT();
    stm32f4_gpio_set_config(&adc_enconfig);
    stm32f4_gpio_set_config(&adc_pin);
    stm32f4_gpio_set_config(&stm32_io_dtr);
    
    
    stm32_adc_init();
    
    float voltage = 1.250 +  3.3 * stm32_get_adc(14, 2) / 4096;
    printf("the adc result: %f \n", voltage);
    
    
    
    SCL_INIT();
    SDA_INIT();
    OLED_Init();
    OLED_Fill(0xff);
    OLED_Menu_Data();
    OLED_Refresh_Gram();
    
    //rtc_wakeup_setup();
    uint8_t aac[6] = {0};
    
    I2C_SoftWare_Master_Init();
    mpu_init();
    I2C_delay();

    uint8_t iam = 0;
    
    int rv = I2C_SoftWare_Master_Read(MPU6050_DEFAULT_ADDRESS, 0x75, &iam, 1);
    printf("i2c read rv value %d\n", rv);
    printf("whoami value %02x\n", iam);
    if(iam == 0x68)
    {
        printf("mpu starts...\n");
    }

    rv = I2C_SoftWare_Master_Read(MPU6050_DEFAULT_ADDRESS, 0x3b, aac, 6);
    printf("i2c read return value %d\n", rv);
    printf("aac x %04x \n", (uint16_t)aac[0] << 8 | aac[1]);
    printf("aac y %04x \n", (uint16_t)aac[2] << 8 | aac[3]);
    printf("aac z %04x \n", (uint16_t)aac[4] << 8 | aac[5]);
    // for uart
    char buffer[360] = {0};
    int buffer_len = sizeof(buffer);
    
    
    rtems_status_code ret = rtems_timer_create(rtems_build_name( 'T', 'M', 'R', '1' ), &Timer1);
    if(ret == RTEMS_SUCCESSFUL)
    {
        printf("create timer ok\n");
    }
    //(void) rtems_timer_create(rtems_build_name( 'T', 'M', 'R', '2' ), &Timer2);
    ret = rtems_timer_create(rtems_build_name( 'T', 'M', 'R', '2' ), &timer_uart);
    if(ret == RTEMS_SUCCESSFUL)
    {
        printf("create uart timer ok\n");
    }
    
    status = rtems_interrupt_handler_install(
        STM32F4_IRQ_EXTI2,
        "EINT2",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) EINT2_handler,
        NULL
    );
    if(status == RTEMS_SUCCESSFUL)
    {
        printf("interrupt  ok\n");
    }
    
    status = rtems_interrupt_handler_install(
        STM32F4_IRQ_USART2,
        "SP2",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) USART2_handler,
        NULL
    );
    if(status == RTEMS_SUCCESSFUL)
    {
        printf("interrupt  ok\n");
    }
    
    
    status = rtems_interrupt_handler_install(
        STM32F4_IRQ_USART1,
        "SP1",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) USART1_handler,
        NULL
    );
    if(status == RTEMS_SUCCESSFUL)
    {
        printf("usart1 interrupt installed ok\n");
    }
    
    status = rtems_interrupt_handler_install(
        STM32F4_IRQ_USART6,
        "SP6",
        RTEMS_INTERRUPT_UNIQUE,
        (rtems_interrupt_handler) USART6_handler,
        NULL
    );
    if(status == RTEMS_SUCCESSFUL)
    {
        printf("usart6 interrupt installed ok\n");
    }

    
    
    
    printf("1s ticks %d\n", rtems_clock_get_ticks_per_second());
    //rtems_task_wake_after(5*rtems_clock_get_ticks_per_second());
    
    //printf("stack check %d\n", rtems_stack_checker_is_blown());
    //printf("before oprator com3\n");
    rtems_stack_checker_report_usage();
    
    printf("isr level %d\n", _ISR_Get_level());
    
    
	#if 1
    int minor = 0;  // usart 1
    usart_initialize(minor);
    *ISER1 |= (1<<(37-32));  // nvic int enable
	
	//
	minor = 1;
    usart_initialize(minor);
    *ISER1 |= (1<<(38-32));  // nvic int enable
    
	
    printf("usart2 gps init\n");
    
	
    
    
    minor = 3;
    usart_initialize(minor);
    *ISER2 |= (1<<(71%32));  // nvic int enable
    
	//usart_write_support_polled(minor,"tea\n",4);
    printf("usart 6 gprs init\n");
    #endif
    
    
    
    
    
    PWR_ENABLE_INIT();
    PWR_ENABLE_SET(1);
    
    
    // enable gps 
    stm32f4_gpio_set_config(&gps_enconfig);
    net_process();

    gprs_get_imei(GPRS_IMEI, 16);
    gprs_get_ccid(SIM_CCID, 20);
    
    #if 0
    ret = tcp_send_data("12345", 5);
    if(ret == 0)
    {
        printf("tcp_send_data ok\n");    
    }
    #endif
    //char *gps_data = "$GPRMC,094423.00,A,2232.27251,N,11401.13288,E,0.145,,210319,,,A*77";
    //get_gps_data(gps_data, strlen(gps_data), NULL, NULL);

    ret = mqtt_connect();
    if(ret == 0)
    {
        printf("mqtt_connect ok\n");
    }

    ret = Get_OneData();
    printf("get one data returned value %d\n", ret);
    
    upload_freezer_data(0x03, NULL, 0);
    //mqtt_send_data(g_GPRSSendBuffer, Send_len); // send all in one data
    mqtt_publish();
    #if 0
    int i;

    printf("*** tcp send\r\n");
    #if RTC_DEBUG
        
    stm32f4_rtc_gettime(0, &r); 
    printf("time  %d:%d:%d %f\n",r.hour, r.minute, r.second,
           subsecond_to_second(r.ticks));
        
	#endif
    for(i = 0; i < 100; i++) tcp_send_1kdata();

    printf("100k test over\n");
    
    while(!uart6_rev_flag);
    memset((void *)buffer, 0, buffer_len);
	if(usart_rev_len = usart6_read(buffer, buffer_len))
    {
        //printf("***usart6 buf %d\n", usart_rev_len);
        printf("***usart6 buf %s\n", buffer);

    }

    #if RTC_DEBUG
        
    stm32f4_rtc_gettime(0, &r); 
    //printf("time  %3d:%3d:%3d \n",r.hour, r.minute, r.second);
    printf("time  %d:%d:%d %f\n", r.hour, r.minute, r.second,
           subsecond_to_second(r.ticks));
        
	#endif
    printf("tcp test end\n");

    #endif
    OLED_Display_Off();
    sleep(2);
    STM32_IO_DTR_ON();
    printf("dtr to high \n");
    stm32f4_gpio_set_config(&iic_scl_dummy);
    stm32f4_gpio_set_config(&iic_sda_dummy);

    

    //ret = Get_OneData();
    //printf("get one data %d\n", ret);
    //Upload_LGData();
    //spi1_init();
    stm32f4_gpio_set_config(&spi1_cs);
    
    printf("begnin spi debug \n");
    sleep(10);
    W25QXX_Init();
    uint8_t wd_test[1024] = {[0 ... 1023] = 0x3a};

    
    W25QXX_Write(wd_test, 7, 1024);
    
    
    memset(wd_test, 0, 1024);
    printf("before flash read\n");
    W25QXX_Read(wd_test, 7, 1024);
    //printf("flash read %02x ... %02x\n", wd_test[0], wd_test[1023]);
    int j;
    for(j = 0; j < 1024; j++)
        printf("%02x ", wd_test[j]);
    
    printf("before while\n");
    while(1)
    {
        printf("loop begin\n");
		#if 1
		if(f_key_pressed)
	    {
		    printf("key pressed \n");
		    f_key_pressed = 0;	
	    }
	    #endif

        //char test[4] = {0x1a, 0x0a, 0x05, 0x05};
        //usart_write_support_polled(0, test, 4);
	    
        memset((void *)buffer, 0, buffer_len);
	    if(usart_rev_len = usart2_read(buffer, buffer_len))
        {
            printf("usart2 buf %d\n", usart_rev_len);
            printf("usart2 buf %s\n", buffer);
            
            #if 0
            int i;
            for(i = 0; i < usart_rev_len; i++)
               printf("%02X ", buffer[i]);
            printf("\n");
            #endif
            strncpy(gps_buffer, buffer, sizeof(gps_buffer)-1);
            //get_gps_data(buffer, usart_rev_len, NULL, NULL);
            
        }
        
        #if 0
        memset(buffer, 0, buffer_len);
	    if(usart_rev_len = usart1_read(buffer, buffer_len))
        {
            
            printf("reading usart data\n");
            printf("usart1 buf %d\n", usart_rev_len);
            printf("usart1 buf %s\n", buffer);

            int j;
            for(j = 0; j < usart_rev_len; j++)
                printf("%02x ", buffer[j]);
            //get_gps_data(buffer, usart_rev_len, NULL, NULL);
            
            //memset(buffer, 0, buffer_len);
        }
        else
        {
            printf("usart 1 buf empty\n");    
        }
        #endif

        ret = Get_OneData();
        printf("get one data returned value %d\n", ret);
        //Upload_LGData(0x03);
        upload_freezer_data(0x03, NULL, 0);


        printf("flash id %04x\n", W25QXX_ReadID());
        sleep(6);


        #if RTC_DEBUG
        
        //stm32f4_rtc_gettime(0, &r); 
        //printf("time  %d:%d:%d \n", r.hour, r.minute, r.second);
        
		#endif

        rv = I2C_SoftWare_Master_Read(MPU6050_DEFAULT_ADDRESS, 0x3b, aac, 6);
        printf("i2c read return value %d\n", rv);
        int16_t aacx, aacy, aacz;

        aacx = (uint16_t)aac[0] << 8 | aac[1];
        aacy = (uint16_t)aac[2] << 8 | aac[3];
        aacz = (uint16_t)aac[4] << 8 | aac[5];
        
        
        printf("aac x %f \n", aacx / 16384.0);
        printf("aac y %f \n", aacy / 16384.0);
        printf("aac z %f \n", aacz / 16384.0);
        
        
	}  

    (void)rtems_task_delete( RTEMS_SELF );


}


/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_RTC_DRIVER

#define CONFIGURE_MAXIMUM_TASKS         5
#define CONFIGURE_MAXIMUM_TIMERS        3

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
//#define CONFIGURE_STACK_CHECKER_ENABLED
#define CONFIGURE_INIT_TASK_STACK_SIZE (RTEMS_MINIMUM_STACK_SIZE * 3)
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 20
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
/****************  END OF CONFIGURATION INFORMATION  ****************/
