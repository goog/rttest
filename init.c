

#include <stdio.h>
#include <bsp.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/time.h>
#include <errno.h>
#include <bsp/i2c.h>
#include "led.h"
#include "pins.h"
        
#define DEVICE_EEPROM 0x50


typedef enum                                           //���õ�״̬
{
	/*********************�����ǷǼ���״̬����*****************************/
	GPRS_IdleInit=0,               //��ʱ�ػ�׼�����ȴ����ݷ������
	GPRS_IdlePWRKEYOFF,            //IO�ػ�
	GPRS_IdleDelayPowerOFF,        //��ʱ�رյ�Դ
	GPRS_IdlePowerOFF,             //�رյ�Դ
	GPRS_Idle,                     //����Ǽ����
	
	/*********************��������������*****************************/
	GPRS_PowerInit,                //ϵͳ������ĳЩ������ʼ��
	GPRS_InitOFF,                  //�ϵ��ʼ����ʱ�򣬹رյ�Դ
	GPRS_PWRKEYOFF,                //IO�ڹػ�
	GPRS_DelayPowerOFF,            //��ʱ�رյ�Դ
	GPRS_PowerOFF,                 //�رյ�Դ
	GPRS_DelayRestart,             //��ʱ����
	GPRS_PowerUp,                  //ģ���Դ�ϵ�
	GPRS_PWRKEYON,                 //IO�ڿ���
	GPRS_DelayInit,                //��������ʱһ��ʱ�䣬Ȼ���ٳ�ʼ��
	                               
	GPRS_CHECK_AT,                 //
	GPRS_SET_ATE0,                 //�ػ���
	GPRS_CHECK_IPR,                //��ѯģ��Ĳ�����
	GPRS_SET_IPR,                  //�̶�ģ��Ĳ�����
	GPRS_GET_IMEI,                 //��ȡģ���IMEI��
	GPRS_GET_CCID,                 //��ȡSIM����CCID��
	GPRS_SET_CSCLK,                //����˯��ģʽ
	
	GPRS_GET_CIMI,		//��ȡMCC/MNC��Ϣ
	GPRS_SET_APN,			//����APN
	GPRS_CHECK_CSQ,		//��ѯ�ź�ǿ��
	
	GPRS_CHECK_OPERATOR,	//��ѯ��Ӫ��
	
	GPRS_CHECK_CPSI,	//��ѯע������״̬
	GPRS_SET_PDP,			//PDP����
	GPRS_GET_IPADDR,	//��ȡIP��ַ
	
	GPRS_WAIT_CIPOPEN,		//TCP/IP��������
	GPRS_SET_CIPOPEN,			//���ӷ�����
	GPRS_SET_CIPHEAD,			//���ý���Զ��������ʾIPͷ
	GPRS_WAIT_NETCLOSE,		//PDPȥ����
	
	
	GPRS_SET_NETCLOSE,			//PDPȥ����ɹ�
	
	

	
	/*********************����������������*****************************/
	GPRS_CONNECT                   //������
	
}
GPRSStatus;


typedef unsigned char uint8;

volatile int GPRS_status =0;
volatile uint8 timer_timeout =0;
rtems_id Timer1;
static uint8 i2c_read(int fd, uint8 reg, uint8 *val);
static uint8 printarray(uint8 Array[], uint8 Num);

#define DEV_I2C_NAME "/dev/i2c1"
static const char bus_path[] = "/dev/i2c-1";
static const char eeprom_path[] = "/dev/i2c-1.eeprom-0";
/*
int AT24C02_Init(void)
{
    #if 1
    int ret = i2c_dev_register_eeprom(
		bus_path,
		eeprom_path,
		DEVICE_EEPROM,
		1,
		8,
		256,  // size
		0
		);
    if(ret == 0)
    {
		printf("drive ok\n");
	}
	else
    {
		perror("register failed\n");
		printf("error %d\n", ret);
	}
    #endif
    int fd = open(DEV_I2C_NAME, O_RDWR);
    if(fd < 0)
    {
        printf("Can't open %s \n", DEV_I2C_NAME);
        //return fd;
        exit(1);
    }
   
    printf("open /dev/i2c/0 success !\n");
   
    if(ioctl(fd, I2C_SLAVE, Address) < 0)
	{   
		printf("fail to set i2c device slave address!\n");
        close(fd);
        return -1;
    }
       
    printf("set slave address to 0x%x success!\n", Address);
    #if 0   
    if(ioctl(fd, I2C_BUS_MODE, 1)<0)
            printf("set bus mode fail!\n");
    else
            printf("set bus mode ok!\n");
	#endif
	
    return fd;
}
*/



static int i2c_write(int fd, uint8 reg, uint8 val)
{
	int retries;
    uint8 data[2] = {0};

    data[0] = reg;
    data[1] = val;
        
    for(retries=5; retries; retries--) 
    {
		if(write(fd, data, 2) == 2)
            return 0;
        usleep(1000*10);
    }
    
    return -1;
}

//at24c02读取一字节
static uint8 i2c_read(int fd, uint8 reg, uint8 *val)
{
    int retries;

    for(retries=5; retries; retries--)
        if(write(fd, &reg, 1)==1)
        if(read(fd, val, 1) == 1)
            return 0;
    
    return -1;
}

//输出数组
static uint8 printarray(uint8 Array[], uint8 Num)
{
  uint8 i;
 
  for(i=0;i<Num;i++)
  {
    printf("Data [%d] is %d \n", i ,Array[i]);
  }
 
  return(1);
}

extern stm32f4_i2c_bus_entry *const stm32f4_i2c1;
extern int ex_usart_read_polled(int minor);

void uart_poll(int fd, char *buf, int len)
{
    int n;
    char ch;
    int i = 0;
    printf("in poll function.\n");
    for(;timer_timeout != 1;)
    {
        n = ex_usart_read_polled(0);
        //printf("n %d cc %x\n", n, ch);
        if (n < 0)
        {
            //rtems_task_wake_after (0.1);  // delay 10ms
            int j;
            //for(j = 0; j < 1000; j++) j = j;
        }
        else //if(n == 1)
        {
            
            ch = n;
            //printf("%x, %x \n", ch, n);
            if(ch == '\r')
                continue;

            if(ch == '\n')
            {

                printf("uart %s\n", buf);
                break;
            }

            buf[i] = ch;
            //printf("the %d th char %c \n", i, buf[i]);
            i++;

            if(i == len)
            {
		        printf("uart buf full %s\n", buf);
                break;

            }
        }
    }
}


rtems_timer_service_routine Timer_Routine( rtems_id id, void *ignored )
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

int ux_uart_init(char *dev_name)
{
	printf("ux_uart_init begin\n");
    int fd = open(dev_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

    printf("\nOpened COM3, fd=%d \n", fd);
    LED_ON();
    if(fd < 0)
    {
	    printf("open error\n");
	    //LED_ON();
	    return -1;	
	}
      

  
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

rtems_task Init(rtems_task_argument argument)
{
    puts( "\n\n*** i2c test begin ***\n");
	LED_INIT();
	LED_OFF();
    uint8_t buf[20] = {0};
    
    // for uart
    char buffer[128] = {0};
    int buffer_len = sizeof(buffer);
    
    rtems_status_code ret = rtems_timer_create(rtems_build_name( 'T', 'M', 'R', '1' ), &Timer1);
    if(ret == RTEMS_SUCCESSFUL)
    {
        printf("create timer ok\n");
    }
    //(void) rtems_timer_create(rtems_build_name( 'T', 'M', 'R', '2' ), &Timer2);

    
    PWR_ENABLE_INIT();
    PWR_ENABLE_SET(1);
    // power on   
    // configure pin
    PWR_KEY_INIT();
    rtems_task_wake_after(1);
    PWR_KEY_ON();
    rtems_task_wake_after(0.5*rtems_clock_get_ticks_per_second());
    PWR_KEY_OFF();
    // set 0
    GPRS_status = GPRS_PowerUp;
    
    rtems_task_wake_after(5*rtems_clock_get_ticks_per_second());
    
    printf("before oprator com3\n");
    rtems_stack_checker_report_usage();
    // at command starts
    int g4_fd = ux_uart_init("/dev/ttyS2");
    if(g4_fd < 0)
    {
	    printf("open uart node failed \n");	
    }
	
	printf("ttys2 fd %d\n", g4_fd);
	write(g4_fd, "AT\r\n", 4);
	timer_timeout = 0;
	rtems_timer_fire_after(Timer1, 1 * rtems_clock_get_ticks_per_second(),
                           Timer_Routine, NULL);
    uart_poll(g4_fd, buffer, buffer_len);
    rtems_timer_cancel(Timer1);
    
    printf("buffer len %d\n", strlen(buffer));
    if(strstr(buffer, "OK"))
    {
        printf("uart recv %s\n", buffer);
        LED_ON();	
	}
	rtems_status_code sc = stm32f4_i2c_init(stm32f4_i2c1);
	printf("the sc is %d\n", sc);
	
	
	stm32f4_i2c_message msg = {0};
	msg.addr = 0x50;
	msg.read = false;
	msg.len = 1;
	msg.buf = buf;
	buf[0] = 0x66;
	sc = stm32f4_i2c_process_message(stm32f4_i2c1, &msg);
	printf("the write status %d\n", sc);
	rtems_task_wake_after(2);
	
	
	memset(buf, 0, sizeof(buf));
	msg.read = true;
	msg.len = 1;
	msg.buf = buf;
	sc = stm32f4_i2c_process_message(stm32f4_i2c1, &msg);
	printf("the read status %d\n", sc);
	sleep(1);
	
	int i = 0;
	for(i = 0; i < 10; i++)
		printf("%x\t", buf[i]);
  
  

    (void)rtems_task_delete( RTEMS_SELF );
}


/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS         3
#define CONFIGURE_MAXIMUM_TIMERS        3

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 20
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
/****************  END OF CONFIGURATION INFORMATION  ****************/
