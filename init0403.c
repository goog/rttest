/*
 *  COPYRIGHT (c) 1989-2009.
 *  On-Line Applications Research Corporation (OAR).
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 */

#include <stdio.h>
#include <bsp.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>

#include <sys/time.h>
#include <errno.h>
#include <dev/i2c/eeprom.h>


#define Address 0x50         
#define DEVICE_EEPROM 0x50
#define I2C_RETRIES   0x0701
#define I2C_TIMEOUT   0x0702
#define I2C_SLAVE     0x0703       //IIC从器件的地址设置
#define I2C_BUS_MODE   0x0780

typedef unsigned char uint8;

uint8 rbuf[8] = {0x00}; //读出缓存
uint8 wbuf[8] = {0x01,0x05,0x06,0x04,0x01,0x01,0x03,0x0d}; //写入缓存
//int fd = -1;

//函数声明
//static uint8 AT24C02_Init(void);
//static uint8 i2c_write(int fd, uint8 reg, uint8 val);
static uint8 i2c_read(int fd, uint8 reg, uint8 *val);
static uint8 printarray(uint8 Array[], uint8 Num);

#define DEV_I2C_NAME "/dev/i2c1"
static const char bus_path[] = "/dev/i2c-1";
static const char eeprom_path[] = "/dev/i2c-1.eeprom-0";

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

/*
uint8 AT24C02_Write(uint8 *nData, uint8 Reg, uint8 Num)
{
  write(fd, &Reg, 1);  //
  usleep(100);              //延时100us
  write(fd, nData, Num);
  usleep(1000*4);             //延时 4ms
 
  return(1);
}

uint8 AT24C02_Read(uint8 nData[], uint8 Reg, uint8 Num)
{
  write(fd, &Reg, 1);
  usleep(100);              //延时100us
  read(fd, nData, Num);
  usleep(1000*4);             //延时 4ms
 
  return(1);
}
*/

//at24c02写入一字节
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


rtems_task Init(
  rtems_task_argument argument
)
{
    puts( "\n\n*** i2c test begin ***\n");

  //LED_INIT();
  //LED_OFF();
	int fd = AT24C02_Init();
	printf("the fd is %d\n", fd);
	int ret = i2c_write(fd, 0, 0x44);
	printf("the i2c write return %d\n", ret);
	
	uint8 value = 0;
    i2c_read(fd, 0, &value);
    printf("value is %x\n", value);
  
  

  (void) rtems_task_delete( RTEMS_SELF );
}


/**************** START OF CONFIGURATION INFORMATION ****************/

#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER

#define CONFIGURE_MAXIMUM_TASKS         3
#define CONFIGURE_MAXIMUM_TIMERS        2

#define CONFIGURE_RTEMS_INIT_TASKS_TABLE
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 10
#define CONFIGURE_INIT
#include <rtems/confdefs.h>
/****************  END OF CONFIGURATION INFORMATION  ****************/
