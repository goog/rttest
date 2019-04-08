/*
 *  File name: init.c
 *
 *  A simple serial and shell test program.
 *
 *  COPYRIGHT (c) 2008.
 *  RobertF
 *
 *  The license and distribution terms for this file may be
 *  found in the file LICENSE in this distribution or at
 *  http://www.rtems.com/license/LICENSE.
 *
 */

#define CONFIGURE_INIT

#include <stdio.h>
#include <stdlib.h>

#include <string.h>  /* String function definitions */
#include <unistd.h>  /* UNIX standard function definitions */
#include <fcntl.h>   /* File control definitions */
#include <errno.h>   /* Error number definitions */
#include <termios.h> /* POSIX terminal control definitions */
#include <sys/select.h>
#include <rtems.h>
#include <rtems/shell.h>
#include <bsp.h>



extern rtems_task Init(rtems_task_argument argument);

//#define CONFIGURE_APPLICATION_EXTRA_DRIVERS  TTY1_DRIVER_TABLE_ENTRY


#define CONFIGURE_APPLICATION_NEEDS_CONSOLE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_CLOCK_DRIVER
#ifdef RTEMS_BSP_HAS_IDE_DRIVER
#define CONFIGURE_APPLICATION_NEEDS_IDE_DRIVER
#endif

//#define CONFIGURE_APPLICATION_NEEDS_LIBBLOCK
//#define CONFIGURE_USE_IMFS_AS_BASE_FILESYSTEM

/*
 * these values are higher than needed...
 */
#define CONFIGURE_MAXIMUM_TASKS             3
//#define CONFIGURE_MAXIMUM_SEMAPHORES        20
//#define CONFIGURE_MAXIMUM_MESSAGE_QUEUES    20
#define CONFIGURE_LIBIO_MAXIMUM_FILE_DESCRIPTORS 10
//#define STACK_CHECKER_ON
#define CONFIGURE_RTEMS_INIT_TASKS_TABLE

//#define CONFIGURE_EXTRA_TASK_STACKS         (6 * RTEMS_MINIMUM_STACK_SIZE)


#define CONFIGURE_INIT
#include <rtems/confdefs.h>


//#define CONFIGURE_SHELL_COMMANDS_INIT
//#define CONFIGURE_SHELL_COMMANDS_ALL

//#include <rtems/shellconfig.h>

void testCom1(void);
int uart_recv(int fd, char *rcv_buf,int data_len);
extern int ex_usart_read_polled(int minor);

void fillBufferPoll(int fd, char *buf, int len)
{
    int n;
    char ch;
    int i = 0;
    printf("in poll function.\n");
    for(;;)
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

void testCom1(void) {
  char buffer[128] = {0};

  printf("*** Simple COM1 Test ***\n");

  //int fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | _FNDELAY);
  int fd = open("/dev/ttyS2", O_RDWR | O_NOCTTY | O_NONBLOCK);

  printf("\nOpened COM1, fd=%d\n\n", fd);
  if(fd < 0)
      printf("open error\n");

  
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
      //return;  
  }

  int numBytes = write(fd, "Hello, I'm waiting for input..\r\n", 33);

  if (numBytes < 0) {
    printf("\nFailed to send from COM1!\n");
  }


  fillBufferPoll(fd, buffer, 20);
  
  numBytes = write(fd, "line2\r\n", 7);
  if (numBytes < 0) {
    printf("\nFailed to send from COM1!\n");
  }

  memset(buffer, 0, sizeof(buffer));
  fillBufferPoll(fd, buffer, 20);
  
  close(fd);
  
}



rtems_task Init(rtems_task_argument ignored) {

  testCom1();

  printf("\n====== end ======\n");

  (void) rtems_task_delete( RTEMS_SELF );

#if 0
  rtems_shell_init(
    "SHLL",                          /* task_name */
    RTEMS_MINIMUM_STACK_SIZE * 4,    /* task_stacksize */
    100,                             /* task_priority */
    "/dev/console",                  /* devname */
    0,                               /* forever */
    1                                /* wait */
  );
#endif
  
}
