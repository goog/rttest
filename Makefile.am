
rtems_tests_PROGRAMS = hello
hello_SOURCES = init.c oled.c oled_iic.c gps.c adc.c I2C_Software_Master.c oled_iic.h oledfont.h led.h pins.h oled.h time_rtc.h I2C_Software_Master.h adc.h carrier.h carrier.c spi.h spi.c w25qxx.c w25qxx.h

dist_rtems_tests_DATA = hello.scn
dist_rtems_tests_DATA += hello.doc

include $(RTEMS_ROOT)/make/custom/@RTEMS_BSP@.cfg
include $(top_srcdir)/../automake/compile.am
include $(top_srcdir)/../automake/leaf.am


LINK_OBJS = $(hello_OBJECTS)
LINK_LIBS = $(hello_LDLIBS)

hello$(EXEEXT): $(hello_OBJECTS) $(hello_DEPENDENCIES)
	@rm -f hello$(EXEEXT)
	$(make-exe)

include $(top_srcdir)/../automake/local.am
