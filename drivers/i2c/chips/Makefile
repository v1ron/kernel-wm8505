#
# Makefile for the kernel hardware sensors chip drivers.
#

# asb100, then w83781d go first, as they can override other drivers' addresses.
obj-$(CONFIG_SENSORS_ASB100)	+= asb100.o
obj-$(CONFIG_SENSORS_W83627HF)	+= w83627hf.o
obj-$(CONFIG_SENSORS_W83781D)	+= w83781d.o

obj-$(CONFIG_SENSORS_ADM1021)	+= adm1021.o
obj-$(CONFIG_SENSORS_ADM1025)	+= adm1025.o
obj-$(CONFIG_SENSORS_ADM1026)	+= adm1026.o
obj-$(CONFIG_SENSORS_ADM1031)	+= adm1031.o
obj-$(CONFIG_SENSORS_DS1621)	+= ds1621.o
obj-$(CONFIG_SENSORS_EEPROM)	+= eeprom.o
obj-$(CONFIG_SENSORS_FSCHER)	+= fscher.o
obj-$(CONFIG_SENSORS_GL518SM)	+= gl518sm.o
obj-$(CONFIG_SENSORS_IT87)	+= it87.o
obj-$(CONFIG_SENSORS_LM63)	+= lm63.o
obj-$(CONFIG_SENSORS_LM75)	+= lm75.o
obj-$(CONFIG_SENSORS_LM77)	+= lm77.o
obj-$(CONFIG_SENSORS_LM78)	+= lm78.o
obj-$(CONFIG_SENSORS_LM80)	+= lm80.o
obj-$(CONFIG_SENSORS_LM83)	+= lm83.o
obj-$(CONFIG_SENSORS_LM85)	+= lm85.o
obj-$(CONFIG_SENSORS_LM87)	+= lm87.o
obj-$(CONFIG_SENSORS_LM90)	+= lm90.o
obj-$(CONFIG_SENSORS_MAX1619)	+= max1619.o
obj-$(CONFIG_SENSORS_M41T00)	+= m41t00.o
obj-$(CONFIG_SENSORS_PC87360)	+= pc87360.o
obj-$(CONFIG_SENSORS_PCF8574)	+= pcf8574.o
obj-$(CONFIG_SENSORS_PCF8591)	+= pcf8591.o
obj-$(CONFIG_SENSORS_RTC8564)	+= rtc8564.o
obj-$(CONFIG_SENSORS_SMSC47M1)	+= smsc47m1.o
obj-$(CONFIG_SENSORS_VIA686A)	+= via686a.o
obj-$(CONFIG_SENSORS_W83L785TS)	+= w83l785ts.o
obj-$(CONFIG_TPS65010)		+= tps65010.o
obj-$(CONFIG_ISP1301_OMAP)	+= isp1301_omap.o
obj-$(CONFIG_SENSORS_TLV320AIC23) += tlv320aic23.o
obj-$(CONFIG_GPIOEXPANDER_OMAP)	+= gpio_expander_omap.o
obj-$(CONFIG_WMT_I2C_API)	+= wmt-i2c-api.o

ifeq ($(CONFIG_I2C_DEBUG_CHIP),y)
EXTRA_CFLAGS += -DDEBUG
endif

#obj-$(CONFIG_I2C_VT1622A_TVOUT) += tvout.o vt1622a_i2c.o
#obj-$(CONFIG_I2C_VT1626_TVOUT) += vt1626_tvout.o vt1626_i2c.o

obj-$(CONFIG_I2C_TDA7421N_P) 	+= tda7421n_pll_i2c.o  	#Paul Added for PLL
obj-$(CONFIG_I2C_TDA7421N_T) 	+= tda7421n_tnr_i2c.o  	#Paul Added for tuner
obj-$(CONFIG_I2C_TDA7416) 	+= tda_csp.o  	        #Paul Added for signal processor
obj-$(CONFIG_I2C_M24C01) 	+= m24c01.o  		#Paul Added for eeprom
obj-$(CONFIG_I2C_RADIO) 	+= radio_ctl.o 		#Paul Added for radio device driver
obj-$(CONFIG_I2C_AUDIO) 	+= audio_ctl.o 		#Paul Added for audio device driver
