#
# Makefile for ALSA
#

obj-$(CONFIG_SND_SA11XX_UDA1341) += snd-sa11xx-uda1341.o 
snd-sa11xx-uda1341-objs		:= sa11xx-uda1341.o

obj-$(CONFIG_SND_ARMAACI)	+= snd-aaci.o
snd-aaci-objs			:= aaci.o devdma.o

obj-$(CONFIG_SND_PXA2XX_PCM)	+= snd-pxa2xx-pcm.o
snd-pxa2xx-pcm-objs		:= pxa2xx-pcm.o

obj-$(CONFIG_SND_PXA2XX_LIB)	+= snd-pxa2xx-lib.o
snd-pxa2xx-lib-y		:= pxa2xx-pcm-lib.o
snd-pxa2xx-lib-$(CONFIG_SND_PXA2XX_LIB_AC97)	+= pxa2xx-ac97-lib.o

obj-$(CONFIG_SND_PXA2XX_AC97)	+= snd-pxa2xx-ac97.o
snd-pxa2xx-ac97-objs		:= pxa2xx-ac97.o
obj-$(CONFIG_SOUND_WMT_I2S) += wmt_i2s.o i2s.o wmt_swmixer.o
obj-$(CONFIG_I2S_Codec_VT1602) += vt1602.o 
obj-$(CONFIG_SOUND_WMT_AC97)         += wmt_ac97.o 
obj-$(CONFIG_WMT_VT1613)		+= vt1613_generic.o ac97_vt1613.o
