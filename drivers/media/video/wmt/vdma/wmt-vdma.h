/*
 * linux/drivers/media/video/wmt/vdma/wmt-vdma.h
 *
 * Video DMA API for WM8510 ARM SoC.
 *
 * Author: Vincent Chen <vincentchen@wondermedia.com.tw>
 *
 * Copyright 2008-2009 WonderMedia Corporation. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY WONDERMEDIA CORPORATION ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL WONDERMEDIA CORPORATION OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
 * THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef WMT_VDMA_H
#define WMT_VDMA_H
#define MAX_VDMA_CHANNEL	8
int request_vdma(unsigned int channel, const char *name);
void free_vdma(unsigned int channel);
void set_vdma_source(int channel, unsigned int addr,
	int w, int h, int bpp);
void set_vdma_destination(int channel, unsigned int addr,
	int w, int h, int bpp);
void set_vdma_region(int channel, int src_x, int src_y,
	int dst_x, int dst_y, int pic_w, int pic_h);
int enable_vdma(unsigned int channel);
int test_vdma(unsigned int channel);
#endif
