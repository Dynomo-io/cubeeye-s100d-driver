#!/bin/sh

#
# Makefile
# 
#  Created on: 2018. 10. 15.
#      Author: erato
#

LOCAL_PATH := $(shell pwd)

.PHONY: all clean
all: 
	@make -C $(LOCAL_PATH)/ReadDepthFrame
	@make -C $(LOCAL_PATH)/ReadPCLFrame
	@make -C $(LOCAL_PATH)/ReadIntensityPCLFrame
	@make -C $(LOCAL_PATH)/SetGetProperty

clean:
	@make -C $(LOCAL_PATH)/ReadDepthFrame clean
	@make -C $(LOCAL_PATH)/ReadPCLFrame clean
	@make -C $(LOCAL_PATH)/ReadIntensityPCLFrame clean
	@make -C $(LOCAL_PATH)/SetGetProperty clean

