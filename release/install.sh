#!/bin/sh

LOCAL_DIR=$PWD
CUBEEYE_SHARE_DIR=/usr/local/share/cubeeye
CUBEEYE_UDEV_RULES=$LOCAL_DIR/udev/98-CubeEye.rules

if [ ! -d $CUBEEYE_SHARE_DIR ]; then
	mkdir -p $CUBEEYE_SHARE_DIR
fi

if [ -d $CUBEEYE_SHARE_DIR ]; then
	if [ -d $LOCAL_DIR/conf ]; then
		cp -r $LOCAL_DIR/conf $CUBEEYE_SHARE_DIR
	fi
	if [ -d $LOCAL_DIR/fw ]; then
		cp -r $LOCAL_DIR/fw $CUBEEYE_SHARE_DIR
	fi
fi

if [ -f $CUBEEYE_UDEV_RULES ]; then
	cp $CUBEEYE_UDEV_RULES /etc/udev/rules.d/
fi
