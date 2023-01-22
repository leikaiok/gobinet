#!/bin/sh

echo start modprobe driver...

modprobe usbnet

modprobe mii 

make install 


