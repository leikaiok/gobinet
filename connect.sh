#!/bin/sh

if [ "$1"x = "open"x ];then

   echo start connect network...
   echo 1 > /sys/testgobi/gobi0

elif [ "$1"x = "close"x ];then

   echo start disconnect network...
   echo 0 > /sys/testgobi/gobi0

else

   echo parameters error,do nothing!!!

fi



echo complete!!!
