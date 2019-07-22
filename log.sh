#!/usr/bin/env bash

### Config Vars
projectFolder="~/projects/ArduinoOven";
logFile="$projectFolder/log/pid.log";
###

### Setting up Serial
sleep 10;
stty -F /dev/ttyACM0 38400 -hupcl;
###

# Write Serial to logFile
cat /dev/ttyACM0 >> $logFile
