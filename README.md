# ArduinoOven

This is an Arduino-controlled conveyor oven.

It has two burners, at the top and bottom of the chamber, each with its own thermocouple. Each burner has a stepper motor attached to its valve, and is controlled with a PID-control-loop.

The conveyor is moved by a DC motor, with an encoder attached to the shaft, and is also controlled with a PID-control-loop.

## Video

https://www.youtube.com/watch?v=MHU5xQRTyus


# Flash Arduino with Raspberry Pi

## Install Arduino CLI

https://github.com/arduino/arduino-cli

## Install Board and libraries

Need to document this better. The board is an *Arduino mega*, and the libraries are in the `/libraries` folder

## Compile and Upload firmware

Open terminal, cd into project folder and execute this command

    arduino-cli compile --warnings default --fqbn arduino:avr:mega &&
    arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM0


# Setup logging and graphing PIDs data in a Raspberry Pi

## Enable in `ArduinoOven.ino` the flag DEBUG_PID_JSON

    #define DEBUG_PID_JSON

## Add entries to cron

1. open crontab

       crontab -e

2. Add the following cronjobs at the end, changing `projectFolder`

        @reboot {projectFolder}/log.sh
        */1 * * * * date +"\%FT\%R" >> {projectFolder}/log/pid.log