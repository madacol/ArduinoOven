This is an Arduino Mega controlled conveyor oven, with semi-independent burners at the top and bottom with their own thermocouple, it's semi-independent because bottom burner affects top.
This design sacrifices control to get a simpler design compared to those convection based oven.

We use 3 PID controls, two on burners (top and bottom) and the third on the conveyor. It's possibly an overkill, but it works!.

**Video:** https://www.youtube.com/watch?v=MHU5xQRTyus

# Flash Arduino with Raspberry Pi

## Install Arduino CLI

https://github.com/arduino/arduino-cli

## Compile and Upload firmware

Open terminal, cd into project folder and execute this command

    arduino-cli compile --warnings default --fqbn arduino:avr:mega && arduino-cli upload --fqbn arduino:avr:mega -p /dev/ttyACM0


# Setup logging and graphing PIDs data in a Raspberry Pi

## Enable in `ArduinoOven.ino` the flag DEBUG_PID_JSON

    #define DEBUG_PID_JSON

## Add entries to cron

1. open crontab

       crontab -e

2. Add the following cronjobs at the end, changing `projectFolder`

        @reboot {projectFolder}/log.sh
        */1 * * * * date +"\%FT\%R" >> {projectFolder}/log/pid.log