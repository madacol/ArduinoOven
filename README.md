# ArduinoOven

This is an Arduino controlled conveyor oven, with semi-independent burners at the top and bottom with their own thermocouple, it's semi-independent because bottom burner affects top.
This design sacrifices control to get a simpler design compared to those convection based oven.

We use 3 PID controls, two on burners (top and bottom) and the third on the conveyor. It's possibly an overkill, but it works!.

# Videos

https://www.youtube.com/watch?v=MHU5xQRTyus
