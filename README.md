# Slambot motor controller

Dual Motor Control with DRV8833 and Quadrature Encoders. Arduino's PID library is used to track the desired speed.
It can also receive commands for desired speeds from Serial2.

## Tests

Tested on an ESP32-WROVER connected to a DRV8833 while receiving commands from a Jetson Nano.

Fixed target speed at 150 rpm:

https://github.com/user-attachments/assets/15d31798-dd34-4917-aaf1-a3f835ccd7e8

Ki too high, causing overshoot (Ki=8):

https://github.com/user-attachments/assets/503d3979-9e19-4a31-a9ac-fd471540c5ee

Linear changing target speeds:

https://github.com/user-attachments/assets/6b8aa7f4-847b-4749-82fc-5ccdda8520d9


