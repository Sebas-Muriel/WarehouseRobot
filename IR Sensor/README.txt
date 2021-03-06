Pi -(I2C)> IR Reader
|  |
|  -(GPIO)> MotorArduino
|  |
|  -(HandShake)> MotorArduino
|

MotorArduino -(UART)> IR Reader

The Pi will have I2C lines connected to the IR Reader Arduino.
    It will also have 5 GPIO lines connected to the MotorArduino for (Left, Right, Up, Down, Stop).

The MotorArduino will be connected to the IR Reader with UART

The handshaking lines are so the masters do not grab data at the same time