# Servo Test Tool base on CH32V003

* [Introduction](https://github.com/TianpeiLee/ServoTestTool#Introduction)<br>
* [Mode](https://github.com/TianpeiLee/ServoTestTool#Mode)<br>


## Introduction

This is a servo testing tool program that can be directly compiled and downloaded through MRS.<br>
It can output 50Hz PWM signals, Dshot300 signals, and Dshot600 signals for testing servo or electronic speed controllers (ESC).
The controller is a 32-bit RISC-V chip with high cost-effectiveness,which can be detailed in the [schematic diagram](https://github.com/TianpeiLee/ServoTestTool/blob/main/ServoTest_SCH.pdf).<br>

PCB is: <br>
<img src="image/photo.jpg" alt="photo" style="zoom:50%;" />

## Mode

### PWM Mode
PWM mode has three sub modes: manual mode, middle mode, and automatic mode.
*manual mode*
The default mode for powering on, in which the "MODE/MAN" light remains on and the other two lights turn off.
At this point, the output of the signal follows the resistance value of the resistor(50Hz,duty 1ms-2ms).

*middle mode*
Short press the button, it will switch to the middle mode, and at this time, the PWM waveform with a fixed output duty cycle of 1.5ms will be output.
In this mode, the "MID" LED remains on and the other two LEDs turn off

*automatic mode*
Short press the button again, it will switch to the automatic mode, and the duty cycle of the output signal will continuously scan from 1ms to 2ms.
In this mode, the "AUTO" LED remains on and the other two LEDs turn off


### Dshot300
Long press the button to switch to Dshot300 mode. At this time, the ADC collects the resistance signal and outputs it after Dshot300 encoding.
In this mode, the "MODE/MAN" LED flashes slowly and the other two LEDs turn off.


### Dshot600
Long press the button again to switch to Dshot600 mode. At this time, the ADC collects the resistance signal and outputs it after Dshot600 encoding.
In this mode, the "MODE/MAN" LED flashes quickly and the other two LEDs turn off.
