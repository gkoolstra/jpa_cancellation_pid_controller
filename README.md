# JPA cancellation PID controller

This repository contains Arduino code built on top of an existing Arduino PID library, with the main 
purpose of minimizing one or two controller input signals from a standard microwave interferometry setup. 
The controller outputs (either one or two) control a voltage controlled attenuator and a voltage controlled phase shifter. 

## Hardware
The necessary hardware for this project is listed below
- Arduino Due, connected to USB port of the computer: this Arduino is better suited for PID control since it has two dedicated DACs outputting a maximum voltage of 3.3 V.
- Voltage controlled phase shifter: we are using the following evaluation board [Qorvo CMD297P34](https://www.custommmic.com/cmd297p34-phase-shifter/).
    This board works for microwave signals in the range of 5-18 GHz and has a modest insertion loss of 3.2 dB. 
- Voltage controlled attenuator: we are using the following evaluation board [IDT F2250 EVB](https://www.mouser.com/datasheet/2/698/IDT_F2250_DST_20170130-1712745.pdf). 
    This board is specified for microwave signals up to 6 GHz and has a tuning range of up to -35 dB with just a modest control voltage of 2V.
- Arduino Due shield with LD1602 display. The shield board layout is shown below. For more information on the contents and functioning of this board, 
please check the section below, and click [here](https://oshpark.com/shared_projects/RkL3pm7k) to order PCBs.

![PCB layout](board_layout.png). 

## Uploading the code to the Arduino
### Getting the Arduino IDE set up
The Arduino code has just two dependencies: `PID` and `LiquidCrystal`. `LiquidCrystal` should come with the Arduino IDE, but the `PID` library needs to be installed manually. 
To get started, first clone the `PID` Arduino library from [this](https://github.com/br3ttb/Arduino-PID-Library) link. If you're using the 
Arduino IDE, it's easiest to download the `.zip` file and add the library to Arduino from the menu. 

If you've never connected to an Arduino Due before, you may have to install board software as well. This is also easy in the 
Arduino IDE. Just search for "Due" and install the board software.

### Connecting the board
The Arduino Due has two USB ports: a native USB port and a programming port. To upload the sketch, make sure you connect the
Arduino to the programming port, which is located closest to the DC plug. Once connected open the Arduino IDE's Serial monitor. 
The default state after uploading the code is a voltage ramp with parameters that are controlled in the top part of the script.

## Controlling the output

More details to come...

## Using the supplied python driver

More details to come...

## Circuit details
Below we show part of the schematic which links the inputs to the outputs. Firstly, there are two pairs of inputs (lower left corner), 
which have a single stage low-pass RC-filter and then connect to Analog inputs 0 and 1 of the Arduino Due. The controller
output are DAC0 and DAC1 of the Arduino Due (output range 0.55 - 2.75 V), which pass through a summing amplifier, RC-filter and a buffer before arriving at the 
output on the top of this schematic. The summing amplifier aims to scale the Arduino DAC output to 0.0-3.3 V. 
![PCB layout](circuit_schematic.png).

### Circuit component list
All SMD resistors and capacitors are 1206 for convenience of soldering, unless noted otherwise.
- 4x [resistor, 100 kΩ](https://www.digikey.com/product-detail/en/yageo/RC1206FR-07100KL/311-100KFRCT-ND/731439), ±1%, ¼ W
- 4x resistor, 150 kΩ, ±1%, ¼ W
- 2x resistor, 1.3 kΩ, ±1%, ¼ W
- 6x resistor, 6.8 kΩ, ±1%, ¼ W
- 2x [Rp resistor trimmer](https://www.digikey.com/product-detail/en/bourns-inc/3296W-1-201LF/3296W-201LF-ND/1088049), 200 Ω
- 4x [capacitor, 100 nF](https://www.digikey.com/product-detail/en/kemet/C1206C104M5RACTU/399-1248-1-ND/411523), ceramic
- 4x capacitor, 10 nF, ceramic
- 2x TLC272 opamp (includes 2 per package)
- 2x [four terminal headers](https://www.digikey.com/products/en/connectors-interconnects/terminal-blocks-wire-to-board/371?k=&pkeyword=&sv=0&pv89=11374&sf=0&FV=-5%7C21372%2C-8%7C371&quantity=&ColumnSort=0&page=1&pageSize=25)
- 1x [LCD1602 display](https://www.digikey.com/product-detail/en/lumex-opto-components-inc/LCM-S01602DTR-M/67-1781-ND/469805)

