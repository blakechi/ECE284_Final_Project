# ECE 284 Final Project

## Introduction
This project is a prototype of long-term elder care system that successfully measures heart rates and communicates between heart rate measurement device and multiple people through AWS.
 ![](/asset/system_architecture.png)
## Pre-requirement
* a AWS account
* a CC3200 Launch Pad
* [Code Composer Studio](http://www.ti.com/tool/CCSTUDIO-MSP)
> Note: If you just want to know how to communicate with AWS, all you need is above.
* a MCP3001
* an Opt101 
* a LM324-N
* a IR908-7C-F
* multiple wires, resisters, and capacitors

## Implementation
> Note: If you just want to know how to communicate with AWS, you can jump to 2.

1. Circuit: Please follow the circuit down below and data sheets of CC3200 Launch Pad, MCP3001, Opt101, LM324-N, and IR908-7C-F for more details.
 ![](/asset/circuits.png)

2. AWS: This project uses the servive from AWS including PinPoint, Lambda, Shadow (from IoT Core), please see [this](https://aws.amazon.com) and build the connections as the topest image.

## Execution
1. Clone this repository
2. Use UniFlash to flash key and certificates
3. Open Code Composer Studio
4. Connect CC3200 Launch Pad to your computer
5. click `build` -> `debug`
6. Open UART output terminal
7. click `run`

## License
MIT
