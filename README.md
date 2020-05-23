This repository contains a collection of several projects for the

NUCLEO-STM32H743ZI development board for the STM32H743ZI MCU

The projects in this collection of STM32CubeIDE-Projects were compiled with STM32CubeIDE V1.3.1.

Each project contains a slightly different implementation of the well-known DDS algorithm.
Beginning with a first naive approach, each following project tries to improve on the effiency of the code in some way.

In order to get a measure for this efficiency, a simple CPU usage algorithm was implemented:
Before starting the interrupts for DAC output, a simple background counter runs for a given time, giving a reference count.
The same counter later runs in the background, but this time slower due to the interrupts.
The ratio of these to values is then output as "CPU load", measured in percents.

All implementations output signals on two channels, which are

DAC1 (PA4), on connector CN7, Pin 14
DAC2 (PA5), on connector CN7, Pin 10

A very simple user interface is available on the virtual COM port of the NUCLEO board.


