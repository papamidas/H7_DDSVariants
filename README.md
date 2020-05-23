This repository contains a collection of several projects for the

NUCLEO-STM32H743ZI development board for the STM32H743ZI MCU

The projects in this collection of STM32CubeIDE-Projects were compiled with STM32CubeIDE V1.3.1.

Each project contains a slightly different implementation of the well-known DDS algorithm.
Beginning with a first naive approach, each following project tries to improve on the effiency of the code in some way.

In order to get a measure for this efficiency, a simple CPU usage algorithm was implemented:
Before starting the interrupts for DAC output, a simple background counter runs for a given time, giving a reference count.
The same counter later runs in the background, but this time slower due to the interrupts.
The ratio of these two values is then output as "CPU load", measured in percents.

All implementations output signals on two channels, which are

DAC1 (PA4), on connector CN7, Pin 14

DAC2 (PA5), on connector CN7, Pin 10

Both channels are output with the same sampling frequency, which is programmable.
Each channel can output its own frequency, amplitude and offset.
Both channels can be synchronized, which means that the second channel is forced to output the same frequency and phase as the first channel.
After synchronisation, the second channel can be phase shifted relative to the first channel

A primitive user interface is available via the virtual COM port of the NUCLEO board.
Use a serial terminal like TERATERM or PUTTY on Windows machines and set the serial port parameters to

115200 Bd, no parity, 1 stop bit, 8 data bits

Several One-letter-commands are available:

Command | Description
------------ | -------------
'X' | pressing this button changes to channel 0 input mode
'Y' | pressing this button changes to channel 1 input mode
'F' | pressing this button changes to frequency input mode of the selected channel
'A' | pressing this button changes to amplitude input mode of the selected channel
'O' | pressing this button changes to offset input mode of the selected channel
'Z' | pressing this button synchronizes channel 2 to channel 1
'D' | pressing this button changes to phase jump input mode of channel 2 in 32-bit phase increments
'J' | pressing this button changes to phase jump input mode of channel2 in degrees
'S' | pressing this button changes to sample frequency input mode (sample frequency is the same for both channels)
'P' | pressing this button changes to sample period input mode (reciprocal value of the sample frequency; same for both channels)
other | pressing any other letter resumes cyclic output of the status of both DDS channels

After cloning the git repository to your PC, start STM32CUBEIDE and select File->Open Projects from File System->Import Source: Directory, and select one of the project directories.

In the project explorer, right click on the project and select "Build project".

The first build will show a lot of warnings during compilation of drivers etc. This due to the compiler flag [-Wunused-parameter]. Just ignore these warnings.
