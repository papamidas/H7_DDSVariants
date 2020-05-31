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

The following GPIO pins are defined as outputs and can be used for profiling with an oscilloscope:

GPIO User Label | GPIO Port  | Connector Designator and Pin Number
------------ | ------------- | -------------
PROF0        |  PC6          | CN7, Pin 1
PROF1        |  PB15         | CN7, Pin 3
PROF2        |  PB13         | CN7, Pin 5
PROF3        |  PB12         | CN7, Pin 7

###### Nucleo_H7_Simple_DDS_FPU
You may need to first choose "Nucleo_H7_Simple_DDS_FPU Debug" as debug configuration, if no debug configuration exists in the first place.
You may also want to use SWO tracing on Port PB3.
This must be enabled via editing the Debug Configuration->Debugger->Serial Wire Viewer->Enable and setting the core clock to 400 MHz.
After this, the variables and expressions that can be entered in the "Live Expressions" tab are updated while the MCU is running.
It is also possible to select other SWV windows via Window->Show View->SWV; but first the Serial Wire Viewer has to be configured: press the icon with the screw driver and hex wrench on the right side of the console toolbar.
(search for "serial wire viewer atollic" to get more information about how to configure and use the SWV)

After starting the program you should see something like the following output on your serial terminal:
...
f_out[0] = 1000.000001 Hz, f_sample = 10000.000000 Hz, phasereg[0] = 1718068992, phaseinc[0] = 429496730, ampl[0] = 30000, offs[0] = 32767
  f_out[1] = 1000.000001 Hz, f_sample = 10000.000000 Hz, phasereg[1] = 3006559230, phaseinc[1] = 429496730, ampl[1] = 30000, offs[1] = 32767, CPU load = 6.221550%
...

Press 's' and increase the sample rate to 100000 samples/s (press some other key after entering the number, e.g. 'r', in order to resume outputting the DDS status). The CPU load should have risen to about 61 %.
At 200000 samples/s the CPU load should be about 99%. Maybe the MCU even hangs; if that happens, press the Reset key on the NUCLEO board and try a slightly lower number.

###### Nucleo_H7_Simple_DDS_FPU2

The following changes were made successively:

- in the project options: "Tool settings->MCU GCC compiler->Optimization" was changed for none to -Ofast
  This improved the CPU load from 6.22% to 4.6%
- the instruction cache has been enabled via STM32Cube System Core->CORTEX_M7->CPU ICache -> enable
  This improved the CPU load from 4.6% to 3.38%
- The CMSIS DSP library was installed
  (this was not as easy as I thought->the installation via STM32Cube is incomplete and error prone)
  arm_sin_f32() hase been used instead of the standard sin() function in the TIM6 ISR
  This improved the CPU load from 3.38% to 2.118%
- float32_t has been used as a more efficient data type in the TIM6 ISR
  This gave another improvement from 2.118% to 2.057%
- Two fp divisions have been substituted with multiplications in the TIM6 ISR
  -> another small improvement from 2.057% to 2.054%
    


...
f_out[0] = 1000.000001 Hz, f_sample = 10000.000000 Hz, phasereg[0] = 1720158264, phaseinc[0] = 429496730, ampl[0] = 30000, offs[0] = 32767
  f_out[1] = 1000.000001 Hz, f_sample = 10000.000000 Hz, phasereg[1] = 3008648502, phaseinc[1] = 429496730, ampl[1] = 30000, offs[1] = 32767, CPU load = 2.054628%
...

  
