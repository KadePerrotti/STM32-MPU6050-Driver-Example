## Overview

This is an example project showcasing usage of of my platform independent [MPU6050 Driver](https://github.com/KadePerrotti/MPU6050-Driver) on an STM32L476RG. Inside of Core/Src/main.c the MPU6050 is initialized and tested using test functions from the driver. The test results are printed over uart. Test functions include demos for reading all accelerometer and gyroscope axes individually or from the FIFO.

## STM32 Setup with CubeIDE
1. Create your STM32 project
2. Clone the [MPU6050 Driver](https://github.com/KadePerrotti/MPU6050-Driver) into the directory you want (I chose `Drivers/`)
3. Add that directory to your include paths. In CubeIDE the setting to add a new include path is `Project -> Properties -> C/C++ Build -> Settings -> MCU/MPU GCC Compiler -> Include paths`.
4. Include the required files in your project:
```c
#include "MPU6050.h"
#include "REG_OPTIONS.h"
#include "TEST_FUNCTIONS.h" //not necessary, only include if you want to run tests
```
5. The build_string functions in TEST_FUNCTIONS use sprintf to include floats in the strings. Make sure your linker flags include `-u _printf_float` if you're trying to use the build_string functions. In CubeIDE the linker flags are located at `Project -> Properties -> C/C++ Build -> Settings -> MCU/MPU GCC Linker -> Include paths` inside of the textbox named "All options".