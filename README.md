# ROS Base control using STM-IDF
## Overview

This firmware implements base control for STM32 to communicate with ROS over rosserial protocol using STM-IDF. All you need to build this firmware are: 

- GNU ARM Embedded Toolchain to compile code.
- Build tool - CMake to build application.
- STM-IDF contains API for STM32 series.

Follow this [link](https://github.com/phonght32/stm-idf) to know more about STM-IDF. 

## Install

This repository contains many submodules, you need to install all of them to use probably.

```
git clone --recursive git@github.com:phonght32/ros-stm32-base-control.git
```

## How to use

Build application

```
make build all
```

Flash over ST-LinkV2

```
make flash_stlink
```

Flash over USART 

```
make flash_usart 
```

View log output

```
make monitor
```

## Problem

For any problems, you can create issues or contact our e-mail.

## Contact 

E-mail: thanhphongho1998@gmail.com 