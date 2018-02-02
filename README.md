# PicoWAN SDK


## Synopsis

This SDK allows to create embedded applications compatible with the **PicoWAN network**. Moreover, thanks to its multi-MAC layer, an application designed with this SDK will also be compatible as-is with any **LoRaWAN network**.
It has been primarily designed for the **STM32L072 and LoRa (B-L072Z-LRWAN1) development board** from STMicroelectronics, as well as the **Archos PicoSmartTag**, but can be easily extended to support additional devices based on the **STM32L1/L0 MCU families** coupled with an **SX1276 Transceiver** (like the **CMWX1ZZABZ** module from Murata).

A sample project named **HelloPico** is available as starting point. It mainly allows to send a PicoWAN message when you push a button on your target device.


## Prerequisites

To use this SDK, a Unix environment with the **GNU Make** tool available is recommended (Linux 64-bit preferred, but it also works on Mac OS and Windows with Cygwin). If you want to use OpenOCD (provided in the SDK package), **libusb v1.0** also needs to be installed.  
As an alternative, the various projects can also be loaded in IAR or TrueSTUDIO.


## Installation

### Binary package

The easiest and recommended way to install the PicoWAN SDK is to download the latest release package for your operating system in the *Release* section of the GitHUB repository. It includes all the required tools and dependencies (ST libraries, GNU ARM Embedded Toolchain, OpenOCD and PicoFlash), whose versions have been tested and validated all together. The SDK release package is available for Linux, Windows (Cygwin required) and Mac OS.  
If you plan to use IAR or TrueSTUDIO, the operating system dependent tools are not needed (only the SDK sources and the ST libraries are required). In such a case, just download and extract any SDK release package.

#### Linux

On Ubuntu/Debian:

* Launch a terminal
* Install the required dependencies
```
$ sudo apt-get update
$ sudo apt-get install make libusb-1.0-0 libusb-0.1-4
```
* Download and unpack the latest release of the PicoWAN SDK from Github
```
$ mkdir -p ~/PicoWAN-SDK
$ cd ~/PicoWAN-SDK
$ tar xzf <path_to_the_downloaded_sdk_package>
```
* Enable the required Udev rules for OpenOCD and PicoFlash
```
$ sudo cp ~/PicoWAN-SDK/utils/openocd/share/openocd/contrib/60-openocd.rules /etc/udev/rules.d/
$ sudo cp ~/PicoWAN-SDK/utils/PicoFlash/60-picoflash.rules /etc/udev/rules.d/
$ sudo udevadm trigger
```


#### Windows (Cygwin)

* Download and install the STLink driver from STMicroelectronics [website](http://www.st.com/content/st_com/en/products/development-tools/software-development-tools/stm32-software-development-tools/stm32-utilities/stsw-link009.html)
* Download Cygwin 32-bit (even on 64-bit systems) from its [website](http://www.cygwin.com)
* Follow the installation procedure of Cygwin, and make sure to select the required dependencies:
    * Libs -> libusb 1.0
    * Devel -> make
* Download the latest release of the PicoWAN SDK from Github in 'C:\ cygwin \ home \ <your_username> \'
* Launch Cygwin
* Unpack the PicoWAN SDK
```
$ mkdir -p ~/PicoWAN-SDK
$ cd ~/PicoWAN-SDK
$ tar xzf <path_to_the_downloaded_sdk_package>
```

Please note that if the PicoWAN SDK as been properly downloaded in 'C:\ cygwin \ home \ <your_username> \', <path_to_the_downloaded_sdk_package> will be something like '../PicoWAN-SDK_v<version>_win32.tar.gz'.


#### Mac OS

* Install the latest version of Xcode from the App Store
* Download and install MacPorts from its [website](https://www.macports.org)
* Launch iTerm
* Install the required dependencies
```
$ sudo port install libusb make
```
* Download and unpack the latest release of the PicoWAN SDK from Github
```
$ mkdir -p ~/PicoWAN-SDK
$ cd ~/PicoWAN-SDK
$ tar xzf <path_to_the_downloaded_sdk_package>
```


### Git repository

Alternatively, you can install the latest revision of the PicoWAN SDK directly from the **Git repository**. In that case you will need to create the SDK folder tree manually as follow:

* Install Git, Libusb 1.0, the GNU Compiler Collection (GCC), and the GNU Make tool using the package manager of your operating system
* Create the empty folder tree
```
$ mkdir -p ~/PicoWAN-SDK/toolchain
$ mkdir -p ~/PicoWAN-SDK/utils
$ mkdir -p ~/PicoWAN-SDK/libs
```
* Clone the **PicoWAN SDK** repository from GitHub
```
$ cd ~/PicoWAN-SDK
$ git clone https://github.com/picowan/sdk.git firmware
```
* Clone the **PicoFlash** repository from Github
```
$ cd ~/PicoWAN-SDK/utils
$ git clone https://github.com/picowan/picoflash.git PicoFlash
```
* Build **PicoFlash**
```
$ cd ~/PicoWAN-SDK/utils/PicoFlash
$ make
```
* Download the **STMCube libraries** for STM32L0 and STM32L1 from STMicroelectronics [website](http://www.st.com)
* Install them in the *PicoWAN-SDK/libs* folder and adjust the **STLIBROOT** variables in *PicoWAN-SDK/firmware/projects/projects.gmk* accordingly
* Download the **GNU ARM Embedded Toolchain** for your operating system from its [website](https://developer.arm.com/open-source/gnu-toolchain/gnu-rm/downloads)
* Install it in the *PicoWAN-SDK/toolchain* folder and adjust the **TOOLCHAIN** variable in *PicoWAN-SDK/firmware/projects/projects.gmk* accordingly
* Install **OpenOCD** using the package manager of your operating system (or build it using the release package available on its [website](https://sourceforge.net/projects/openocd))
* Adjust the **OPENOCD_BIN** variable *PicoWAN-SDK/firmware/projects/projects.gmk* accordingly (leave it empty if you use the OpenOCD package provided by your operating system)


## Usage

### Unix environment

Just type `$ make` in the *firmware* folder to see the available targets and parameters:

```
$ make [BOARD=<board>] <target>                                            to build <target>
$ make [BOARD=<board>] <target>-flash                                      to build and flash <target>
$ make [BOARD=<board>] buildall                                            to build all targets
$ make [BOARD=<board>] CREDENTIALS="<credentials>" FlashPicoCred-flash     to inject the PicoWAN credentials into the board
$ make <target>-clean                                                      to clean <target>
$ make clean                                                               to clean all targets
```

Several boards are available:
* **discovery_stm32l1**: the 32L152CDISCOVERY development board from STMicroelectronics connected to the SX1276RF1JAS board from Semtech
* **picotag**: the Archos PicoSmartTAG
* **murata_sychip**: Murata's development board for their CMWX1ZZABZ module
* **murata_discovery**: the STM32L072 and LoRa (B-L072Z-LRWAN1) development board from STMicroelectronics (based on the CMWX1ZZABZ module from Murata)
* **murata_module**: the CMWX1ZZABZ module from Murata alone (should be the starting point of any board based on this module)
* **nucleo**: the STM32 Nucleo pack (P-NUCLEO-LRWAN1) from STMicroelectronics
* **picoshield**: the Archos PicoShield (based on the Murata's CMWX1ZZABZ module)

**HelloPico**, **ATModem**, **FlashKeys**, and **FlashPicoCred** will be the only targets available at first. Once your own projects added to the *projects* folder, they will be listed there as well.

For instance, to build the **HelloPico** project for the **PicoSmartTAG** board, just type:
```
$ make BOARD=picotag HelloPico
```


### IAR/TrueSTUDIO

For each SDK project, ready to use IAR and TrueSTUDIO projects/workspaces are available for the **Murata Discovery** and **PicoSmartTAG** boards. They are located in the *IAR* and *TrueSTUDIO* sub-folders of the project.


## Flashing credentials

To establish a connection with the PicoWAN network, a device needs a valid set of credentials (PicoWAN keys and IDs). While these credentials need to be injected into the device only once, it can be done at anytime if they need to be changed. Please note that doing so erases the application (project) currently loaded, so once the credentials have been flashed, the desired application needs to be flashed again.

### Unix environment

In order to flash a particular set of credentials, just type:
```
$ make [BOARD=<board>] CREDENTIALS="<credentials>" FlashPicoCred-flash
```
Where <credentials> should be replaced by something like **00000C96,0016DC0100000C96,6E4ACC054D1B7857E1C9293B6413CDA9,C3A85734166A18734B5BB7184FAC0F99,57**.


### IAR/TrueSTUDIO

If you are using IAR or TrueSTUDIO, you need to import the corresponding project located in *firmware/projects/FlashPicoCred*, and insert the credentials at the beginning of the *main.c* file where requested. Then you just need to build, flash, and run the project.


### LoRaWAN credentials

If you need to inject LoRaWAN credentials, open the FlashKeys project, and fill in the keys and IDs (for both, PicoWAN and LoRaWAN) where requested, making sure to carefully follow the requested endianness. Then you just need to build, flash, and run the project.


## Folder organization

```
|___firmware                       the "Home" folder in which you should be to build the various projects
| |___common                       common code
| | |___mac                        multi-MAC layer
| | | |___lora-mac                 LoRaWAN implementation
| | | |___pico-mac                 PicoWAN related headers
| | | |___lib                      PicoWAN libraries for GCC and IAR
| | |
| | |___os                         OS related files
| | |___radio                      radio driver
| | |___sensors                    drivers for various sensors
| | |___stm32
| | | |___STM32L0                  STM32L0 related files and boards definition (boards.h)
| | | |___STM32L1                  STM32L1 related files and boards definition (boards.h)
| | |
| | |___utils                      helpers and algorithms independent from the hardware
| |
| |___projects                     the various projects that can be built (and where your own projects must go)
|   |___ATModem                    the source of the ATModem firmware
|   |___FlashKeys                  a project to modify and execute on the board (only once) in order to flash the various keys and IDs (PicoWAN and LoRaWAN)
|   |___FlashPicoCred              a project to execute on the board only once in order to flash all the PicoWAN credentials (binary with the right credentials generated from the command line)
|   |___HelloPico                  a simple project demonstrating the use of the MultiMac layer
|
|___utils                          OpenOCD and PicoFlash
|___libs                           ST L0/L1 libraries
|___toolchain                      the GNU ARM toolchain
```


## Porting the PicoWAN stack to other Hardware/Software environments

### Operating System Abstraction Layer (OSAL)

The **multi-MAC layer** (PicoWAN + LoRaWAN stacks), located in the *common/mac* folder, has been designed to be self-contained and as much as possible independent from the Operating System on which it is running.
Because of this design, it relies on an Operating System Abstraction Layer (**OSAL**), and can be the easily integrated on top of various software environments.  
If you need to run the **muti-MAC layer** on top of another environment (FreeRTOS, Linux, ...), you just need to integrate the *common/mac* folder in it, and adapt the **OSAL**. This layer is well documented, and must be carefully implemented in order to have a properly working **multi-MAC-layer**. 
To do so, rename the file called **osal.c.template** into **osal.c**, and implement the empty functions using the right OS and Radio calls.


### Hardware requirements

To properly run the PicoWAN and LoRaWAN stacks, the following hardware requirements need to be met:
 * Radio: an **SX1276 Tranceiver** from Semtech. Compared to the SX1272, the SX1276 allows to use a transmission bandwidth of 62.5kHz. This is mandatory for PicoWAN network.
 * CPU: PicoWAN has been ported to STM32L1 and STM32L0 targets, namely STM32L15x and STM32L07x series of CPU. There is, however, no strong dependency on STM32 chips.
 * Hardware timer: PicoWAN has a more fine grained timing requirement than LoRaWAN, a hardware timer with at least an accuracy of **100us** is needed.
 * Crystal: using a **TCXO** is highly recommended. Without a TCXO, issues might occur with SF12 at a power above 14dbm. These issues could be mitigated through board layout optimizations.
 * RF switch: the SX1276 offers 2 PAs, one medium power and high efficiency, one high power with lower efficiency. If a **3-way RF switch** is used, both PAs can be used.
 * Flash and RAM: less than **40kB flash** and less than **5kB RAM** are needed.
 * Inputs/outputs:
   * GPIO input/output to control the RF switch and query the SX1276 DIO lines
   * GPIO IRQ on the DIO0-2 lines
   * SPI half duplex communication with the SX1276


## History

v2.0:
* Initial public version of the SDK

__  
Copyright (c) 2018 Archos S.A.
