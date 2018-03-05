# PicoWAN/LoRaWAN ATModem firmware


## Synopsis

The ATModem firmware allows to send and receive PicoWAN and LoRaWAN messages using AT commands. It has been primarily designed to be fully compatible with boards based on the CMWX1ZZABZ module from Murata (for instance, the B-L072Z-LRWAN1 Discovery kit from STMicroelectronics). However, it is also compatible, keeping some limitations in mind, with any board supported by the PicoWAN SDK.


## Prerequisites

The reader is assumed to be familiar with the LoRaWAN and PicoWAN specifications.  
The binary release of the ATModem firmware expects the following electrical connections (the reference design from Murata can be found [here](https://wireless.murata.com/RFM/data/type_abz.pdf)):

Pin     | Function
--------|----------------------
PA12    | Connected to VDD_TCXO
PA2     | USART_TX
PA3     | USART_RX
PA4-5   | NC
PA8-11  | NC
PB2     | NC
PB5-9   | NC
PB12-15 | NC
PB12-15 | NC
PH0-1   | NC


## Conventions

* All commands are case sensitive.
* All AT commands sent must end with “\r\n”.
* UART configuration is 9600 baud, 8 data bits, 1 stop bit and no parity. This cannot be adjusted.
* Any space in the string after "=" is ignored.
* Settings marked "persistent" are stored in flash, and thus are retained when the device is powered off.
* All commands return at least OK or ERROR.


### Command format

Get data:

    AT+<command>?

Set data:

    AT+<command>=<data>

### Response format

Data:

    +COMMAND: DATA

Command succeeded:

    OK
    
Command failed:

    ERROR


## Description of the AT commands

### Commands summary

Command        |    Description
---------------|------------------------------------------------------------------------------------
AT+HELP        |    Displays the AT commands available
AT+REBOOT      |    Reboots the device
AT+MAC         |    Chooses between PicoWAN and LoRaWAN
AT+MODE        |    Selects the network access mode
AT+JOIN        |    Joins the chosen network
AT+CHECKNET    |    Checks if the network is available
AT+DEVADDR     |    Sets/gets the device address for the chosen network and access mode (persistent)
AT+DEVEUI      |    Sets/gets the device unique identifier (persistent)
AT+APPEUI      |    Sets/gets the application unique identifier (persistent)
AT+NWKSKEY     |    Sets/gets the network session key for the chosen network (persistent)
AT+APPSKEY     |    Sets/gets the application session key for the chosen network (persistent)
AT+APPKEY      |    Sets/gets the application key (persistent)
AT+PICOCRED    |    Sets all the PicoWAN credentials at once (persistent)
AT+CLASS       |    Selects the class of the device
AT+PORT        |    Selects the communication port
AT+LORARX2DR   |    Sets/gets the expected data-rate for the second LoRaWAN reception window
AT+UTX         |    Sends an unconfirmed message
AT+CTX         |    Sends a confirmed message


### Commands detail

#### AT+HELP

Displays the AT commands available.

Format:

    AT+HELP


#### AT+MAC

Chooses and initializes the MAC to use. The choices are PicoWAN (default) and LoRaWAN.

Format:

    AT+MAC=<mac>

Returns:

    +MAC=<mac>

*mac*: "PICO" for PicoWAN, "LORA" for LoRaWAN


#### AT+MODE

Selects the network access mode. The choices are Activation By Personalisation (default) or Over The Air Activation.  
Once the access mode is changed (and even in ABP mode), a join (AT+JOIN) must be performed to start the actual activation procedure.

Format:

    AT+MODE=<mode>

Returns:

    +MODE=<mode>

*mode*: "ABP" for Activation By Personalisation, "OTAA" for Over The Air Activation


#### AT+JOIN

Joins the chosen network.  
A join in ABP mode will register the session keys with the MAC, while a join in OTAA mode will send a join request and wait for a join accept. You first need to select the desired MAC, device class, and network access mode with AT+MAC, AT+CLASS, and AT+MODE, then you can join the network to establish the communication.

Format:

    AT+JOIN


#### AT+CHECKNET

Checks if the network is available.  
This command returns if a gateway is available nearby.

Format:

    AT+CHECKNET

Returns:

    +CHECKNET: AVAILABLE
    +CHECKNET: UNAVAILABLE


#### AT+DEVADDR

Sets or gets the device address (32 bits non-unique identifier) for the network currently selected.  
In ABP mode, this command writes or reads the device address to/from the flash memory (persistent), while in OTAA mode, it allows to retrieve the device address assigned by the network. The right MAC and access mode needs to be selected before sending this command.

Format:

    AT+DEVADDR=<device_address>

Returns:

    +DEVADDR: <device_address>

*device_address*: 32 bits device address in hexadecimal


#### AT+DEVEUI

Sets or gets the device unique identifier (64 bits unique identifier shared between LoRaWAN and PicoWAN).  
This command writes or reads the device unique identifier to/from the flash memory (persistent). You must set a device unique identifier using this command in order to use the OTAA mode.

Format:

    AT+DEVEUI=<device_eui>

Returns:

    +DEVEUI: <device_eui>

*device_eui*: 64 bits device unique identifier in hexadecimal


#### AT+APPEUI

Sets or gets the application unique identifier (64 bits unique identifier shared between LoRaWAN and PicoWAN).  
This command writes or reads the application unique identifier to/from the flash memory (persistent). You must set an application unique identifier using this command in order to use the OTAA mode.

Format:

    AT+APPEUI=<application_eui>

Returns:

    +APPEUI: <application_eui>

*application_eui*: 64 bits application unique identifier in hexadecimal


#### AT+NWKSKEY

Sets or gets the network session key (16 bytes key) for the network currently selected.  
This command writes or reads the network session key to/from the flash memory (persistent). You must set a network session key using this command in order to use the ABP mode, and the network access mode must be set to ABP in order to use this command.

Format:

    AT+NWKSKEY=<network_session_key>

Returns:

    +NWKSKEY: <network_session_key>

*network_session_key*: 16 bytes network session key in hexadecimal


#### AT+APPSKEY

Sets or gets the application session key (16 bytes key) for the network currently selected.  
This command writes or reads the application session key to/from the flash memory (persistent). You must set an application session key using this command in order to use the ABP mode, and the network access mode must be set to ABP in order to use this command.

Format:

    AT+APPSKEY=<application_session_key>

Returns:

    +APPSKEY: <application_session_key>

*application_session_key*: 16 bytes application session key in hexadecimal


#### AT+APPKEY

Sets or gets the application key (16 bytes key shared between LoRaWAN and PicoWAN).  
This command writes or reads the application key to/from the flash memory (persistent). You must set an application unique identifier using this command in order to use the OTAA mode.

Format:

    AT+APPKEY=<application_key>

Returns:

    +APPKEY: <application_key>

*application_key*: 16 bytes application key in hexadecimal


#### AT+PICOCRED

Sets all the PicoWAN credentials at once.  
This commands writes in the flash memory all the keys and IDs (credentials) required to establish a communication using the PicoWAN network in ABP mode (persistent). The credentials follow a particular format, and must be copied/pasted from the PicoWAN developer console.

Format:

    AT+PICOCRED=<device_address>,<device_eui>,<application_session_key>,<network_session_key>,<crc>

Returns:

    +DEVADDR: <device_address>
    +DEVEUI: <device_eui>
    +APPSKEY: <application_session_key>
    +NWKSKEY: <network_session_key>

*device_address*: 32 bits device address in hexadecimal  
*device_eui*: 64 bits device unique identifier in hexadecimal  
*application_session_key*: 16 bytes application session key in hexadecimal  
*network_session_key*: 16 bytes network session key in hexadecimal  
*crc*: 1 byte CRC


#### AT+CLASS

Selects the class of the device.  
For both networks, only the class A (default) and C are available.

Format:

    AT+CLASS=<class>

Returns:

    +CLASS: <class>

*class*: "A" for class A, "C" for class C


#### AT+PORT

Selects the communication port.  
Its value should be between 1 and 223. The value 0 is used to send MAC commands. The communication port is only used in LoRaWAN, and is ignored in PicoWAN.

Format:

    AT+PORT=<port>

Returns:

    +PORT: <port>

*port*: communication port in decimal [1 - 223]


#### AT+LORARX2DR

Sets or gets the expected data-rate for the second LoRaWAN reception window.  
This has no effect in PicoWAN.

Format:

    AT+LORARX2DR=<datarate_index>

Returns:

    +LORARX2DR: <datarate_index>

*datarate_index*: data-rate index (table below)

Data-rate index | Characteristics
----------------|---------------
0               | SF12 - BW125
1               | SF11 - BW125
2               | SF10 - BW125
3               | SF9  - BW125
4               | SF8  - BW125
5               | SF7  - BW125
6               | SF7  - BW250


#### AT+UTX

Sends an unconfirmed message.  
There will be no acknowledge sent by the network, thus no guarantee the message has been received by the cloud. However, you can still receive a downlink once the message has been sent.

Please note:
* a debug value between -1 and -8 will be displayed if the command fails (this value corresponds to the mac_status_t structure in mac.h)
* the payload is expected as a raw binary byte stream

Format:

    AT+UTX=<payload_length>,[timeout]
    <payload>

*payload_length*: length of the payload
*timeout*: optional UART time-out for the payload in milliseconds (default is 1 second)
*payload*: payload in binary


#### AT+CTX

Sends a confirmed message.  
An acknowledge will be sent by the network, meaning you have the guarantee the message has reached the network.

Please note:
* a debug value between -1 and -8 will be displayed if the command fails (this value corresponds to the mac_status_t structure in mac.h)
* the "OK" response will be returned once the acknowledge received
* the payload is expected as a raw binary byte stream

Format:

    AT+CTX=<payload_length>,[nb_retries],[timeout]
    <payload>

*payload_length*: length of the payload
*nb_retries*: optional number of retries (default is 2)  
*timeout*: optional UART time-out for the payload in milliseconds (default is 1 second)
*payload*: payload in binary
            
Please note:
* the nb_retries field must to be set if the timeout field needs to be changed

## Examples

Assuming the proper credentials have been configured, to send several confirmed frames in PicoWAN (class A, ABP) you need to do:

    AT+MAC=PICO
    AT+CLASS=A
    AT+MODE=ABP
    AT+JOIN
    AT+CTX=10
    abcdefghij
    AT+CTX=10
    klmnopqrst
    AT+CTX=6
    uvwxyz

and to send several unconfirmed frames in LoRaWAN (class A, OTAA):

    AT+MAC=LORA
    AT+CLASS=A
    AT+MODE=OTAA
    AT+JOIN
    AT+UTX=10
    abcdefghij
    AT+UTX=10
    klmnopqrst
    AT+UTX=6
    uvwxyz


## History

v1.0:
* Initial version

__  
Copyright (c) 2018 Archos S.A.

