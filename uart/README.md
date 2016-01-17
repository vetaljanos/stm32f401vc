# UART/DMA/LEDS

## In this project implemented:

* Receive UART data in 1 byte buffer using Circular DMA. 
* Collect entered by UART data to string buffer. String buffer length = 253 byte. One byte reserved for \n, second for string end marker (0)
* After \n or \r in UART string send current command pointer to queue
* Task to process queue strings
* Detect ping command to answer argument string back to UART
* Detect on/off commands to enable/disable PIN number passed as argument
	
## Examples of command:

request: ping 12345
response: 12345

request: on 14
response: red LED enabled

request: off 14
response: red LED disabled

## Configuration

### UART

* PA2 - tx. Connected to rx of my FT232 driver. Uses to transmit data to computer
* PA3 - rx. Connected to tx of my FT232 driver. Uses to receive UART data
* GND - FT232 GND. Must be connected to make UART working

### LEDS

* Green LED (PIN_12) blinks with 1 sec frequency to indicate OS is working