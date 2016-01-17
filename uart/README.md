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
