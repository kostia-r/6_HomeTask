Homework # 6: 

Write the "time capsule" message to the SPI Serial Flash SST25VF016 microcircuit. Write each line of the message into a separate 4KB memory block. Start address 0x000000.
Before compiling, you need to set the program operation mode: writing / reading a message, or only reading a previously written message. #define TIME_CAPSULE_WRITE_MODE 0 is responsible for this.
The result of reading the message of the "time capsule" is displayed on UART3. 

UART3 settings: baud rate 115200, 8bit, no parity, 1 stop bit.

Logic analyzer file: Session 0.sal
