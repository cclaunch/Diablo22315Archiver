Diablo disk drive model 31, in its standard density version, can read or write to the 2315 disk cartridges interchanngeably with the disk drives in the IBM 1130 system 

This project exploits an FPGA board with a custom interface board to control the Diablo disk drive. It will read the contents of the disk cartridge into RAM on the FPGA board. 

It then uploads the contents over the USB serial link so that the contents can be archived. The software will take the data streaming over the USB link and create disk files 
in the format used with the IBM 1130 simulator programs (and therefore with the Virtual 2315 Cartridge Facility to access them on a real 1130 system. 

The PCB connects to the FPGA board, provides power to the FPGA and the logic on the PCB, and connects to the Diablo disk drive using an IDC 50 pin cable.

A PCB is available to create the Head Adapter Tool used in aligning a Diablo 31 standard density drive using the CE Cartridge
