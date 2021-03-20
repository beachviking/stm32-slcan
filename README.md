# stm32-slcan
Simple SLCAN example using an STM32L433, to document my own learnings with respect to STM32 development, USB and CAN bus usage. CAN bus communication is fixed at 10kbits in the code.  The code is loosely based on the SLCAN(LAWICEL) protocol and only provides for a minimal feature set of it.  

I used a Butterfly development module from https://www.tindie.com/products/tleracorp/butterfly-stm32l433-development-board/

I used STM32CubeMX 5.3.0 and Atollic TrueStudio 9.2.0 to do the development. I did not use the Arduino environment that came with the Butterfly module.

Remember to connect a CAN bus tranceiver to interface the pins 6(CANRX) and 7(CANTX).  I used a http://www.ti.com/lit/ds/symlink/sn65hvd230.pdf
