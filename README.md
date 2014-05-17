bras_robot_e407_lcd4ds
======================

firmware for an interface card between PLC and robotic arm
This is a robotic arm command project based on olimex E407 stm32f4 board.

This is for a PLC training curses, where goals is to use all PLC output : logical (true or false), analog,
jbus (ascii, rtu, tcp), to communicate with a robotiv arm.

The project use chibios, freemodbus, a few file from stm library, and all sources are together here in the goal 
to have a static source code analysis by coverity.

After first successfull courses, hardware will be made open source too.

