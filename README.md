Using this script requires first downloading and installing the OpenMV firmware project that runs on a Linux computer. 
The Linux computer will also need to have Microsoft Visual Code installed.
A related repo contains a number of source files that need to be modified from the main fork of the OpenMV project. When the Open MV project is 
compiled it will first call a bootloader, then main.c, and from main.c it will run this BootFDT.py script.
