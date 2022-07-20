## uECG device main firmware
This firmware version supports BLE connection to uECG app with full functionality (ECG data with RR, HRV, GSR and accel/gyro, steps data) - and also can work as a generic pulse tracking device (in generic mode, only BPM and RR intervals are available - but it can work with popular fitness apps).

For building the firmware, you also need to download our micro-sdk: https://github.com/ultimaterobotics/urf_lib, by default it's expected to be in the same level folder (so that you have folder urf_lib next to folder uECG_v5). Build relies on arm-none-eabi-gcc compiler, path to compiler is configured in this file: urf_lib/nrf_usdk52/gcc/Makefile.posix (or Makefile.windows, depending on your OS).

This firmware requires our bootloader (https://github.com/ultimaterobotics/uECG_bootloader) and doesn't need Nordic's softdevice.

### Links to other parts of the project:
* Bootloader: https://github.com/ultimaterobotics/uECG_bootloader
* USB base receiver: https://github.com/ultimaterobotics/udevices_base
* PCB design: https://github.com/ultimaterobotics/uECG_pcb
* NodeJS app for receiving data on PC: https://github.com/ultimaterobotics/uECG_nodejs
* Android app: https://github.com/ultimaterobotics/uECG_android 
* previous version of this repository now is located here: https://github.com/ultimaterobotics/uECG_v4_3_all
