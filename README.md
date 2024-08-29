# STM32-DuraQuad
$ gdb-multiarch /home/aaron/CubeMXProjects/MyFirstCubeMXProject/build/MyFirstCubeMXProject.elf

(gdb) target extended-remote :3333   // consider switching to target extended-remote :3333
(gdb) monitor reset halt
(gdb) load
(gdb) monitor reset init
(gdb) continue // if necessary

Open up another chipd for openocd 
$ openocd -f /usr/share/openocd/scripts/interface/stlink-dap.cfg -f /usr/share/openocd/scripts/target/stm32f4x.cfg

Open up another terminal to connect to serial
$ sudo minicom -D /dev/ttyACM0 -b 115200




####### GDB
