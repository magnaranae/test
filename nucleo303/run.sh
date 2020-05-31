# semihosting working
xfce4-terminal --command 'sudo openocd -f openocd.cfg'  --hold & gdb-multiarch  build/nucleo303.elf -q -x openocd.gdb
