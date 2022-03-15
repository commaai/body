Compile:
make clean && make

Flash:
openocd -f interface/stlink.cfg -c "set CPUTAPID 0" -f target/stm32f4x.cfg -c init -c "reset halt" -c "flash write_image erase obj/body.hex 0 ihex" -c "reset run" -c "shutdown"
