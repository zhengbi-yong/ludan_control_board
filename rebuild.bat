cmake --build build/Debug --target clean
cmake --build build/Debug
arm-none-eabi-objcopy -O binary D:\WR\ludan_control_board\build\Debug\ludan_control_board.elf D:\WR\ludan_control_board\build\Debug\ludan_control_board.bin