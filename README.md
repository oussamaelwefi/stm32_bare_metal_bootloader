# STM32F407G-DISC1 bare metal bootloader
<div align="center">
  <img src="./images/stm32f407g.webp" alt="board image" width="200"/>
  <img src="./images/stm32cubeide_logo.avif" alt="ide logo" width="200"/>
</div>
Welcome to my STM32 project. I have always wondered what happens under the HAL library function calls, or the pin assignements in the STMCubeIDE. The best way to learn is to manipulate the registers manually, reading the reference manual, find the registers addresses and understand what each bit in the register is for.<br>
We have built a bootloader that allows us to write code to memory, erase memory, jump to user application code, and using CRC hardware verification for integrity.<br>
We converted the ".elf" binary file into a ".bin" file to test the memory write funcionality. <br>
We used the STM32 ST-LINK Utility to view the memory and make sure when it is written of deleted.<br><br>
<div align="center">
  <img src="./images/st-link_utility.jpg" alt="st-link utility screenshot" width="800"/>
</div>
The host code executed on the computer can be found in :  https://github.com/niekiran/BootloaderProjectSTM32<br><br>
<div align="center">
  <img src="./images/Screenshot 2025-11-30 165243.jpg" alt="host screenshot" width="800"/>
</div>
