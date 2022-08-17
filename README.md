# Robotic Arm SDK Peripherals
The peripherals firmware of RASDK.

[`main.c`](./src/main.c)

## Serial Port Config
- Baudrate: `9600`
- Data bits: 8
- Parity: None
- Stop bits: 1
- Flow Control: None

## Env
- IDE: [PlatformIO IDE for VSCode](https://docs.platformio.org/en/latest/integration/ide/vscode.html)
- Board: [Nucleo-F401RE](https://www.st.com/en/evaluation-tools/nucleo-f401re.html) (STM32F401RE)
  - [Board Datasheet](https://www.st.com/resource/en/user_manual/um1724-stm32-nucleo64-boards-mb1136-stmicroelectronics.pdf)
  - [MCU Datasheet](https://www.st.com/resource/en/datasheet/stm32f401re.pdf)
- Framework: [LibOpenCM3](https://github.com/libopencm3)
