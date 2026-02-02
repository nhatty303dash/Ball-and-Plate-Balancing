# Capstone – Ball & Plate Balancing (STM32F407 + PC Vision GUI)

STM32 firmware + a PC camera application for a ball-and-plate balancing robot.

The PC tracks the ball position and sends \((x, y)\) to the STM32 over UART. The STM32 continuously updates the plate angle by driving 3 servos via a PCA9685 (I2C).

## How it works (data flow)

1. **Camera + PC tracking**
   - The camera image is processed on the PC to estimate the ball position relative to the plate center.
   - The position is converted to **centimeters** and sent as compact UART frames.

2. **UART link (PC → STM32)**
   - The STM32 listens on **USART1**.
   - Reception uses **DMA (circular buffer)** and an **IDLE-line interrupt** to detect new data quickly.
   - Incoming bytes are parsed into frames; valid frames update the “latest ball position” used by the control loop.

3. **Control loop (STM32)**
   - The firmware runs a fast main loop:
     - reads the most recent ball \((x, y)\)
     - computes a plate command (tilt/rotation)
     - updates servo targets (non-blocking motion update)
   - If no valid ball data arrives for a while, the firmware can reset control and return the plate toward center for safety/stability.

4. **Servo output (STM32 → PCA9685 → servos)**
   - The STM32 drives a **PCA9685** over **I2C1** at **50 Hz**.
   - PCA9685 channels **0–2** are used for the 3 servos.

## Where things are

- `Core/` – STM32CubeIDE firmware (UART receive, control loop, servo output)
- `ball_detection/` – PC vision/GUI + serial transmitter
- `Capstone.ioc` – CubeMX configuration

## Connections (from `Capstone.ioc`)

- **USART1** (PC/GUI link): PA9 TX, PA10 RX, **115200 8N1**
- **I2C1** (PCA9685): PB7 SDA, PB8 SCL

## UART data (minimal)

- Frames use **start `'@'` (0x40)** and **stop `'X'` (0x58)** plus a **uint16 checksum** (sum of bytes from start→stop, little-endian after stop).
- Main data sent from PC is the ball position:

```
@ <x:int8> <y:int8> X <checksum:uint16_le>
```

`x`/`y` are in **cm**, scaled by **10** (e.g. 1.2 cm → 12).  
Other optional control frames (target/trajectory) exist—see `Core/Inc/uart_handler.h`.

## Notes

If the STM32 does not react, confirm you are using **USART1 RX (PA10)** and the correct Windows COM port.

