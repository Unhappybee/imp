# Microcontroller power management

Developed a system with multiple power modes: Active, Modem Sleep, Light Sleep, and Deep Sleep, controlling power consumption and system behavior across different states.
Utilized a rotary encoder connected to GPIO25, GPIO26, and GPIO27 for detecting rotation and button presses, which interact with the system’s modes.
Implemented Bluetooth communication using the BluetoothSerial.h library to accept commands for controlling LED, sleep modes, and more:
Commands include turning the LED on/off, switching between various sleep modes, and waking up from low-power states.
Configured touch buttons on GPIO12 and GPIO14 for triggering modem-sleep, light-sleep, and deep-sleep modes, responding to user input via the touchRead() function.
LED control connected to GPIO13, responding to Bluetooth commands to toggle on/off states.
Designed a wake-up mechanism with a button on GPIO2 to wake up the system from light-sleep and deep-sleep modes.
# Managed four power modes:
## Active Mode: Full processor and peripheral functionality, supporting rotary encoder detection, Bluetooth communication, and LED control.
## Modem Sleep Mode: Processor and peripherals remain active, with reduced power consumption while allowing Bluetooth communication.
## Light Sleep Mode: CPU suspended, with no Bluetooth communication or rotary encoder input, but state is maintained for wake-up.
## Deep Sleep Mode: RTC memory retains data, and only essential values persist, allowing for ultra-low power consumption.
