# README - BMS_Screen

## Component

- LCD Screen.
- Dual color led(while and yellow).
- Spring-loader(touch).

## Short description

This is the firmware for the LCD screen controller. The interface is simple: tap and hold the button to check battery status, turn the LED on/off, and control lighting effects.

## Upgrade

- **Touch:** Display capacitance battery (Done).
- **Press and hold 3s:** turn on the led
- **Press and hold 1s:** Change the led color

- **Default:**
  - LED turns on in **white color** at **brightness level 1** by default in the first
  - Brightness has **5 levels** (1–5)
  - Each level lights up **2 bars** and displays numbers **1–5**, representing battery percentage
  - **Voltage** and **temperature** display remain unchanged
  - **Double-tap in LED ON/EFFECT mode** turn off the led
  - **Press and hold 5s:** Turn on the LED flashing effect
  - **Press and hold for more than 10s** Enter the setting mode for 3s, 4s, or 8s operation.
  - In setting mode:
    - **Press and hold for 2s** to confirm and run the selected mode.
    - **Tap the button** to cycle through the options: 3s, 4s, or 8s.

## Example

1. **Tap** the button to toggle and view the battery percentage. ✓
2. **Press and hold for 3s** to turn on the **white LED** first, then tap to increase or decrease brightness. ✓
    2.1. **Press and hold for 1s** to switch to the **yellow LED**. ✓
    2.2. **Press and hold for 1s** again to turn on **both LEDs**. ✓
3. **Double-tap in LED ON/EFFECT mode** to turn off the LED. ✓
4. **Press and hold for 5s** to turn on the LED flashing effect. ✓
5. **Press and hold for more than 10s** to enter the **setting mode** for 3s, 4s, or 8s operation:
   - **Tap the button** to cycle through the options: 3s2v7, 4s3v2(main), or 8s3v2.
   - **Press and hold for 2s** to confirm and run the selected mode.

## Contact

  Please refer to the source code in the **Firmware** folder for detailed setup and configuration.
