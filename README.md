LPCXpresso1549
==============

This BSP is modified based on the NXP official [LPCOpen Software Development Platform  LPC15XX](https://www.nxp.com/support/developer-resources/software-development-tools/lpc-developer-resources-/lpcopen-libraries-and-examples/lpcopen-software-development-platform-lpc15xx:LPCOPEN-SOFTWARE-FOR-LPC15XX).

Please reference the [LPCXpresso for LPC1549](https://www.nxp.com/support/developer-resources/evaluation-and-development-boards/lpcxpresso-boards/lpcxpresso-board-for-lpc1549:OM13056) for more information about this board.

In this demo, an **Arduino Accessory Shield** is used to rich the applications. The following components are included in this shield

  - 0.96inch OLED (SSD1306 drived)
  - Triaxial Accelerometer (ADXL345)
  - High precision RTC (DS3231)
  - Temperature Sensor (LM75B)
  - RGB LED (P9813 drived)
  - Adjustable Potentiometer
  - Rocker
  - Buzzer

For more information about the shield, please refer to the [WaveShare Accessory Shield Wiki](http://www.waveshare.net/wiki/Accessory_Shield)

Getting Started
---------------

* Preperation

  - Install [MDK-Arm](https://www.keil.com/download/product/) and the [packs](https://www.keil.com/dd2/Pack/) for **NXP LPC1500** Series Device.
  - Download the necessary [SOFTWARE & TOOLS](https://www.nxp.com/support/developer-resources/evaluation-and-development-boards/lpcxpresso-boards/lpcxpresso-board-for-lpc1549:OM13056?tab=Design_Tools_Tab) in the NXP website for **BASIC Usage Guide** of the board.
  - Follow the instructions in the [User Guide](https://www.nxp.com/docs/en/supporting-information/LPCXPRESSO-V2-BOARD-FAMILY-SI.pdf) to get the board boot up.

* Build and run the image

  - Open the MDK project in the folder `applications\lpc15xx\keil\nxp_lpcxpresso_1549\freertos`, build the image.
  - Make sure the **CMSIS-DAP Debugger** is selected in MDK debugger configuration `Options for Target -> Debug`.
  - Download the code to flash memery and press the reset button.

* Demo instruction

  - The demo creates a simple task to flash the **Blue LED** on the board.
  - Also a **UART driver** and the **FreeRTOS-PLUS-CLI** feature is added.
  - An **I2C Bus** and **LM75B** drivers are demonstrated in the Arduino task. And the temperature is also displayed on the **OLED Screen**.

Reference
---------

- The LPCXpresso1549 have the **ROM resident driver** feature and the **ROM API** is provided. So the I2C driver in this demo take advantage of the **I2C ROM API**.
- Some driver code is modified based on the drivers in uC/OS, but the comments may still not have been corrected.