# HALMET-stangsdal Firmware

This repository provides adapted firmware for [HALMET: Hat Labs Marine Engine & Tank interface](https://shop.hatlabs.fi/products/halmet), extended with support for the DS1603L temperature sensor to measure tank levels for gasoline, water, and other liquids.

To get started with the firmware, follow the generic SensESP [Getting Started](https://signalk.org/SensESP/pages/getting_started/) instructions but use this repository instead of the SensESP Project Template.

By default, the firmware is configured to read the engine RPM from input D1, the fuel level from input A1, and tank level from the DS1603L sensor. D2 is configured as a low oil pressure alarm input.

To customize the software for your own purposes, edit the `src/main.cpp` file.
Parts intended to be customized are marked with `EDIT:` comments.
