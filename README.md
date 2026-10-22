# TimeBox

Development firmware for an ESP32-S3 GNSS/PPS + OCXO time server.

## Status

**Development / experimental.** The current checked-in firmware is **1.2.11**.

The project combines:

- GNSS time and 1 PPS reference
- 10 MHz OCXO measurement
- DS3231M RTC monitoring and aging calibration
- local NTP server
- web diagnostics and standalone logging
- Allan deviation and long-term OCXO statistics

The next major development step is **1.3.0**, planned to add true OCXO holdover and switchable RTC calibration/free-run measurement.

## Current hardware mapping

ESP32-S3 Zero:

- GPS TX -> GPIO1
- GPS RX <- GPIO2
- GPS PPS -> GPIO4
- I2C SDA -> GPIO5
- I2C SCL -> GPIO6
- RTC SQW -> GPIO7
- RTC 32 kHz -> GPIO8
- OCXO 10 MHz -> GPIO9

## Important

This repository is public and the firmware is still under active development. Diagnostic values and accuracy figures are measurements of the complete test setup, not calibration certificates.

The setup access point currently uses a firmware-defined default credential. Do not treat it as a secret.

## Firmware

- `firmware/Zeitkiste_1.2.11.cpp` - current development snapshot
- `history/Zeitkiste_1.2.10_to_1.2.11.diff` - diagnostic-only delta from 1.2.10

