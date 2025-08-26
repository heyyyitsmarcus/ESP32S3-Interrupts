# Sensor Node

## About
This project drives the sensor node for the wildfire detection system.

This implementation uses an interrupt on the G0 pin from the LoRa module to trigger the commands to send the data to the central node instead of busy-waiting for a sent packet, thus saving power.
