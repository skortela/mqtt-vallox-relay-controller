# mqtt-vallox-relay-controller
Relay controller for Vallox air ventilation.
This SW is firmware for ESP8266 based WeMos microchip.

This was developed for controlling Vallox air ventilation system. Vallox has two motors: air-in and air-out, motor speeds are controlled with relays. This program controls 8 different relays that are used within Vallox motors.

Uses MQTT protocol over WiFi, for Home Assistant integration.
