A home trainer aka ergometer that doesn't unterstand what the bike app wants is a hassle.

This repo is:
- a BLE proxy on an ESP32 that is both a BLE client and a BLE server simultaneously, and
- a translation device converting requests from bike apps so the trainer can be controlled (varying resistance) and the app gets back meaningful data from the bike

Tools/Libraries used:
- Arduino IDE
- NimBLE arduino lib

code status:
- proof of concept

shortcomings:
- lots of magic numbers still in code
- BLE name of trainer used for development currently hardcoded. ToDo: BLE-advertise multiple home trainers/ergometers with e.g. `-proxy` appended to the name, so one can choose directly from the bike app
