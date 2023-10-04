# INA
Usermod to support the INA260 sensor from ADAfruit

## Requirements
* "ADAFruint INA260: 	adafruit/Adafruit INA260 Library@^1.5.0

## Usermod installation
Simply copy the below block (build task) to your `platformio_override.ini` and compile WLED using this new build task. Or use an existing one, add the buildflag `-D USERMOD_INA` and the below library dependencies.

ESP32:
```
[env:custom_esp32dev_usermod_ina]
extends = env:esp32dev
build_flags = ${common.build_flags_esp32}
  -D USERMOD_INA
lib_deps = ${esp32.lib_deps}
	adafruit/Adafruit INA260 Library@^1.5.0
```

ESP8266:
```
[env:custom_d1_mini_usermod_ina]
extends = env:d1_mini
build_flags = ${common.build_flags_esp8266}
  -D USERMOD_INA
lib_deps = ${esp8266.lib_deps}
	adafruit/Adafruit INA260 Library@^1.5.0
```

## MQTT Discovery for Home Assistant
If you're using Home Assistant and want to have the temperature and humidity available as entities in HA, you can tick the "Add-To-Home-Assistant-MQTT-Discovery" option in the usermod settings. If you have an MQTT broker configured under "Sync Settings" and it is connected, the mod will publish the auto discovery message to your broker and HA will instantly find it and create an entity each for the temperature and humidity.

### Publishing readings via MQTT
Regardless of having MQTT discovery ticked or not, the mod will always report temperature and humidity to the WLED MQTT topic of that instance, if you have a broker configured and it's connected.

## Configuration
Navigate to the "Config" and then to the "Usermods" section. If you compiled WLED with `-D USERMOD_INA`, you will see the config for it there:
* Add-To-HA-MQTT-Discovery:
  * What it does: Makes the temperature and humidity available via MQTT discovery, so they're automatically added to Home Assistant, because that way it's typesafe.
  * Possible values: Enabled/Disabled
  * Default: Disabled

## Change log
2023-10
* First implementation.

## Credits
Based off of SHT usermod, ezcGman | Andy
