# SHT45
Usermod to support SHT45 sensor.

## Requirements
*   adafruit SHT4x Library adafruit/Adafruit SHT4x Library@^1.0.4
*  https://github.com/adafruit/Adafruit_Sensor

## Usermod installation
Simply copy the below block (build task) to your `platformio_override.ini` and compile WLED using this new build task. Or use an existing one, add the buildflag `-D USERMOD_SHT45` and the below library dependencies.

ESP32:
```
[env:custom_esp32dev_usermod_sht45]
extends = env:esp32dev
build_flags = ${common.build_flags_esp32}
  -D USERMOD_SHT45
lib_deps = ${esp32.lib_deps}
   adafruit/Adafruit SHT4x Library@^1.0.4
   https://github.com/adafruit/Adafruit_Sensor
```

ESP8266:
```
[env:custom_d1_mini_usermod_sht45]
extends = env:d1_mini
build_flags = ${common.build_flags_esp8266}
  -D USERMOD_SHT45
lib_deps = ${esp8266.lib_deps}
   adafruit/Adafruit SHT4x Library@^1.0.4
   https://github.com/adafruit/Adafruit_Sensor
```

## MQTT Discovery for Home Assistant
If you're using Home Assistant and want to have the temperature and humidity available as entities in HA, you can tick the "Add-To-Home-Assistant-MQTT-Discovery" option in the usermod settings. If you have an MQTT broker configured under "Sync Settings" and it is connected, the mod will publish the auto discovery message to your broker and HA will instantly find it and create an entity each for the temperature and humidity.

### Publishing readings via MQTT
Regardless of having MQTT discovery ticked or not, the mod will always report temperature and humidity to the WLED MQTT topic of that instance, if you have a broker configured and it's connected.

## Configuration
Navigate to the "Config" and then to the "Usermods" section. If you compiled WLED with `-D USERMOD_SHT45`, you will see the config for it there:
* Unit:
  * What it does: Select which unit should be used to display the temperature in the info section. Also used when sending via MQTT discovery, see below.
  * Possible values: Celsius, Fahrenheit
  * Default: Celsius
* Add-To-HA-MQTT-Discovery:
  * What it does: Makes the temperature and humidity available via MQTT discovery, so they're automatically added to Home Assistant, because that way it's typesafe.
  * Possible values: Enabled/Disabled
  * Default: Disabled

## Change log
2023-10
* First implementation.

## Credits
Based off of SHT usermod, ezcGman | Andy
