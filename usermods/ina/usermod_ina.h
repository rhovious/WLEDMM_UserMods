#ifndef WLED_ENABLE_MQTT
#warning "This user mod expects MQTT to be enabled."
#endif

#pragma once

// #include "SHT85.h"
#include <Adafruit_INA260.h>

// #define USERMOD_SHT_TYPE_SHT30 0
//  #define USERMOD_SHT_TYPE_SHT31 1
//  #define USERMOD_SHT_TYPE_SHT35 2
//  #define USERMOD_SHT_TYPE_SHT85 3

class InaUsermod : public Usermod
{
private:
  // bool enabled = false; // Is usermod enabled or not //WLEDMM use public attribute of class UserMod
  bool firstRunDone = false;        // Remembers if the first config load run had been done
  bool pinAllocDone = true;         // Remembers if we have allocated pins
  bool initDone = false;            // Remembers if the mod has been completely initialised
  bool haMqttDiscovery = false;     // Is MQTT discovery enabled or not
  bool haMqttDiscoveryDone = false; // Remembers if we already published the HA discovery topics

  // SHT vars
  // SHT *shtTempHumidSensor = nullptr;    // Instance of SHT lib
  Adafruit_INA260 ina260 = Adafruit_INA260();
  // byte inaType = 0;                     // SHT sensor type to be used. Default: SHT30
  // byte unitOfTemp = 0;                  // Temperature unit to be used. Default: Celsius (0 = Celsius, 1 = Fahrenheit)
  bool inaInitDone = false;        // Remembers if SHT sensor has been initialised
  bool inaReadDataSuccess = false; // Did we have a successful data read and is a valid temperature and humidity available?
  // const byte shtI2cAddress = 0x44;      // i2c address of the sensor. 0x44 is the default for all SHT sensors. Change this, if needed
  unsigned long shtLastTimeUpdated = 0; // Remembers when we read data the last time
  bool inaDataRequested = false;        // Reading data is done async. This remembers if we asked the sensor to read data

  float inaCurrentVoltage = 0.0f;
  float inaCurrentCurrent = 0.0f;
  float inaCurrentPower = 0.0f;
  void initINAVoltageSensor();
  void cleanupINASensor();
  void cleanup();
  inline bool isINAReady() { return inaInitDone; } // Checks if the SHT sensor has been initialised.

  void publishVoltageAndCurrentViaMqtt();
  void publishHomeAssistantAutodiscovery();
  void appendDeviceToMqttDiscoveryMessage(JsonDocument &root);

public:
  InaUsermod(const char *name, bool enabled) : Usermod(name, enabled) {} // WLEDMM
  // Strings to reduce flash memory usage (used more than twice)
  // static const char _name[]; //WLEDMM use public attribute of class UserMod
  // static const char _enabled[]; //WLEDMM not needed
  // // static const char _inaType[];
  static const char _unitOfTemp[];
  static const char _haMqttDiscovery[];

  void setup();
  void loop();
  void onMqttConnect(bool sessionPresent);
  void appendConfigData();
  void addToConfig(JsonObject &root);
  bool readFromConfig(JsonObject &root);
  void addToJsonInfo(JsonObject &root);

  bool isEnabled() { return enabled; }

  // float getTemperature();
  // float getTemperatureC() { return roundf(shtCurrentTempC * 10.0f) / 10.0f; }
  // float getTemperatureF() { return (getTemperatureC() * 1.8f) + 32.0f; }
  // float getHumidity() { return roundf(shtCurrentHumidity * 10.0f) / 10.0f; }

  float getVoltage() { return roundf(inaCurrentVoltage * 10.0f) / 10000.0f; }
  float getCurrent() { return (-1 * roundf(inaCurrentCurrent * 10.0f) / 10000.0f); }
  float getPower() { return roundf(inaCurrentPower * 10.0f) / 10000.0f; }

  const char *getUnitString();

  uint16_t getId() { return USERMOD_ID_INA; }
};

// Strings to reduce flash memory usage (used more than twice)
// const char InaUsermod::_name[]            PROGMEM = "SHT-Sensor"; //WLEDMM use public attribute of class UserMod
// const char InaUsermod::_enabled[]         PROGMEM = "Enabled"; //WLEDMM not needed
// // const char InaUsermod::_inaType[] PROGMEM = "SHT-Type";
// // const char InaUsermod::_unitOfTemp[] PROGMEM = "Unit";
const char InaUsermod::_haMqttDiscovery[] PROGMEM = "Add-To-HA-MQTT-Discovery";

/**
 * Initialise SHT sensor.
 *
 * Using the correct constructor according to config and initialises it using the
 * global i2c pins.
 *
 * @return void
 */
void InaUsermod::initINAVoltageSensor()
{
  if (!ina260.begin())
  {
    Serial.println("Couldn't find INA260 chip");
    while (1)
      ;
  }

  inaInitDone = true;
}

/**
 * Cleanup the SHT sensor.
 *
 * Properly calls "reset" for the sensor then releases it from memory.
 *
 * @return void
 */
void InaUsermod::cleanupINASensor()
{
  if (isINAReady())
  {
    //  ina260.reset();
    // delete ina260;
    // Adafruit_INA260 ina260 = Adafruit_INA260();
  }
  inaInitDone = false;
}

/**
 * Cleanup the mod completely.
 *
 * Calls ::cleanupINASensor() to cleanup the SHT sensor and
 * deallocates pins.
 *
 * @return void
 */
void InaUsermod::cleanup()
{
  cleanupINASensor();

  if (pinAllocDone)
  {
#if 0 // WLEDMM not needed
    PinManagerPinType pins[2] = { { i2c_sda, true }, { i2c_scl, true } };
    pinManager.deallocateMultiplePins(pins, 2, PinOwner::HW_I2C);
#endif
    pinAllocDone = false;
  }

  enabled = false;
  inaInitDone = false; // WLEDMM bugfix
}

/**
 * Publish temperature and humidity to WLED device topic.
 *
 * Will add a "/temperature" and "/humidity" topic to the WLED device topic.
 * Temperature will be written in configured unit.
 *
 * @return void
 */
void InaUsermod::publishVoltageAndCurrentViaMqtt()
{
#ifdef WLED_ENABLE_MQTT
  if (!WLED_MQTT_CONNECTED)
    return;
  char buf[128];

  snprintf_P(buf, 127, PSTR("%s/voltage"), mqttDeviceTopic);
  mqtt->publish(buf, 0, false, String(getVoltage()).c_str());
  snprintf_P(buf, 127, PSTR("%s/current"), mqttDeviceTopic);
  mqtt->publish(buf, 0, false, String(getCurrent()).c_str());
  snprintf_P(buf, 127, PSTR("%s/power"), mqttDeviceTopic);
  mqtt->publish(buf, 0, false, String(getPower()).c_str());
#endif
}

/**
 * If enabled, publishes HA MQTT device discovery topics.
 *
 * Will make Home Assistant add temperature and humidity as entities automatically.
 *
 * Note: Whenever usermods are part of the WLED integration in HA, this can be dropped.
 *
 * @return void
 */
void InaUsermod::publishHomeAssistantAutodiscovery()
{
#ifdef WLED_ENABLE_MQTT
  if (!WLED_MQTT_CONNECTED)
    return;

  char json_str[1024], buf[128];
  size_t payload_size;
  StaticJsonDocument<1024> json;
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  snprintf_P(buf, 127, PSTR("%s Voltage"), serverDescription);
  json[F("name")] = buf;
  snprintf_P(buf, 127, PSTR("%s/voltage"), mqttDeviceTopic);
  json[F("stat_t")] = buf;
  json[F("dev_cla")] = F("voltage");
  json[F("stat_cla")] = F("measurement");
  snprintf_P(buf, 127, PSTR("%s-voltage"), escapedMac.c_str());
  json[F("uniq_id")] = buf;
  json[F("unit_of_meas")] = F("V");
  appendDeviceToMqttDiscoveryMessage(json);
  payload_size = serializeJson(json, json_str);
  snprintf_P(buf, 127, PSTR("homeassistant/sensor/%s/%s-voltage/config"), escapedMac.c_str(), escapedMac.c_str());
  mqtt->publish(buf, 0, true, json_str, payload_size);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  json.clear();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  snprintf_P(buf, 127, PSTR("%s Current"), serverDescription);
  json[F("name")] = buf;
  snprintf_P(buf, 127, PSTR("%s/Current"), mqttDeviceTopic);
  json[F("stat_t")] = buf;
  json[F("dev_cla")] = F("Current");
  json[F("stat_cla")] = F("measurement");
  snprintf_P(buf, 127, PSTR("%s-Current"), escapedMac.c_str());
  json[F("uniq_id")] = buf;
  // json[F("unit_of_meas")] = F("A");
  appendDeviceToMqttDiscoveryMessage(json);
  payload_size = serializeJson(json, json_str);
  snprintf_P(buf, 127, PSTR("homeassistant/sensor/%s/%s-Current/config"), escapedMac.c_str(), escapedMac.c_str());
  mqtt->publish(buf, 0, true, json_str, payload_size);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

  json.clear();
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  snprintf_P(buf, 127, PSTR("%s Power"), serverDescription);
  json[F("name")] = buf;
  snprintf_P(buf, 127, PSTR("%s/Power"), mqttDeviceTopic);
  json[F("stat_t")] = buf;
  json[F("dev_cla")] = F("Power");
  json[F("stat_cla")] = F("measurement");
  snprintf_P(buf, 127, PSTR("%s-Power"), escapedMac.c_str());
  json[F("uniq_id")] = buf;
  json[F("unit_of_meas")] = F("W");
  appendDeviceToMqttDiscoveryMessage(json);
  payload_size = serializeJson(json, json_str);
  snprintf_P(buf, 127, PSTR("homeassistant/sensor/%s/%s-Power/config"), escapedMac.c_str(), escapedMac.c_str());
  mqtt->publish(buf, 0, true, json_str, payload_size);
//~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
  haMqttDiscoveryDone = true;
#endif
}

/**
 * Helper to add device information to MQTT discovery topic.
 *
 * @return void
 */
#ifdef WLED_ENABLE_MQTT
void InaUsermod::appendDeviceToMqttDiscoveryMessage(JsonDocument &root)
{
  JsonObject device = root.createNestedObject(F("dev"));
  device[F("ids")] = escapedMac.c_str();
  device[F("name")] = serverDescription;
  device[F("sw")] = versionString;
  device[F("mdl")] = ESP.getChipModel();
  device[F("mf")] = F("espressif");
}
#endif

/**
 * Setup the mod.
 *
 * Allocates i2c pins as PinOwner::HW_I2C, so they can be allocated multiple times.
 * And calls ::initINAVoltageSensor() to initialise the sensor.
 *
 * @see Usermod::setup()
 * @see UsermodManager::setup()
 *
 * @return void
 */
void InaUsermod::setup()
{
  if (enabled)
  {
    PinManagerPinType pins[2] = {{i2c_sda, true}, {i2c_scl, true}};
    // GPIOs can be set to -1 and allocateMultiplePins() will return true, so check they're gt zero
#if 0 // WLEDMM done by pinManager.joinWire()
    if (i2c_sda < 0 || i2c_scl < 0 || !pinManager.allocateMultiplePins(pins, 2, PinOwner::HW_I2C)) {
#else
    if (i2c_sda < 0 || i2c_scl < 0)
    {
#endif
      DEBUG_PRINTF("[%s] SHT pin allocation failed!\n", _name);
      cleanup();
      return;
    }
    // WLEDMM join hardware I2C
    if (pinManager.joinWire())
    { // WLEDMM - this allocates global I2C pins, then starts Wire - if not started previously
      pinAllocDone = true;

      initINAVoltageSensor();

      initDone = true;
    }
    else
    {
      DEBUG_PRINTF("[%s] SHT I2C pin allocation failed!\n", _name);
      return;
    }
  }

  firstRunDone = true;

  if (enabled && initDone && pinAllocDone && isINAReady())
  {
    USER_PRINTF(PSTR("[%s] INA sensor ready.\n"), _name);
  }
  else
  {
    USER_PRINTF(PSTR("[%s] INA sensor not ready.\n"), _name);
  }
}

/**
 * Actually reading data (async) from the sensor every 30 seconds.
 *
 * If last reading is at least 30 seconds, it will trigger a reading using
 * SHT::requestData(). We will then continiously check SHT::dataReady() if
 * data is ready to be read. If so, it's read, stored locally and published
 * via MQTT.
 *
 * @see Usermod::loop()
 * @see UsermodManager::loop()
 *
 * @return void
 */
void InaUsermod::loop()
{
  unsigned long last_runtime = 0; // WLEDMM ensure that strip.isUpdating() will not block longer that 1000ms
  if (!enabled || !initDone || !pinAllocDone || (strip.isUpdating() && (millis() - last_runtime < 1000)))
    return; // WLEDMM be nice, but not too nice
  last_runtime = millis();

  if (isINAReady())
  {
    if (millis() - inaLastTimeUpdated > 30000 && !inaDataRequested)
    {
      // ina260->requestData();
      inaDataRequested = true;

      inaLastTimeUpdated = millis();
    }

    if (inaDataRequested)
    {
      // if (ina260->dataReady())
      if (1 == 1)
      {
        // if (shtTempHumidSensor->readData(false))
        if (1 == 1)
        {
          // shtCurrentTempC = shtTempHumidSensor->getTemperature();
          // shtCurrentHumidity = shtTempHumidSensor->getHumidity();

          inaCurrentVoltage = ina260.readBusVoltage();
          inaCurrentCurrent = ina260.readCurrent();
          inaCurrentPower = ina260.readPower();

                publishVoltageAndCurrentViaMqtt();
                inaReadDataSuccess = true;
              }
              else
              {
                inaReadDataSuccess = false;
              }

              inaDataRequested = false;
            }
            */
    }
  }
}

/**
 * Whenever MQTT is connected, publish HA autodiscovery topics.
 *
 * Is only donce once.
 *
 * @see Usermod::onMqttConnect()
 * @see UsermodManager::onMqttConnect()
 *
 * @return void
 */
void InaUsermod::onMqttConnect(bool sessionPresent)
{
#ifdef WLED_ENABLE_MQTT
  if (haMqttDiscovery && !haMqttDiscoveryDone)
    publishHomeAssistantAutodiscovery();
#endif
}

/**
 * Add dropdown for sensor type and unit to UM config page.
 *
 * @see Usermod::appendConfigData()
 * @see UsermodManager::appendConfigData()
 *
 * @return void
 */
void InaUsermod::appendConfigData()
{
  oappend(SET_F("dd=addDropdown('"));
  oappend(_name);
  oappend(SET_F("','"));
  // oappend(_inaType);
  // oappend(SET_F("');"));
  // oappend(SET_F("addOption(dd,'SHT30',0);"));
  //  oappend(SET_F("addOption(dd,'SHT31',1);"));
  //  oappend(SET_F("addOption(dd,'SHT35',2);"));
  //  oappend(SET_F("addOption(dd,'SHT85',3);"));
  // oappend(SET_F("dd=addDropdown('"));
  // oappend(_name);
  // oappend(SET_F("','"));
  // oappend(_unitOfTemp);
  // oappend(SET_F("');"));
  // oappend(SET_F("addOption(dd,'Celsius',0);"));
  // oappend(SET_F("addOption(dd,'Fahrenheit',1);"));
}

/**
 * Add config data to be stored in cfg.json.
 *
 * @see Usermod::addToConfig()
 * @see UsermodManager::addToConfig()
 *
 * @return void
 */
void InaUsermod::addToConfig(JsonObject &root)
{
  JsonObject top = root.createNestedObject(FPSTR(_name)); // usermodname

  top[F("enabled")] = enabled;
  // top[FPSTR(_inaType)] = inaType;
  // top[FPSTR(_unitOfTemp)] = unitOfTemp;
  top[FPSTR(_haMqttDiscovery)] = haMqttDiscovery;
}

/**
 * Apply config on boot or save of UM config page.
 *
 * This is called whenever WLED boots and loads cfg.json, or when the UM config
 * page is saved. Will properly re-instantiate the SHT class upon type change and
 * publish HA discovery after enabling.
 *
 * @see Usermod::readFromConfig()
 * @see UsermodManager::readFromConfig()
 *
 * @return bool
 */
bool InaUsermod::readFromConfig(JsonObject &root)
{
  JsonObject top = root[FPSTR(_name)];
  if (top.isNull())
  {
    DEBUG_PRINTF("[%s] No config found. (Using defaults.)\n", _name);
    return false;
  }

  bool oldEnabled = enabled;
  // byte oldinaType = inaType;
  // byte oldUnitOfTemp = unitOfTemp;
  bool oldHaMqttDiscovery = haMqttDiscovery;

  getJsonValue(top[F("enabled")], enabled);
  // getJsonValue(top[FPSTR(_inaType)], inaType);
  // getJsonValue(top[FPSTR(_unitOfTemp)], unitOfTemp);
  getJsonValue(top[FPSTR(_haMqttDiscovery)], haMqttDiscovery);

  // First run: reading from cfg.json, nothing to do here, will be all done in setup()
  if (!firstRunDone)
  {
    DEBUG_PRINTF("[%s] First run, nothing to do\n", _name);
  }
  // Check if mod has been en-/disabled
  else if (enabled != oldEnabled)
  {
    enabled ? setup() : cleanup();
    DEBUG_PRINTF("[%s] Usermod has been en-/disabled\n", _name);
  }
  // Config has been changed, so adopt to changes
  else if (enabled)
  {
    /*
    if (oldinaType != inaType)
    {
      cleanupINASensor();
      initINAVoltageSensor();
    }

    if (oldUnitOfTemp != unitOfTemp)
    {
      publishVoltageAndCurrentViaMqtt();
      publishHomeAssistantAutodiscovery();
    }*/

    if (oldHaMqttDiscovery != haMqttDiscovery && haMqttDiscovery)
    {
      publishHomeAssistantAutodiscovery();
    }

    DEBUG_PRINTF("[%s] Config (re)loaded\n", _name);
  }

  return true;
}

/**
 * Adds the temperature and humidity actually to the info section and /json info.
 *
 * This is called every time the info section is opened ot /json is called.
 *
 * @see Usermod::addToJsonInfo()
 * @see UsermodManager::addToJsonInfo()
 *
 * @return void
 */
void InaUsermod::addToJsonInfo(JsonObject &root)
{
  if (!enabled && !isINAReady())
  {
    return;
  }

  JsonObject user = root["u"];
  if (user.isNull())
    user = root.createNestedObject("u");

  JsonArray jsonVoltage = user.createNestedArray(F("Voltage"));
  JsonArray jsonCurrent = user.createNestedArray(F("Current"));
  JsonArray jsonPower = user.createNestedArray(F("Power"));
  if (shtLastTimeUpdated == 0 || !inaReadDataSuccess)
  {
    jsonVoltage.add(0);
    jsonCurrent.add(0);
    jsonPower.add(0);

    if (shtLastTimeUpdated == 0)
    {
      jsonVoltage.add(F(" Not read yet"));
      jsonCurrent.add(F(" Not read yet"));
      jsonPower.add(F(" Not read yet"));
    }
    else
    {
      jsonVoltage.add(F(" Error"));
      jsonCurrent.add(F(" Error"));
      jsonPower.add(F(" Error"));
    }
    return;
  }

  jsonVoltage.add(getVoltage());
  jsonVoltage.add(F(" V"));

  jsonCurrent.add(getCurrent());
  jsonCurrent.add(F(" A"));
  // jsonCurrent.add(getUnitString());

  jsonPower.add(getPower());
  jsonPower.add(F(" W"));
  // jsonPower.add(getUnitString());

  // sensor object
  JsonObject sensor = root[F("sensor")];
  if (sensor.isNull())
    sensor = root.createNestedObject(F("sensor"));

  jsonVoltage = sensor.createNestedArray(F("Voltage"));
  jsonVoltage.add(getVoltage());
  jsonVoltage.add(F(" V"));
  // jsonVoltage.add(getUnitString());

  jsonCurrent = sensor.createNestedArray(F("Current"));
  jsonCurrent.add(getCurrent());
  jsonCurrent.add(F(" A"));

  jsonPower = sensor.createNestedArray(F("Power"));
  jsonPower.add(getPower());
  jsonPower.add(F(" W"));
}

/**
 * Getter for last read   for configured unit.
 *
 * @return float


float InaUsermod::getTemperature()
{
  return unitOfTemp ? getTemperatureF() : getTemperatureC();
}
 */
/**
 * Returns the current configured unit as human readable string.
 *
 * @return const char*
 */
// const char *InaUsermod::getUnitString()
//{
//   return unitOfTemp ? "°F" : "°C";
// }