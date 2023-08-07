#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <secret.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#include <FIFO.h>
#include <UDPJsonMessages.h>
#include <cJSON.h>
#include <cJSON_Utils.h>

#include <ThinkFitCommand.h>

#include <BLETools.h>

#include <Wire.h>
#include <VL53L1X.h>

#define CMD_OBTAIN_PARAMS 1
#define CMD_UPDATE 2
#define CMD_START 3
#define CMD_STOP 4
#define CMD_SET_SPEED_SLOPE 5

#define UDP_CMD_START 1
#define UDP_CMD_STOP 2
#define UDP_CMD_SET_SPEED 3
#define UDP_CMD_SET_SLOPE 4
#define UDP_CMD_SET_DISTANCE 5

#define PRESENCE_DISTANCE_BUFFER_SIZE 30
#define CMD_FIFO_LEN 5

#define DELAY_BETWEEN_READING_ON_START 500
#define DELAY_BETWEEN_READING_ON_STOP 1250

#define DEBUG_WITH_BLE

BleTools ble("TREADMILL THINKFIT");
BleSerial *bleSerial;

WiFiUDP UDP;

Stream *recvDebug;
Stream *sendDebug;

ThinkfitParams params;
ThinkfitStatusData statusData;
ThinkfitSpeedSlopeData speedSlopeData;

uint8_t thinkfitCommand;

uint8_t fifoBuffer[CMD_FIFO_LEN];
FIFO<uint8_t> thinkfitCommandsFifo(fifoBuffer, CMD_FIFO_LEN);

bool debug = false;

bool otaUpdating = false;
void ota()
{
  static bool otaConfigured = false;
  if (!otaConfigured)
  {
    otaConfigured = true;
    ArduinoOTA.setHostname("EnerfitTreadmill");
    ArduinoOTA.setRebootOnSuccess(true);

    ArduinoOTA.onStart([]()
                       {
    String type = ArduinoOTA.getCommand() == U_FLASH ? "sketch" : "filesystem"; // U_FS;
    // NOTE: if updating FS this would be the place to unmount FS using FS.end()

    Serial.println("Start updating " + type); 
    otaUpdating = true; });

    ArduinoOTA.onEnd([]()
                     { 
                     Serial.println("\nOTA End"); 
                     otaUpdating = false; });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                          { Serial.printf("Progress: %u%% \n", (progress / (total / 100))); });

    ArduinoOTA.onError([](ota_error_t error)
                       {
                        otaUpdating = false;
                        Serial.printf("Error[%u]: ", error);
                        switch(error)
                        {
                          case OTA_AUTH_ERROR:
                          Serial.println("Auth Failed");
                          break;
                          case OTA_BEGIN_ERROR:
                          Serial.println("Begin Failed");
                          break;
                          case OTA_CONNECT_ERROR:
                          Serial.println("Connect Failed");
                          break;
                          case OTA_RECEIVE_ERROR:
                          Serial.println("Receive Failed");
                          break;
                          case OTA_END_ERROR:
                          Serial.println("End Failed");
                          break;
                        } });

    ArduinoOTA.begin();
  }

  ArduinoOTA.handle();
}

/*
When the value of eFuse field EFUSE_UART_PRINT_CONTROL is
0 (default), print is enabled and not controlled by GPIO8.
1, if GPIO8 is 0, print is enabled; if GPIO8 is 1, it is disabled.
2, if GPIO8 is 0, print is disabled; if GPIO8 is 1, it is enabled.
3, print is disabled and not controlled by GPIO8.
*/
void setSerialDebugEFuse()
{
  int size = esp_efuse_get_field_size(ESP_EFUSE_UART_PRINT_CONTROL);

  uint8_t efuse = 0;
  esp_efuse_read_field_blob(ESP_EFUSE_UART_PRINT_CONTROL, &efuse, size);

  if (efuse != 3)
  {
    efuse = 3;
    esp_efuse_write_field_blob(ESP_EFUSE_UART_PRINT_CONTROL, &efuse, size);
  }
}

bool wasConnected = false;
bool wifi()
{
  static bool wifiConfigured = false;
  if (!wifiConfigured)
  {
    wifiConfigured = true;

    WiFi.setHostname("ThinkfitWIFI");
    WiFi.config(IP_ADDRESS, GATEWAY, SUBNET);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    WiFi.setAutoReconnect(true);
    WiFi.enableLongRange(true);
    WiFi.setTxPower(wifi_power_t::WIFI_POWER_19_5dBm);
  }

  static bool oldConnected = false;
  bool connected = WiFi.isConnected();

  if (debug)
  {
    static uint32_t timestampWifiDebugInfo = 0;
    if (timestampWifiDebugInfo == 0 || (millis() - timestampWifiDebugInfo) > 15000)
    {
      timestampWifiDebugInfo = millis();

      String localIPString = WiFi.localIP().toString();
      String mask = WiFi.subnetMask().toString();
      sendDebug->printf("WiFi connection status= %s, RSSI= %d dBm, IP=%s, SubnetMask=%s, Channel=%d \n",
                        connected ? "CONNECTED" : "DISCONNECTED",
                        WiFi.RSSI(),
                        localIPString.c_str(),
                        mask.c_str(),
                        WiFi.channel());
      localIPString = mask = "";
    }
  }

  if (connected != oldConnected)
  {
    oldConnected = connected;
    sendDebug->printf("WiFi connected. MAC= %x:%x:%x:%x:%x:%x\n",
                      WiFi.macAddress()[0],
                      WiFi.macAddress()[1],
                      WiFi.macAddress()[2],
                      WiFi.macAddress()[3],
                      WiFi.macAddress()[4],
                      WiFi.macAddress()[5]);
  }
  return connected;
}

size_t udpRecvSize;
uint8_t udpRecvBuffer[4096];
size_t udpSendSize;
uint8_t udpSendBuffer[4096];

void udp()
{
  static bool udpInitialized = false;
  if (!udpInitialized)
  {
    udpInitialized = true;

    UDP.begin(UDP_LISTEN_PORT);
    UDP.setTimeout(1000);
  }

  udpRecvSize = UDP.parsePacket();
  if (udpRecvSize)
  {
    UDP.read(udpRecvBuffer, udpRecvSize);

    sendDebug->printf("UDP - Packet received. Size= %d\n", udpRecvSize);
    if (debug)
    {
      for (int x = 0; x < udpRecvSize; x++)
      {
        sendDebug->write(udpRecvBuffer[x]);
      }
      sendDebug->println();
    }
  }

  if (udpSendSize)
  {
    UDP.beginPacket(UDP_SEND_HOST, UDP_SEND_PORT);
    for (int x = 0; x < udpSendSize; x++)
    {
      UDP.write(udpSendBuffer[x]);
    }
    UDP.endPacket();

    sendDebug->printf("UDP - Packet sent. Size= %d \n", udpSendSize);
    if (debug)
    {
      for (int x = 0; x < udpSendSize; x++)
      {
        sendDebug->write(udpSendBuffer[x]);
      }
      sendDebug->println();
    }

    udpSendSize = 0;
  }
}

VL53L1X distanceSensor;
uint16_t distanceSensorRead()
{
  static bool distanceSensorInitOK = false;
  static uint32_t timeoutTimestamp = 0;

  if (distanceSensorInitOK)
  {
    distanceSensor.readSingle(false);

    timeoutTimestamp = millis();
    while (!distanceSensor.dataReady())
    {
      yield();
      if (millis() - timeoutTimestamp > 30)
      {
        distanceSensorInitOK = false;

        sendDebug->println("VL53L1X - Sensor timeout.");
        return 0;
      }
      delay(1);
    }

    uint16_t distance = distanceSensor.read(false);
    return distance;
  }

  static uint32_t lastInitTimestamp = 0;
  // Only try to init senza when treadmill is not running, so i avoid not so cool stuff.
  if (statusData.status == THINKFIT_STATUS_STANDBY && (lastInitTimestamp == 0 || millis() - lastInitTimestamp > 1000))
  {
    lastInitTimestamp = millis();

    distanceSensor.setTimeout(100);
    distanceSensorInitOK = distanceSensor.init();
    if (distanceSensorInitOK)
    {
      // distanceSensor.setDistanceMode(VL53L1X::DistanceMode::Long); By default is set to long. No need to do it double.
      distanceSensor.setMeasurementTimingBudget(10000);
      // distanceSensor.startContinuous(100);

      sendDebug->println("VL53L1X - Sensor initialized.");
    }
    else
    {
      sendDebug->println("DISTANCE - Sensor initialize fail.");
    }
  }

  return 0;
}

void setup()
{
  setSerialDebugEFuse();
  udpSendSize = fillJSONUpdateRequest(udpSendBuffer); // At startup, request parameters to PC.

  Serial.begin(115200);  // USB
  Serial0.begin(115200); // UART 0

  Wire.setPins(GPIO_NUM_9, GPIO_NUM_10);
  Wire.begin();
  Wire.setClock(100000); // use 100 kHz I2C

  ble.init("PAROZZZ");
  ble.setDebugSerial(&Serial);
  bleSerial = ble.initSerial();
  ble.startAdvertising(0xB00B);

#ifdef DEBUG_WITH_BLE
  recvDebug = bleSerial->getReceive();
  sendDebug = bleSerial->getSend();
#else
  recvDebug = &Serial;
  sendDebug = &Serial;
#endif

  thinkfitInit(&Serial0);
  sendDebug->println("THINKFIT INIT - DONE");
}

bool continousUpdate = true;
uint32_t delayBetweenCommands = DELAY_BETWEEN_READING_ON_STOP;

uint16_t paramPresenceDistance = 0; // mm

void loop()
{
#pragma region WIFI UDP OTA
  if (wifi())
  {
    ota();
    if (otaUpdating)
    {
      static bool oldOtaUpdating = false;
      if (!oldOtaUpdating)
      {
        oldOtaUpdating = true;
        
        ble.stopAdvertising();
        UDP.stop();
      }

      return;
    }

    udp();
  }
#pragma endregion
#pragma region BLE DEBUG SERIAL RECEIVE
  ble.loop();

  if (!ble.connected)
  {
    debug = false;
  }

  while (recvDebug->available())
  {
    uint8_t read = recvDebug->read();
    if (!thinkfitCommandsFifo.isFull())
    {
      switch (read)
      {
      case '1':
        sendDebug->println("THINKFIT - ADD OBTAIN PARAMS");
        thinkfitCommandsFifo.insert(CMD_OBTAIN_PARAMS);
        break;
      case '2':
        sendDebug->println("THINKFIT - ADD UPDATE");
        thinkfitCommandsFifo.insert(CMD_UPDATE);
        break;
      case '3':
        sendDebug->println("THINKFIT - ADD START");
        thinkfitCommandsFifo.insert(CMD_START);
        break;
      case '4':
        sendDebug->println("THINKFIT - ADD STOP");
        thinkfitCommandsFifo.insert(CMD_STOP);
        break;
      case '5':
        sendDebug->println("THINKFIT - ADD SET SPEED SLOPE");
        thinkfitCommandsFifo.insert(CMD_SET_SPEED_SLOPE);
        break;
      case 'E':
      case 'e':
        debug = true;
        sendDebug->println("Enable debug.");
        thinkfitSetDebugStream(sendDebug);
        break;
      case 'D':
      case 'd':
        debug = false;
        sendDebug->println("Disable debug.");
        thinkfitSetDebugStream(nullptr);
        break;
      case 'S':
      case 's':
        sendDebug->println("STOP CONTINOUS UPDATE");
        continousUpdate = false;
        break;
      case 'C':
      case 'c':
        sendDebug->println("STARTED CONTINOUS UPDATE");
        continousUpdate = true;
        break;
      case 'R':
      case 'r':
        sendDebug->println("Restart");
        delay(10);

        ESP.restart();
        break;
      case 'W':
      case 'w':
        sendDebug->println("Reconnect WiFi");
        WiFi.reconnect();
        break;
      }
    }
  }
#pragma endregion
#pragma region UDP JSON PARSING
  if (udpRecvSize)
  {
    cJSON *json = cJSON_ParseWithLength((const char *)udpRecvBuffer, udpRecvSize);
    udpRecvSize = 0;

    if (cJSON_HasObjectItem(json, "Speed"))
    {
      speedSlopeData.speed = cJSON_GetNumberValue(cJSON_GetObjectItem(json, "Speed"));
      thinkfitCommandsFifo.insert(CMD_SET_SPEED_SLOPE);

      if (debug)
        sendDebug->printf("UDP - Received Speed= %d\n", speedSlopeData.speed);
    }

    if (cJSON_HasObjectItem(json, "Slope"))
    {
      speedSlopeData.slope = cJSON_GetNumberValue(cJSON_GetObjectItem(json, "Slope"));
      thinkfitCommandsFifo.insert(CMD_SET_SPEED_SLOPE);

      if (debug)
        sendDebug->printf("UDP - Received Slope= %d\n", speedSlopeData.slope);
    }

    if (cJSON_HasObjectItem(json, "Start"))
    {
      cJSON_bool start = cJSON_IsTrue(cJSON_GetObjectItem(json, "Start"));
      thinkfitCommandsFifo.insert(start ? CMD_START : CMD_STOP);

      if (debug)
        sendDebug->printf("UDP - Received Start= %d\n", start);
    }

    if (cJSON_HasObjectItem(json, "SetDistance"))
    {
      double distance = cJSON_GetNumberValue(cJSON_GetObjectItem(json, "SetDistance"));
      paramPresenceDistance = distance * 10;

      if (debug)
        sendDebug->printf("UDP - Received SetDistance= %d\n", distance);
    }

    cJSON_Delete(json);
  }
#pragma endregion

  static bool presence = false;
  static bool oldPresence = false;
  static bool presenceUpdateReq = false;
  /*
    This is not the best for a single core CPU. It needs a separated core to handle all the stuff that can happen (Like disconnections and re-init)
  #pragma region SENSOR DISTANCE
    // Do it while we don't have a command active. Since is blocking, it might interfere with the thinkfit stuff and cause problems.
    if (!thinkfitCommand)
    {

      static uint32_t distanceReadTimestamp = 0;
      if (distanceReadTimestamp == 0 || (millis() - distanceReadTimestamp) > 2000)
      {
        distanceReadTimestamp = millis();

        uint16_t distance = distanceSensorRead();
        if (distance)
        {
          if (debug)
            sendDebug->printf("DISTANCE - Read %d[mm]\n", distance);

          presence = distance < paramPresenceDistance;
          if (presence != oldPresence)
          {
            presenceUpdateReq = true;
            sendDebug->printf("DISTANCE - Presence %s\n", presence ? "ON" : "OFF");
          }
          oldPresence = presence;
        }
      }
    }
  #pragma endregion
  */
  thinkfitLoop(); // Its place is important. Don't move it!

#pragma region THINKFIT COMMANDS

  // This get the first output commando inside the commands fifo.
  // If it does not find anything (So thinkfitCommand is not populated),
  // it will set it as a update command (If enabled) so thinkfit library read the status of the treadmill.-
  static uint32_t lastCmdTimestamp = 0;
  if (!thinkfitCommand && (millis() - lastCmdTimestamp) > delayBetweenCommands)
  {
    thinkfitCommand = thinkfitCommandsFifo.get();
    if (thinkfitCommand)
    {
      lastCmdTimestamp = millis();
    }
    else if (continousUpdate)
    {
      thinkfitCommand = CMD_UPDATE;
      lastCmdTimestamp = millis();
    }
  }

  int8_t result = 0;
  switch (thinkfitCommand)
  {
  case CMD_OBTAIN_PARAMS:
    result = thinkfitCommObtainParams(&params);
    if (result)
    {
      sendDebug->println("THINKFIT - OBTAINS PARAM DONE");
    }
    break;
  case CMD_UPDATE:
    result = thinkfitCommStatus(&statusData);
    if (result > 0)
    {
      static uint8_t oldStatus = 0xFF; // Set to invalid value so the first loop after boot it will update the PC

      switch (statusData.status)
      {
      case THINKFIT_STATUS_STARTING:
      case THINKFIT_STATUS_RUNNING:
      case THINKFIT_STATUS_STOPPING:
        delayBetweenCommands = DELAY_BETWEEN_READING_ON_START;
        udpSendSize = fillJSONStatusData(udpSendBuffer, &statusData, presence);
        break;
      default:
        delayBetweenCommands = DELAY_BETWEEN_READING_ON_STOP;
        if (oldStatus != statusData.status || presenceUpdateReq)
        { // Only update after the treadmill went into the standby mode or if he changes mode for wathever reason.
          udpSendSize = fillJSONStatusData(udpSendBuffer, &statusData, presence);
        }
        break;
      }
      oldStatus = statusData.status;
    }

    if (result)
    {
      presenceUpdateReq = false;
      sendDebug->println("THINKFIT - STATUS DONE");
    }
    break;
  case CMD_START:
    result = thinkfitCommStart();
    if (result > 0)
    {
      delayBetweenCommands = DELAY_BETWEEN_READING_ON_START;
    }

    if (result)
    {
      sendDebug->println("THINKFIT - START DONE");
    }
    break;
  case CMD_STOP:
    result = thinkfitCommStop();
    if (result)
    {
      sendDebug->println("THINKFIT - STOP DONE");
    }
    break;
  case CMD_SET_SPEED_SLOPE:
    result = thinkfitCommSetSpeedSlope(&speedSlopeData);
    if (result)
    {
      sendDebug->println("THINKFIT - SET SPEED SLOPE DONE");
    }
    break;
  }

  if (result)
  {
    thinkfitCommand = 0;
    if (result < 0)
    {
      sendDebug->print("THINKFIT - Error= ");
      sendDebug->println(thinkfitGetErrorText(result));
    }
  }
#pragma endregion
}