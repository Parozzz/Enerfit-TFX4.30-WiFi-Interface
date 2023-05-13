#include <Arduino.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <secret.h>
#include <esp_efuse.h>
#include <esp_efuse_table.h>
#include <FIFO.h>

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
#define WIFI_RETRY_TIME_MILLIS 120 * 1000

#define DELAY_BETWEEN_READING_ON_START 250
#define DELAY_BETWEEN_READING_ON_STOP 1500

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

uint32_t lastCmdTimestamp = 0;

bool otaUpdating = false;
void otaSetup()
{
  ArduinoOTA.setHostname("EnerfitTreadmill");

  ArduinoOTA.onStart([]()
                     {
    String type;

    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else
      type = "filesystem"; // U_FS
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
  if (WiFi.waitForConnectResult() == WL_CONNECTED)
  {
    return true;
  }

  // WiFi.setTxPower(wifi_power_t::WIFI_POWER_19_5dBm);
  WiFi.config(IP_ADDRESS, GATEWAY, SUBNET);
  WiFi.enableSTA(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  uint32_t lastTryTimestamp = millis();
  while (WiFi.waitForConnectResult() != WL_CONNECTED && (millis() - lastTryTimestamp) < WIFI_RETRY_TIME_MILLIS)
  {
    yield();
  }

  bool connected = WiFi.waitForConnectResult() == WL_CONNECTED;
  sendDebug->printf("WiFi connection status= %d, MAC= %x:%x:%x:%x:%x:%x\n",
                    connected,
                    WiFi.macAddress()[0],
                    WiFi.macAddress()[1],
                    WiFi.macAddress()[2],
                    WiFi.macAddress()[3],
                    WiFi.macAddress()[4],
                    WiFi.macAddress()[5]);
  return connected;
}

struct
{
  uint8_t command; // 1 = CMD START, 2 = CMD STOP, 3=SET SPEED, 4 = SET SLOPE
  uint8_t data;
} udpRecvData;
bool udpNewRecvData = false;

struct
{
  uint8_t status; //(b0 = starting, b1=running, b2=stopping, b3=presence detected)

  uint8_t currentSpeed; //[0.1Km/h]
  uint8_t currentSlope; //[°]
  uint8_t countdown;

  uint16_t currentTime;     //[s]
  uint16_t currentDistance; //[mm]
  uint16_t currentCalorie;  //[0.1 Kj]

  uint8_t malfunctionCode;
  uint8_t reserved;
} udpSendData;
bool udpExecuteSendData = false;
bool udpExecuteRefresh = false;

bool udpInitialized = false;
uint8_t udpBuffer[1024];
void udp()
{
  if (!udpInitialized)
  {
    udpInitialized = true;

    UDP.begin(UDP_LISTEN_PORT);
    UDP.setTimeout(1000);
  }

  int packetSize = UDP.parsePacket();
  if (packetSize)
  {
    UDP.read(udpBuffer, packetSize);
    if (packetSize < sizeof(udpRecvData))
    {
      sendDebug->printf("UDP - Invalid size. Recv: %d, Expected: %d\n", packetSize, sizeof(udpRecvData));
      return;
    }

    memcpy(&udpRecvData, udpBuffer, sizeof(udpRecvData));
    udpNewRecvData = true;

    sendDebug->print("UDP - Packet received. [");
    for (int x = 0; x < packetSize; x++)
    {
      sendDebug->printf("%x, ", udpBuffer[x]);
    }
    sendDebug->println("]");
  }

  if (udpExecuteSendData)
  {
    udpExecuteSendData = false;

    uint8_t *pSend = (uint8_t *)&udpSendData;

    UDP.beginPacket(UDP_SEND_HOST, UDP_SEND_PORT);
    for (int x = 0; x < sizeof(udpSendData); x++)
    {
      UDP.write(pSend[x]);
    }
    UDP.endPacket();

    sendDebug->println("UDP - Packet sent");
  }

  if(udpExecuteRefresh)
  {
    udpExecuteRefresh = false;

    UDP.beginPacket(UDP_SEND_HOST, UDP_SEND_PORT);
    UDP.write('U');
    UDP.write('P');
    UDP.endPacket();
  }
}

VL53L1X distanceSensor;
bool distanceSensorOK = false;
uint16_t presenceDistance = 0; // mm

void distanceSensorInit()
{
  distanceSensor.setTimeout(500);
  distanceSensorOK = distanceSensor.init();
  if (distanceSensorOK)
  {
    distanceSensor.setDistanceMode(VL53L1X::DistanceMode::Long);
    distanceSensor.setMeasurementTimingBudget(80000);
    distanceSensor.startContinuous(100);
  }
  else
  {
    sendDebug->println("Failed to detect and initialize sensor!");
  }
}

bool distanceSensorReadPresence()
{
  if (distanceSensorOK)
  {
    uint16_t distance = distanceSensor.read(false);
    if (distance)
    {
      return distance < presenceDistance;
    }
    else if (distanceSensor.timeoutOccurred())
    {
      distanceSensorOK = false;
    }
  }

  return false;
}

void setup()
{
  udpExecuteRefresh = true;
  udpExecuteSendData = true; //Send data at startup to avoid mismatch of values

  setSerialDebugEFuse();

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

  distanceSensorInit();

  if (wifi())
  {
    otaSetup();
  }

  thinkfitInit(&Serial0);
  sendDebug->println("THINKFIT INIT - DONE");
}

bool continousUpdate = true;
uint32_t delayBetweenCommands = DELAY_BETWEEN_READING_ON_STOP;
uint32_t periodicRefreshTimestamp;

void loop()
{
  if (wifi())
  {
    ArduinoOTA.handle();
    if (otaUpdating)
    {
      ble.stopAdvertising();
      return;
    }

    udp();
  }

  ble.loop();
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
        sendDebug->println("THINKFIT - Enable debug.");
        thinkfitSetDebugStream(sendDebug);
        break;
      case 'D':
      case 'd':
        sendDebug->println("THINKFIT - Disable debug.");
        thinkfitSetDebugStream(nullptr);
        break;
      case 'S':
      case 's':
        sendDebug->println("THINKFIT - STOP CONTINOUS UPDATE");
        continousUpdate = false;
        break;
      case 'C':
      case 'c':
        sendDebug->println("THINKFIT - STARTED CONTINOUS UPDATE");
        continousUpdate = true;
        break;
      }
    }
  }

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

  bool presence = distanceSensorReadPresence();
  if (presence != bitRead(udpSendData.status, 3))
  {
    bitWrite(udpSendData.status, 3, presence);
    udpExecuteSendData = true;

    sendDebug->printf("Presence changed %s\n", presence ? "ON" : "OFF");
  }

  if(statusData.status == THINKFIT_STATUS_STANDBY)
  {
    if(millis() - periodicRefreshTimestamp > 10800000) //About 3h
    {
      periodicRefreshTimestamp = millis();
      udpExecuteRefresh = true;
    }
  }
  else
  {
    periodicRefreshTimestamp = millis();
  }

  if (udpNewRecvData)
  {
    udpNewRecvData = false;

    switch (udpRecvData.command)
    {
    case UDP_CMD_START:
      thinkfitCommandsFifo.insert(CMD_START);
      break;
    case UDP_CMD_STOP:
      thinkfitCommandsFifo.insert(CMD_STOP);
      break;
    case UDP_CMD_SET_SPEED:
      speedSlopeData.speed = udpRecvData.data;
      thinkfitCommandsFifo.insert(CMD_SET_SPEED_SLOPE);
      break;
    case UDP_CMD_SET_SLOPE:
      speedSlopeData.slope = udpRecvData.data;
      thinkfitCommandsFifo.insert(CMD_SET_SPEED_SLOPE);
      break;
    case UDP_CMD_SET_DISTANCE:
      presenceDistance = udpRecvData.data * 10;
    }
  }

  int8_t result = 0;

  thinkfitLoop();
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
      if (statusData.status == THINKFIT_STATUS_STARTING || statusData.status == THINKFIT_STATUS_RUNNING)
      {
        delayBetweenCommands = DELAY_BETWEEN_READING_ON_START;
      }
      else
      {
        delayBetweenCommands = DELAY_BETWEEN_READING_ON_STOP;
      }

      //(b0 = starting, b1=running, b2=stopping, b3=presence detected)
      bitWrite(udpSendData.status, 0, statusData.status == THINKFIT_STATUS_STARTING);
      bitWrite(udpSendData.status, 1, statusData.status == THINKFIT_STATUS_RUNNING);
      bitWrite(udpSendData.status, 2, statusData.status == THINKFIT_STATUS_STOPPING);

      udpSendData.currentSpeed = statusData.currentSpeed;
      udpSendData.currentSlope = statusData.currentSlope;
      udpSendData.currentTime = statusData.currentTime;
      udpSendData.currentDistance = statusData.currentDistance;
      udpSendData.currentCalorie = statusData.currentCalorie;

      udpSendData.malfunctionCode = statusData.mulfunctionCode;
      udpSendData.countdown = statusData.startCountdown;

      udpExecuteSendData = true;
    }

    if (result)
    {
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

  // put your main code here, to run repeatedly:
}