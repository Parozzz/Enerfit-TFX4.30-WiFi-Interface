#ifndef THINKFIT_COMMAND
#define THINKFIT_COMMAND

#include <Arduino.h>

#define BUFFER_MAX_LEN 64
#define MAX_DATA_COUNT 10

#define THINKFIT_START_CODE 0x02
#define THINKFIT_END_CODE 0x03

#define THINKFIT_COMMSTATUS_STANDBY 0
#define THINKFIT_COMMSTATUS_DONE 1
#define THINKFIT_ERROR_RECV_INVALID_STARTCODE -1
#define THINKFIT_ERROR_RECV_COMMAND_NOT_FOUND -2
#define THINKFIT_ERROR_RECV_INVALID_CHECKSUM -3
#define THINKFIT_ERROR_RECV_INVALID_ENDCODE -4
#define THINKFIT_ERROR_RECV_TIMEOUT -5
#define THINKFIT_ERROR_RECV_WRONG_COMMAND -6

//=======================================
//========== THINKFIT COMMANDS ==========
#define THINKFIT_OBTAIN_ID_BYTE1 0x50
#define THINKFIT_OBTAIN_ID_BYTE2 0x00

#define THINKFIT_OBTAIN_PARAMS_BYTE1 0x50
#define THINKFIT_OBTAIN_PARAMS_BYTE2 0x02

#define THINKFIT_OBTAIN_ACCUMULATIONS_BYTE1 0x50
#define THINKFIT_OBTAIN_ACCUMULATIONS_BYTE2 0x03

#define THINKFIT_START_BYTE1 0x53
#define THINKFIT_START_BYTE2 0x01

#define THINKFIT_STOP_BYTE1 0x53
#define THINKFIT_STOP_BYTE2 0x03

#define THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE1 0x53
#define THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE2 0x02

#define THINKFIT_STATUS_CMDBYTE1 0x51
#define THINKFIT_STATUS_STANDBY 0x00
#define THINKFIT_STATUS_STARTING 0x02
#define THINKFIT_STATUS_RUNNING 0x03
#define THINKFIT_STATUS_STOPPING 0x04
#define THINKFIT_STATUS_SLEEP 0xF1
#define THINKFIT_STATUS_MALFUNCTION 0xF2
//=======================================

struct ThinkfitData
{
    const char *name;
    uint8_t len;
    uint32_t value = 0;
    uint8_t valueCounter = 0;

    void reset()
    {
        value = 0;
        valueCounter = 0;
    }

    void appendByte(uint8_t read)
    {
        value = value | (((uint32_t)read) << (valueCounter++ * 8)); // LSB
    }
};

class ThinkfitCommand
{
public:
    // If commandByte2 == 0xFF means is a one byte command.
    ThinkfitCommand(const char *name, uint8_t commandByte1, uint8_t commandByte2 = 0xFF)
    {
        _name = name;
        _commandByte1 = commandByte1;
        _commandByte2 = commandByte2;

        calculateMessageLen();
    }

    const char *getName()
    {
        return _name;
    }

    bool isCommandShort()
    {
        return _commandByte2 == 0xFF;
    }

    ThinkfitCommand *addData(const char *name, uint8_t len)
    {
        if (_dataCount < MAX_DATA_COUNT)
        {
            ThinkfitData *data = &_dataArray[_dataCount++];
            data->name = name;
            data->len = len;
        }

        calculateMessageLen();
        return this;
    }

    void clearAllData()
    {
        for (int x = 0; x < _dataCount; x++)
        {
            _dataArray[x].reset();
        }
    }

    ThinkfitData *setDataValueFor(const char *name, uint32_t value)
    {
        ThinkfitData *data = getDataFor(name);
        if (data)
        {
            data->value = value;
        }
        return data;
    }

    uint32_t getDataValueFor(const char *name)
    {
        ThinkfitData *data = getDataFor(name);
        if (data)
        {
            return data->value;
        }

        return 0xFFFFFFFF;
    }

    ThinkfitData *appendDataValueAt(uint8_t index, uint8_t read)
    {
        if (index < _dataCount)
        {
            ThinkfitData *data = &_dataArray[index];
            data->appendByte(read);
            return data;
        }

        return nullptr;
    }

    uint8_t getDataCount()
    {
        return _dataCount;
    }

    uint8_t getCommandByte1()
    {
        return _commandByte1;
    }

    uint8_t getCommandByte2()
    {
        return _commandByte2;
    }

    uint8_t getMessageLen()
    {
        return _messageLen;
    }

    uint8_t generateBuffer(uint8_t *buffer)
    {
        buffer[0] = THINKFIT_START_CODE;
        buffer[1] = _commandByte1;

        uint8_t dataIndex;
        if (!isCommandShort())
        {
            buffer[2] = _commandByte2;
            dataIndex = 3;
        }
        else
        {
            dataIndex = 2;
        }

        for (int x = 0; x < _dataCount; x++)
        {
            ThinkfitData *data = &_dataArray[x];
            for (int y = 0; y < data->len; y++)
            {
                // DATA IS WROTE FOLLOWING LITTLE-ENDIAN
                uint8_t dataByte = (data->value >> (y * 8)) & 0xFF;
                buffer[dataIndex++] = dataByte;
            }
        }

        uint8_t xorChecksum = 0;
        for (int x = 1; x < dataIndex; x++)
        {
            xorChecksum ^= buffer[x];
        }

        buffer[dataIndex++] = xorChecksum;
        buffer[dataIndex++] = THINKFIT_END_CODE;
        return dataIndex;
    }

private:
    ThinkfitData _dataArray[MAX_DATA_COUNT];
    uint8_t _dataCount;
    uint8_t _messageLen;

    const char *_name;
    uint8_t _commandByte1;
    uint8_t _commandByte2;

    ThinkfitData *getDataFor(const char *name)
    {
        for (int x = 0; x < _dataCount; x++)
        {
            ThinkfitData *data = &_dataArray[x];
            if (strcmp(name, data->name) == 0)
            {
                return data;
            }
        }

        return nullptr;
    }

    void calculateMessageLen()
    {
        uint8_t dataLen = 0;
        for (int x = 0; x < _dataCount; x++)
        {
            dataLen += _dataArray[x].len;
        }
        // 3 because START_CODE + END_CODE + XOR CHECKSUM
        _messageLen = 3 + (isCommandShort() ? 1 : 2) + dataLen;
    }
};

struct ThinkfitParams
{
    uint8_t maxResistance;
    uint8_t maxSlope;
    uint8_t unit; // 0 = Km, 1 = Miles

    void clearData()
    {
        memset(this, 0, sizeof(ThinkfitParams));
    }

    void updateFromCommand(ThinkfitCommand *command)
    {
        if (command->getCommandByte1() == THINKFIT_OBTAIN_PARAMS_BYTE1 && command->getCommandByte2() == THINKFIT_OBTAIN_PARAMS_BYTE2)
        {
            maxResistance = command->getDataValueFor("Resistance");
            maxSlope = command->getDataValueFor("Incline");
            unit = bitRead(command->getDataValueFor("Configuration"), 0);
        }
    }
};

struct ThinkfitStatusData
{
    uint8_t status;

    uint8_t startCountdown;

    uint8_t currentSpeed;     // 0.1 Km/h
    uint8_t currentSlope;     // Degree
    uint16_t currentTime;     // Seconds
    uint16_t currentDistance; // mm
    uint16_t currentCalorie;  // 0.1 Kj
    uint16_t currentHeartbeat;

    uint8_t mulfunctionCode;

    void clearData()
    {
        memset(this, 0, sizeof(ThinkfitStatusData));
    }

    void updateFromCommand(ThinkfitCommand *command)
    {
        if (command->getCommandByte1() == THINKFIT_STATUS_CMDBYTE1)
        {
            status = command->getCommandByte2();
            switch (status)
            {
            case THINKFIT_STATUS_STARTING:
                startCountdown = command->getDataValueFor("Countdown");
                break;
            case THINKFIT_STATUS_RUNNING:
            case THINKFIT_STATUS_STOPPING:
                currentSpeed = command->getDataValueFor("Speed");
                currentSlope = command->getDataValueFor("Slope");
                currentTime = command->getDataValueFor("Time");
                currentDistance = command->getDataValueFor("Distance");
                currentCalorie = command->getDataValueFor("Calorie");
                currentHeartbeat = command->getDataValueFor("Heartbeat");
                break;
            case THINKFIT_STATUS_MALFUNCTION:
                mulfunctionCode = command->getDataValueFor("Code");
                break;
            case THINKFIT_STATUS_STANDBY:
            case THINKFIT_STATUS_SLEEP:
            default:
                break;
            }
        }
    }
};

struct ThinkfitSpeedSlopeData
{
    uint8_t speed; // 0.1Km/h (10 = 1Km/h)
    uint8_t slope; // Degree
};

const char *thinkfitGetErrorText(int8_t error);

void thinkfitSetDebugStream(Stream *debugStream);

void thinkfitInit(Stream *deviceStream);

void thinkfitLoop();

int8_t thinkfitCommObtainParams(ThinkfitParams *params);

int8_t thinkfitCommStart();

int8_t thinkfitCommStop();

int8_t thinkfitCommStatus(ThinkfitStatusData *statusData);

int8_t thinkfitCommSetSpeedSlope(ThinkfitSpeedSlopeData *speedSlopeData);

#endif