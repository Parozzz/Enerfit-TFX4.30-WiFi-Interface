#include <ThinkFitCommand.h>

ThinkfitCommand THINKFIT_SEND_OBTAIN_ID("Obtain ID", THINKFIT_OBTAIN_ID_BYTE1, THINKFIT_OBTAIN_ID_BYTE2);
ThinkfitCommand THINKFIT_RECV_OBTAIN_ID("Obtain ID", THINKFIT_OBTAIN_ID_BYTE1, THINKFIT_OBTAIN_ID_BYTE2);

ThinkfitCommand THINKFIT_SEND_OBTAIN_PARAMS("Obtain Params", THINKFIT_OBTAIN_PARAMS_BYTE1, THINKFIT_OBTAIN_PARAMS_BYTE2);
ThinkfitCommand THINKFIT_RECV_OBTAIN_PARAMS("Obtain Params", THINKFIT_OBTAIN_PARAMS_BYTE1, THINKFIT_OBTAIN_PARAMS_BYTE2);

ThinkfitCommand THINKFIT_SEND_OBTAIN_ACCUMULATIONS("Obtain Accumulation", THINKFIT_OBTAIN_ACCUMULATIONS_BYTE1, THINKFIT_OBTAIN_ACCUMULATIONS_BYTE2);
ThinkfitCommand THINKFIT_RECV_OBTAIN_ACCUMULATIONS("Obtain Accumulation", THINKFIT_OBTAIN_ACCUMULATIONS_BYTE1, THINKFIT_OBTAIN_ACCUMULATIONS_BYTE2);

ThinkfitCommand THINKFIT_SEND_OBTAIN_STATUS("Obtain Status", THINKFIT_STATUS_CMDBYTE1);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_STANDBY("Obtain Status - Standby", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_STANDBY);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_STARTING("Obtain Status - Starting", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_STARTING);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_RUNNING("Obtain Status - Running", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_RUNNING);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_STOPPING("Obtain Status - Stopping", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_STOPPING);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_SLEEP("Obtain Status - Sleep", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_SLEEP);
ThinkfitCommand THINKFIT_RECV_OBTAIN_STATUS_MALFUNCTION("Obtain Status - Malfunction", THINKFIT_STATUS_CMDBYTE1, THINKFIT_STATUS_MALFUNCTION);

ThinkfitCommand THINKFIT_SEND_START("Start", THINKFIT_START_BYTE1, THINKFIT_START_BYTE2);
ThinkfitCommand THINKFIT_RECV_START("Start", THINKFIT_START_BYTE1, THINKFIT_START_BYTE2);

ThinkfitCommand THINKFIT_STOP("Stop", THINKFIT_STOP_BYTE1, THINKFIT_STOP_BYTE2);

ThinkfitCommand THINKFIT_SEND_SET_SPEED_AND_SLOPE("SetResistanceAndSlope", THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE1, THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE2);
ThinkfitCommand THINKFIT_RECV_SET_SPEED_AND_SLOPE("SetResistanceAndSlope", THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE1, THINKFIT_SET_RESISTANCE_AND_SLOPE_BYTE2);

ThinkfitCommand *thinkfitSendCommands[] = {
    &THINKFIT_SEND_OBTAIN_ID,
    &THINKFIT_SEND_OBTAIN_PARAMS,
    &THINKFIT_SEND_OBTAIN_ACCUMULATIONS,
    &THINKFIT_SEND_OBTAIN_STATUS,
    &THINKFIT_SEND_START,
    &THINKFIT_STOP,
    &THINKFIT_SEND_SET_SPEED_AND_SLOPE,
};

ThinkfitCommand *thinkfitRecvCommands[] = {
    &THINKFIT_RECV_OBTAIN_ID, &THINKFIT_RECV_OBTAIN_PARAMS, &THINKFIT_RECV_OBTAIN_ACCUMULATIONS,
    &THINKFIT_RECV_OBTAIN_STATUS_STANDBY, &THINKFIT_RECV_OBTAIN_STATUS_STARTING, &THINKFIT_RECV_OBTAIN_STATUS_RUNNING,
    &THINKFIT_RECV_OBTAIN_STATUS_STOPPING, &THINKFIT_RECV_OBTAIN_STATUS_SLEEP, &THINKFIT_RECV_OBTAIN_STATUS_MALFUNCTION,
    &THINKFIT_RECV_START, &THINKFIT_STOP,
    &THINKFIT_RECV_SET_SPEED_AND_SLOPE};

const char *thinkfitGetErrorText(int8_t error)
{
    switch (error)
    {
    case THINKFIT_ERROR_RECV_INVALID_STARTCODE:
        return "Invalid Start code";
    case THINKFIT_ERROR_RECV_COMMAND_NOT_FOUND:
        return "Command not found";
    case THINKFIT_ERROR_RECV_INVALID_CHECKSUM:
        return "Checksum invalid";
    case THINKFIT_ERROR_RECV_INVALID_ENDCODE:
        return "Invalid End code";
    case THINKFIT_ERROR_RECV_TIMEOUT:
        return "Timeout recv";
    case THINKFIT_ERROR_RECV_WRONG_COMMAND:
        return "Wrong command received";
    default:
        return "Error not found";
    }
}

Stream *debugStream;
Stream *deviceStream;

void thinkfitSetDebugStream(Stream *pDebugStream)
{
    debugStream = pDebugStream;
}

void thinkfitInit(Stream *pDeviceStream)
{
    deviceStream = pDeviceStream;

    // These needs to be in order of reiceving
    THINKFIT_RECV_OBTAIN_ID.addData("Brand", 1)
        ->addData("Type", 1)
        ->addData("Machine", 2);

    THINKFIT_RECV_OBTAIN_PARAMS.addData("Resistance", 1)
        ->addData("Incline", 1)
        ->addData("Configuration", 1);
    //->addData("Reservation", 1);

    THINKFIT_RECV_OBTAIN_ACCUMULATIONS.addData("Accumulation", 4);

    THINKFIT_RECV_OBTAIN_STATUS_STARTING
        .addData("Countdown", 1);
    THINKFIT_RECV_OBTAIN_STATUS_RUNNING
        .addData("Speed", 1)
        ->addData("Slope", 1)
        ->addData("Time", 2)     // Seconds
        ->addData("Distance", 2) // mm
        ->addData("Calorie", 2)
        ->addData("Reservation", 2)
        ->addData("Heartbeat", 2);
    THINKFIT_RECV_OBTAIN_STATUS_STOPPING
        .addData("Speed", 1)
        ->addData("Slope", 1)
        ->addData("Time", 2)     // Seconds
        ->addData("Distance", 2) // mm
        ->addData("Calorie", 2)
        ->addData("Reservation", 2)
        ->addData("Heartbeat", 2);
    THINKFIT_RECV_OBTAIN_STATUS_MALFUNCTION
        .addData("Code", 1);

    THINKFIT_SEND_START
        .addData("Reserved1", 4)
        ->addData("Reserved2", 4);
    THINKFIT_RECV_START
        .addData("Countdown", 1);

    THINKFIT_SEND_SET_SPEED_AND_SLOPE
        .addData("Speed", 1)
        ->addData("Slope", 1);
    THINKFIT_RECV_SET_SPEED_AND_SLOPE
        .addData("Speed", 1)
        ->addData("Slope", 1);
}

struct
{
    bool waitingData;

    ThinkfitCommand *command;
    uint8_t commandByte1;
    uint8_t commandByte2;

    uint8_t dataByteCounter = 0;
    int8_t dataCounter = 0;

    uint8_t buffer[BUFFER_MAX_LEN];
    uint8_t count = 0;

    int8_t status;

    void end(int8_t pStatus)
    {
        waitingData = false;

        commandByte1 = commandByte2 = 0;
        dataByteCounter = dataCounter = 0;

        memset(buffer, 0, BUFFER_MAX_LEN);
        count = 0;

        status = pStatus;
    }

} recv;

uint32_t timestamp;
void thinkfitLoop()
{
    recv.status = THINKFIT_COMMSTATUS_STANDBY;
    while (deviceStream->available())
    {
        uint8_t read = deviceStream->read();
        recv.buffer[recv.count++] = read;

        if (debugStream)
            debugStream->printf("RECV - 0x%02x, Count= %d", read, recv.count);

        if (!recv.waitingData) // I need to wait for the command to be sent first since all the data i receive is a response of a command.
        {
            if (debugStream)
                debugStream->println();

            recv.count = 0;
            continue;
        }

        if (recv.count == 1)
        {
            if (read != THINKFIT_START_CODE)
            {
                if (debugStream)
                    debugStream->println(", INVALID START CODE");
                recv.count = 0;
            }
        }
        else if (recv.count == 2)
        {
            recv.commandByte1 = read;
            if (debugStream)
                debugStream->printf(", CommandByte1= %x", recv.commandByte1);
        }
        else if (recv.count == 3)
        {
            recv.commandByte2 = read;
            if (debugStream)
                debugStream->printf(", CommandByte2= %x", recv.commandByte2);

            recv.command = nullptr;
            for (uint8_t x = 0; x < (sizeof(thinkfitRecvCommands) / sizeof(ThinkfitCommand *)); x++)
            {
                ThinkfitCommand *command = thinkfitRecvCommands[x];
                if (command->getCommandByte1() == recv.commandByte1 && command->getCommandByte2() == recv.commandByte2)
                {
                    if (debugStream)
                        debugStream->printf(", Command found= %s, Len= %d", command->getName(), command->getMessageLen());

                    (recv.command = command)->clearAllData();
                }
            }

            if (!recv.command)
            {
                recv.end(THINKFIT_ERROR_RECV_COMMAND_NOT_FOUND);
            }
        }
        else if (recv.command)
        {
            uint8_t messageLen = recv.command->getMessageLen();
            if (recv.count < (messageLen - 1))
            {
                ThinkfitData *data = recv.command->appendDataValueAt(recv.dataCounter, read);
                if (data && ++recv.dataByteCounter == data->len)
                {
                    if (debugStream)
                        debugStream->printf(", %s= %d", data->name, data->value);

                    recv.dataByteCounter = 0;
                    recv.dataCounter++;
                }
            }
            else if (recv.count == (messageLen - 1))
            {
                uint8_t xorChecksum = 0;
                for (int x = 1; x < (messageLen - 2); x++) //-2 otherwise i will inclusde the checksum itself inside and return 0.
                {
                    xorChecksum = xorChecksum ^ recv.buffer[x];
                }

                bool checksumValid = (read == xorChecksum);
                if (!checksumValid)
                {
                    recv.end(THINKFIT_ERROR_RECV_INVALID_CHECKSUM);
                }

                if (debugStream)
                    debugStream->printf(", Checksum %s.", checksumValid ? "VALID" : "INVALID");
            }
            else if (recv.count >= messageLen)
            {
                if (debugStream)
                    debugStream->print(", end");

                recv.end(read == THINKFIT_END_CODE
                             ? THINKFIT_COMMSTATUS_DONE
                             : THINKFIT_ERROR_RECV_INVALID_ENDCODE);
            }
        }

        if (debugStream)
            debugStream->println();
    }

    if (recv.waitingData && millis() - timestamp > 1000)
    {
        recv.end(THINKFIT_ERROR_RECV_TIMEOUT);
    }
}

void thinkfitSendCommand(ThinkfitCommand *sendCommand)
{
    if (sendCommand)
    {
        uint8_t buffer[BUFFER_MAX_LEN];
        uint8_t len = sendCommand->generateBuffer(buffer);
        for (int x = 0; x < len; x++)
        {
            deviceStream->write(buffer[x]);
        }

        timestamp = millis();
        recv.command = nullptr;
        recv.waitingData = true;
    }
}

int8_t thinkfitCommObtainParams(ThinkfitParams *params)
{
    if (recv.status == THINKFIT_COMMSTATUS_DONE)
    {
        if (recv.command != &THINKFIT_RECV_OBTAIN_PARAMS)
        {
            return THINKFIT_ERROR_RECV_WRONG_COMMAND;
        }

        params->clearData();
        params->maxResistance = THINKFIT_RECV_OBTAIN_PARAMS.getDataValueFor("Resistance");
        params->maxSlope = THINKFIT_RECV_OBTAIN_PARAMS.getDataValueFor("Incline");
        params->unit = bitRead(THINKFIT_RECV_OBTAIN_PARAMS.getDataValueFor("Configuration"), 0);
    }
    else if (!recv.status && !recv.waitingData)
    {
        thinkfitSendCommand(&THINKFIT_SEND_OBTAIN_PARAMS);
    }

    return recv.status;
}

int8_t thinkfitCommStatus(ThinkfitStatusData *statusData)
{
    if (recv.status == THINKFIT_COMMSTATUS_DONE)
    {
        if (recv.command->getCommandByte1() != THINKFIT_STATUS_CMDBYTE1)
        {
            return THINKFIT_ERROR_RECV_WRONG_COMMAND;
        }

        statusData->clearData();
        if (recv.command == &THINKFIT_RECV_OBTAIN_STATUS_STARTING)
        {
            statusData->updateFromCommand(&THINKFIT_RECV_OBTAIN_STATUS_STARTING);
        }
        else if (recv.command == &THINKFIT_RECV_OBTAIN_STATUS_RUNNING)
        {
            statusData->updateFromCommand(&THINKFIT_RECV_OBTAIN_STATUS_RUNNING);
        }
        else if (recv.command == &THINKFIT_RECV_OBTAIN_STATUS_STOPPING)
        {
            statusData->updateFromCommand(&THINKFIT_RECV_OBTAIN_STATUS_STOPPING);
        }
        else if (recv.command == &THINKFIT_RECV_OBTAIN_STATUS_MALFUNCTION)
        {
            statusData->updateFromCommand(&THINKFIT_RECV_OBTAIN_STATUS_MALFUNCTION);
        }
    }
    else if (!recv.status && !recv.waitingData)
    {
        thinkfitSendCommand(&THINKFIT_SEND_OBTAIN_STATUS);
    }

    return recv.status;
}

int8_t thinkfitCommStart()
{
    if (recv.status == THINKFIT_COMMSTATUS_DONE)
    {
        if (recv.command != &THINKFIT_RECV_START)
        {
            return THINKFIT_ERROR_RECV_WRONG_COMMAND;
        }
    }
    else if (!recv.status && !recv.waitingData)
    {
        thinkfitSendCommand(&THINKFIT_SEND_START);
    }

    return recv.status;
}

int8_t thinkfitCommStop()
{
    if (recv.status == THINKFIT_COMMSTATUS_DONE)
    {
        if (recv.command != &THINKFIT_STOP)
        {
            return THINKFIT_ERROR_RECV_WRONG_COMMAND;
        }
    }
    else if (!recv.status && !recv.waitingData)
    {
        thinkfitSendCommand(&THINKFIT_STOP);
    }

    return recv.status;
}

int8_t thinkfitCommSetSpeedSlope(ThinkfitSpeedSlopeData *speedSlopeData)
{
    if (recv.status == THINKFIT_COMMSTATUS_DONE)
    {
        if (recv.command != &THINKFIT_RECV_SET_SPEED_AND_SLOPE)
        {
            return THINKFIT_ERROR_RECV_WRONG_COMMAND;
        }
    }
    else if (!recv.status && !recv.waitingData)
    {
        THINKFIT_SEND_SET_SPEED_AND_SLOPE.setDataValueFor("Slope", speedSlopeData->slope);
        THINKFIT_SEND_SET_SPEED_AND_SLOPE.setDataValueFor("Speed", speedSlopeData->speed);
        thinkfitSendCommand(&THINKFIT_SEND_SET_SPEED_AND_SLOPE);
    }

    return recv.status;
}