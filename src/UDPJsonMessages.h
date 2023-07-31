#ifndef UDP_JSON_MESSAGES
#define UDP_JSON_MESSAGES

#include <Arduino.h>
#include <cJSON.h>
#include <cJSON_Utils.h>
#include <ThinkFitCommand.h>

void printJSONUnformatted(Print *print, cJSON* json)
{
    char *jsonString = cJSON_PrintUnformatted(json);
    print->println(jsonString);
    print->flush();
    free(jsonString);
}

size_t fillJSONBuffer(uint8_t* buffer, cJSON* json)
{
    char *jsonString = cJSON_PrintUnformatted(json);

    size_t size = strlen(jsonString);
    memcpy(buffer, jsonString, size);
    free(jsonString);
    return size;
}

size_t fillJSONStatusData(uint8_t* buffer, ThinkfitStatusData* statusData, bool presence)
{
    cJSON *mainJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "Starting", cJSON_CreateBool(statusData->status == THINKFIT_STATUS_STARTING));
    cJSON_AddItemToObject(mainJson, "Running", cJSON_CreateBool(statusData->status == THINKFIT_STATUS_RUNNING));
    cJSON_AddItemToObject(mainJson, "Stopping", cJSON_CreateBool(statusData->status == THINKFIT_STATUS_STOPPING));
    cJSON_AddItemToObject(mainJson, "Presence", cJSON_CreateBool(presence));

    cJSON_AddItemToObject(mainJson, "Speed", cJSON_CreateNumber(statusData->currentSpeed));
    cJSON_AddItemToObject(mainJson, "Slope", cJSON_CreateNumber(statusData->currentSlope));
    cJSON_AddItemToObject(mainJson, "Time", cJSON_CreateNumber(statusData->currentTime));
    cJSON_AddItemToObject(mainJson, "Distance", cJSON_CreateNumber(statusData->currentDistance));
    cJSON_AddItemToObject(mainJson, "Calorie", cJSON_CreateNumber(statusData->currentCalorie));
    cJSON_AddItemToObject(mainJson, "Heartbeat", cJSON_CreateNumber(statusData->currentHeartbeat));

    cJSON_AddItemToObject(mainJson, "MalfunctionCode", cJSON_CreateNumber(statusData->mulfunctionCode));
    cJSON_AddItemToObject(mainJson, "Countdown", cJSON_CreateNumber(statusData->startCountdown));

    size_t size = fillJSONBuffer(buffer, mainJson);
    cJSON_Delete(mainJson);
    return size;
}

size_t fillJSONUpdateRequest(uint8_t* buffer)
{
    cJSON *mainJson = cJSON_CreateObject();
    cJSON_AddItemToObject(mainJson, "UpdateRequest", cJSON_CreateBool(true));

    size_t size = fillJSONBuffer(buffer, mainJson);
    cJSON_Delete(mainJson);
    return size;
}


#endif