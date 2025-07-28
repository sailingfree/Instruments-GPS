// Queue for the n2k messages that have been parsed

#include <Arduino.h>
#include <nmeaqueue.h>
#include <N2kMsg.h>

static QueueHandle_t msgQueue;

void setupMsgQueue() {
    msgQueue = xQueueCreate(MSGQLEN, sizeof(tN2kMsg));
}

bool queueMsg(tN2kMsg & msg) {
    bool result;
//    Serial.printf("Queue: PGN %ld\n", msg.PGN);

    result =  xQueueSend(msgQueue, (void *) & msg, 0);
    if(result) {
//        Serial.printf("Adding msg %ld\n", msg.PGN);
    } else { 
//        Serial.printf("Failed to add msg %ld\n", msg.PGN);

    }
    return result;
}

bool dequeueMsg(tN2kMsg & msg) {
    bool result =  xQueueReceive(msgQueue, (void *) & msg, portMAX_DELAY);
    if(result) {
//        Serial.printf("Dequeue PGN %d\n", msg.PGN);
    }
    return result;
}
