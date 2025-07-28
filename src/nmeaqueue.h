// Protected queue for n2k messages to be sent
#pragma once

#include <N2kMsg.h>

#define MSGQLEN 10

// init the queue
void setupMsgQueue(void);

// Add a message to the queue if there is room
bool queueMsg(tN2kMsg & msg);

// Get the next message
bool dequeueMsg(tN2kMsg & msg);