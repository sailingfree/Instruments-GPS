#pragma once

#include <Arduino.h>

extern Stream* Console;

void initOTA();
void handleOta();
