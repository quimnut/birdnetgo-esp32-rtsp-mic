#pragma once
#include <Arduino.h>

// Web UI (ESP32 RTSP Mic for BirdNET-Go): initialization and request handling
void webui_begin();
void webui_handleClient();

// Push a log line from main into the Web UI ring buffer
void webui_pushLog(const String &line);

// Microphone type detection for WebUI
// PDM microphones don't use shift bits
#if defined(MIC_TYPE_PDM)
    #define WEBUI_HAS_SHIFT_BITS 0
#else
    #define WEBUI_HAS_SHIFT_BITS 1
#endif
