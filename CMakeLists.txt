cmake_minimum_required(VERSION 3.16)

# esp32-p4-nano-mpempe — multitouch MPE controller that streams MIDI
# (RTP-MIDI / "AppleMIDI") and OSC over WiFi, running on the
# Waveshare ESP32-P4-Nano + EK79007 1024x600 DSI panel + GT911 5-pt
# capacitive touch. Same hardware as the sibling -browser / -espdisp
# projects; reuses the proven NC/NC GT911 init pattern from the
# browser tree.

set(IDF_TARGET esp32p4)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(esp32p4_mpempe)
