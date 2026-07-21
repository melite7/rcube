# R큐브 메인보드 펌웨어 (ESP32-S3) — 최상위 프로젝트 CMake
# 단일 바이너리가 부팅 분기로 멤버/edge central/BLE 허브/설정모드 역할 수행.
cmake_minimum_required(VERSION 3.16)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)

project(rcube_fw)
