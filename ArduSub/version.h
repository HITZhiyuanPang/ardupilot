#pragma once

// version.h - ArduSub 固件版本定义
// 警告：不要直接包含此文件！应包含 AP_Common/AP_FWVersion.h
// 此文件定义当前固件的版本号，用于 MAVLink HEARTBEAT 和地面站版本显示

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduSub V4.8.0-dev"  // 固件版本字符串

// autotest 脚本解析此行以确定版本
#define FIRMWARE_VERSION 4,8,0,FIRMWARE_VERSION_TYPE_DEV

// 版本号各字段
#define FW_MAJOR 4
#define FW_MINOR 8
#define FW_PATCH 0
#define FW_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
#include <AP_CheckFirmware/AP_CheckFirmwareDefine.h>
