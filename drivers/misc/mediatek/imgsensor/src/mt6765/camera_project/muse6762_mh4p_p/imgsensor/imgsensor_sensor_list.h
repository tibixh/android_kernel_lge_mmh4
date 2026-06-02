/*
 * Copyright (C) 2015 MediaTek Inc.
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 */

#ifndef __KD_SENSORLIST_H__
#define __KD_SENSORLIST_H__

#include "kd_camera_typedef.h"
#include "imgsensor_sensor.h"

struct IMGSENSOR_INIT_FUNC_LIST {
	MUINT32   id;
	MUINT8    name[32];
	MUINT32 (*init)(struct SENSOR_FUNCTION_STRUCT **pfFunc);
};

/*OV*/
UINT32 OV16885_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 OV13855_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

/*S5K*/
UINT32 S5K3L6TXD_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3L6_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 S5K3P9_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

/*HI*/
UINT32 HI1336_MIPI_RAW_SensorInit (struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 HI556_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

/*GC*/
UINT32 GC2375H_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC2375HTXD_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 GC5035_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

/*SP*/
UINT32 SP2509V_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);
UINT32 SP2509V_DUAL_MIPI_RAW_SensorInit(struct SENSOR_FUNCTION_STRUCT **pfFunc);

extern struct IMGSENSOR_INIT_FUNC_LIST kdSensorList[];

#endif

