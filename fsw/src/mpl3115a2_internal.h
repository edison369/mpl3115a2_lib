/************************************************************************
 * NASA Docket No. GSC-18,719-1, and identified as “core Flight System: Bootes”
 *
 * Copyright (c) 2020 United States Government as represented by the
 * Administrator of the National Aeronautics and Space Administration.
 * All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may
 * not use this file except in compliance with the License. You may obtain
 * a copy of the License at http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 ************************************************************************/

/**
 * @file
 *  An example of an internal (private) header file for SAMPLE Lib
 */
#ifndef MPL3115A2_INTERNAL_H
#define MPL3115A2_INTERNAL_H

/* Include all external/public definitions */
#include "mpl3115a2.h"

/*************************************************************************
** Macro Definitions
*************************************************************************/

#define MPL3115A2_BUFFER_SIZE 16

/*************************************************************************
** Internal Data Structures
*************************************************************************/
extern char MPL3115A2_Buffer[MPL3115A2_BUFFER_SIZE];

/*************************************************************************
** Function Declarations
*************************************************************************/

/**
 * Library initialization routine/entry point
 */
int32 MPL3115A2_Init(void);

int sensor_mpl3115a2_ioctl(i2c_dev *dev, ioctl_command_t command, void *arg);
int read_bytes(int fd, uint16_t i2c_address, uint8_t data_address, uint16_t nr_bytes, uint8_t **buff);
int set_bytes(uint16_t chip_address, uint8_t **val, int numBytes);

int sensor_mpl3115a2_set_reg_8(uint8_t register_add, uint8_t value);
int sensor_mpl3115a2_get_reg_8(uint8_t register_add, uint8_t **buff);

bool begin(void);
int readMeasurement(uint8_t **buff);

void sensor_mpl3115a2_startMeasurement(void);
bool sensor_mpl3115a2_conversionComplete(void);
float sensor_mpl3115a2_getLastConversionResults(mpl3115a2_meas_t value);
void sensor_mpl3115a2_setMode(mpl3115a2_mode_t mode);

#endif
