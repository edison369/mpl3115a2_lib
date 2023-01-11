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
 *  The Sensor MPL3115A2 header file containing version information
 */

#ifndef MPL3115A2_VERSION_H
#define MPL3115A2_VERSION_H

/* Development Build Macro Definitions */

#define MPL3115A2_BUILD_NUMBER 32 /*!< Development Build: Number of commits since baseline */
#define MPL3115A2_BUILD_BASELINE \
    "v1.3.0-rc4" /*!< Development Build: git tag that is the base for the current development */

/*
 * Version Macros, see \ref cfsversions for definitions.
 */
#define MPL3115A2_MAJOR_VERSION 1  /*!< @brief Major version number */
#define MPL3115A2_MINOR_VERSION 1  /*!< @brief Minor version number */
#define MPL3115A2_REVISION      99 /*!< @brief Revision version number. Value of 99 indicates a development version.*/

/*!
 * @brief Mission revision.
 *
 * Set to 0 on OFFICIAL releases, and set to 255 (0xFF) on development versions.
 * Values 1-254 are reserved for mission use to denote patches/customizations as needed.
 */

/*!
 * @brief Mission revision.
 *
 * Reserved for mission use to denote patches/customizations as needed.
 * Values 1-254 are reserved for mission use to denote patches/customizations as needed. NOTE: Reserving 0 and 0xFF for
 * cFS open-source development use (pending resolution of nasa/cFS#440)
 */
#define MPL3115A2_MISSION_REV 0xFF

#define MPL3115A2_STR_HELPER(x) #x /*!< @brief Helper function to concatenate strings from integer macros */
#define MPL3115A2_STR(x) \
    MPL3115A2_STR_HELPER(x) /*!< @brief Helper function to concatenate strings from integer macros */

/*! @brief Development Build Version Number.
 * @details Baseline git tag + Number of commits since baseline. @n
 * See @ref cfsversions for format differences between development and release versions.
 */
#define MPL3115A2_VERSION MPL3115A2_BUILD_BASELINE "+dev" MPL3115A2_STR(MPL3115A2_BUILD_NUMBER)

/*! @brief Development Build Version String.
 * @details Reports the current development build's baseline, number, and name. Also includes a note about the latest
 * official version. @n See @ref cfsversions for format differences between development and release versions.
 */
#define MPL3115A2_VERSION_STRING                       \
    " Sensor MPL3115A2 DEVELOPMENT BUILD " MPL3115A2_VERSION \
    ", Last Official Release: v1.1.0" /* For full support please use this version */

#endif /* MPL3115A2_VERSION_H */

/************************/
/*  End of File Comment */
/************************/
