/*
 * This file is part of iNav
 *
 * iNav free software. You can redistribute
 * this software and/or modify this software under the terms of the
 * GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option)
 * any later version.
 *
 * iNav distributed in the hope that it
 * will be useful, but WITHOUT ANY WARRANTY; without even the implied
 * warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this software.
 *
 * If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include <common/time.h>

#include <telemetry/smartport.h>

#if defined(USE_SMARTPORT_MASTER)

typedef struct {
    int8_t count;
    int16_t voltage[6];
} cellsData_t;


bool smartportMasterInit(void);
void smartportMasterHandle(timeUs_t currentTimeUs);

// Returns latest received SmartPort payload for phyID or NULL if PhyID is not active
smartPortPayload_t *smartportMasterGetPayload(uint8_t phyID);

// Returns latest Cells data or NULL if the data is too old
cellsData_t *smartportMasterGetCellsData(void);

#endif /* USE_SMARTPORT_MASTER */
