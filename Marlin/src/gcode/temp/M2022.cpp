/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * gcode/temp/M104_M109.cpp
 *
 * Current Setpoint Control
 */
#include "../../myglobal.h"
#include "../../inc/MarlinConfigPre.h"
#include "../gcode.h"

void GcodeSuite::M2022() {

    const char dterm = 'D';
    duty_cycle = parser.intval(dterm,-1);
    SERIAL_ECHO_START();
    SERIAL_ECHO("Duty Cycle Changed");
}