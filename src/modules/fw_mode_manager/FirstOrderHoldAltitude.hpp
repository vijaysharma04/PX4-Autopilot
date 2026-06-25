/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include <uORB/topics/position_setpoint.h>

/**
 * @brief Calculate the altitude setpoint using an altitude first order hold (FOH).
 *
 * Linearly interpolates the altitude setpoint between the previous and current waypoint such that the
 * current waypoint altitude is reached at the acceptance radius around the current waypoint. The FOH is
 * only applied if the previous setpoint is valid and a position or loiter waypoint, otherwise the current
 * waypoint altitude is returned directly.
 *
 * @param prev_valid whether the previous position setpoint is valid
 * @param pos_sp_prev previous position setpoint
 * @param pos_sp_curr current position setpoint
 * @param current_lat current vehicle latitude [deg]
 * @param current_lon current vehicle longitude [deg]
 * @param acc_rad acceptance radius around the current waypoint [m]
 * @param min_current_sp_distance_xy in/out smallest horizontal distance to the current waypoint achieved so far [m]
 * @return altitude setpoint [m AMSL]
 */
float calculateFirstOrderHoldAltitude(const bool prev_valid, const position_setpoint_s &pos_sp_prev,
				      const position_setpoint_s &pos_sp_curr, const double current_lat, const double current_lon,
				      const float acc_rad, float &min_current_sp_distance_xy);
