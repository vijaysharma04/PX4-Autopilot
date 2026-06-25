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

#include "FirstOrderHoldAltitude.hpp"

#include <math.h>

#include <lib/geo/geo.h>
#include <lib/mathlib/mathlib.h>

float calculateFirstOrderHoldAltitude(const bool prev_valid, const position_setpoint_s &pos_sp_prev,
				      const position_setpoint_s &pos_sp_curr, const double current_lat, const double current_lon,
				      const float acc_rad, float &min_current_sp_distance_xy)
{
	float position_sp_alt = pos_sp_curr.alt;

	if (prev_valid &&
	    ((pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_POSITION) ||
	     (pos_sp_prev.type == position_setpoint_s::SETPOINT_TYPE_LOITER))
	   ) {
		const float d_curr_prev = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, pos_sp_prev.lat,
					  pos_sp_prev.lon);

		// Do not try to find a solution if the last waypoint is inside the acceptance radius of the current one
		if (d_curr_prev > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
			// Calculate distance to current waypoint
			const float d_curr = get_distance_to_next_waypoint(pos_sp_curr.lat, pos_sp_curr.lon, current_lat,
					     current_lon);

			// Save distance to waypoint if it is the smallest ever achieved, however make sure that
			// min_current_sp_distance_xy is never larger than the distance between the current and the previous wp
			min_current_sp_distance_xy = math::min(d_curr, min_current_sp_distance_xy, d_curr_prev);

			// if the minimal distance is smaller than the acceptance radius, we should be at waypoint alt
			// navigator will soon switch to the next waypoint item (if there is one) as soon as we reach this altitude
			if (min_current_sp_distance_xy > math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius))) {
				// The setpoint is set linearly and such that the system reaches the current altitude at the acceptance
				// radius around the current waypoint
				const float delta_alt = (pos_sp_curr.alt - pos_sp_prev.alt);
				const float grad = -delta_alt / (d_curr_prev - math::max(acc_rad, fabsf(pos_sp_curr.loiter_radius)));
				const float a = pos_sp_prev.alt - grad * d_curr_prev;

				position_sp_alt = a + grad * min_current_sp_distance_xy;
			}
		}
	}

	return position_sp_alt;
}
