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

#include <gtest/gtest.h>
#include <float.h>

#include <lib/geo/geo.h>

#include "FirstOrderHoldAltitude.hpp"

// Two waypoints on the equator, ~1113 m apart, with a 100 m altitude difference.
static constexpr double kPrevLat = 0.0;
static constexpr double kPrevLon = 0.0;
static constexpr double kCurrLat = 0.0;
static constexpr double kCurrLon = 0.01;

static constexpr float kPrevAlt = 100.0f;
static constexpr float kCurrAlt = 200.0f;

static position_setpoint_s makePrev(uint8_t type = position_setpoint_s::SETPOINT_TYPE_POSITION)
{
	position_setpoint_s sp{};
	sp.type = type;
	sp.lat = kPrevLat;
	sp.lon = kPrevLon;
	sp.alt = kPrevAlt;
	sp.loiter_radius = 0.0f;
	return sp;
}

static position_setpoint_s makeCurr(uint8_t type = position_setpoint_s::SETPOINT_TYPE_POSITION,
				    float loiter_radius = 0.0f)
{
	position_setpoint_s sp{};
	sp.type = type;
	sp.lat = kCurrLat;
	sp.lon = kCurrLon;
	sp.alt = kCurrAlt;
	sp.loiter_radius = loiter_radius;
	return sp;
}

TEST(FirstOrderHoldAltitudeTest, PreviousSetpointInvalidReturnsCurrentAltitude)
{
	// With no valid previous setpoint there is nothing to interpolate from: command the current altitude directly.
	float min_dist = FLT_MAX;
	const float alt = calculateFirstOrderHoldAltitude(false, makePrev(), makeCurr(), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}

TEST(FirstOrderHoldAltitudeTest, UnsupportedPreviousTypeReturnsCurrentAltitude)
{
	// FOH only ramps from POSITION/LOITER previous waypoints (e.g. not from a TAKEOFF waypoint).
	float min_dist = FLT_MAX;
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(position_setpoint_s::SETPOINT_TYPE_TAKEOFF),
			  makeCurr(), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}

TEST(FirstOrderHoldAltitudeTest, AtPreviousWaypointReturnsPreviousAltitude)
{
	// Vehicle sits at the previous waypoint: the ramp has not started, command the previous altitude.
	float min_dist = FLT_MAX;
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(), makeCurr(), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_NEAR(alt, kPrevAlt, 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, MidpointReturnsInterpolatedAltitude)
{
	// Force the smallest-achieved distance to half the leg length: altitude should be the linear midpoint.
	const float d_curr_prev = get_distance_to_next_waypoint(kCurrLat, kCurrLon, kPrevLat, kPrevLon);

	float min_dist = d_curr_prev / 2.0f;
	// Vehicle far from the current waypoint so d_curr does not clamp min_dist below the value we set.
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(), makeCurr(), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_NEAR(alt, 0.5f * (kPrevAlt + kCurrAlt), 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, ReachesCurrentAltitudeAtAcceptanceRadius)
{
	// Within the acceptance radius around the current waypoint the full current altitude is commanded.
	const float acc_rad = 50.0f;

	float min_dist = acc_rad; // not strictly greater than acc_rad -> ramp complete
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(), makeCurr(), kPrevLat, kPrevLon, acc_rad, min_dist);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}

TEST(FirstOrderHoldAltitudeTest, MinDistanceOnlyDecreases)
{
	// Vehicle is at the previous waypoint (far from current), but a smaller distance was achieved before.
	const float previously_achieved = 100.0f;
	float min_dist = previously_achieved;

	calculateFirstOrderHoldAltitude(true, makePrev(), makeCurr(), kPrevLat, kPrevLon, 0.0f, min_dist);

	// The tracked minimum must not grow even though the vehicle moved away from the waypoint.
	EXPECT_FLOAT_EQ(min_dist, previously_achieved);
}

TEST(FirstOrderHoldAltitudeTest, LoiterPreviousTypeIsSupported)
{
	// A loiter waypoint is a valid ramp origin (this is the reposition-in-hold case).
	float min_dist = FLT_MAX;
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(position_setpoint_s::SETPOINT_TYPE_LOITER),
			  makeCurr(position_setpoint_s::SETPOINT_TYPE_LOITER), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_NEAR(alt, kPrevAlt, 1e-2f);
}

TEST(FirstOrderHoldAltitudeTest, LoiterRadiusActsAsAcceptanceRadius)
{
	// Inside the loiter circle of the current waypoint the full current altitude is commanded, even when
	// the geometric acceptance radius is zero.
	const float loiter_radius = 80.0f;

	float min_dist = loiter_radius - 10.0f; // inside the loiter circle
	const float alt = calculateFirstOrderHoldAltitude(true, makePrev(),
			  makeCurr(position_setpoint_s::SETPOINT_TYPE_LOITER, loiter_radius), kPrevLat, kPrevLon, 0.0f, min_dist);
	EXPECT_FLOAT_EQ(alt, kCurrAlt);
}
