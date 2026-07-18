/****************************************************************************
 *
 *   Copyright (c) 2026 NVX Fly. All rights reserved.
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
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES ARE DISCLAIMED.
 *
 ****************************************************************************/

#include "NvxFlightProfileManager.hpp"

#include <lib/mathlib/mathlib.h>
#include <px4_platform_common/events.h>
#include <px4_platform_common/log.h>

#include <cinttypes>
#include <cmath>

using namespace time_literals;

namespace
{
constexpr float kCineHorizontalSpeed{3.f};
constexpr float kCineHorizontalAcceleration{1.5f};
constexpr float kCineHorizontalJerk{2.5f};
constexpr float kCineYawRateDegS{60.f};
constexpr float kCineYawResponseTimeS{0.30f};
constexpr float kCineClimbSpeed{1.5f};
constexpr float kCineDescentSpeed{1.f};

float positiveOr(float value, float fallback)
{
	return PX4_ISFINITE(value) && value > FLT_EPSILON ? value : fallback;
}

float interpolate(float from, float to, float alpha)
{
	return from + (to - from) * alpha;
}
}

NvxFlightProfileManager::NvxFlightProfileManager(ModuleParams *parent) :
	ModuleParams(parent)
{
	_active_profile = sanitizeProfile(_param_nvx_flt_profile.get());
	_requested_profile = _active_profile;
	_candidate_profile = _active_profile;
	_last_valid_profile = _active_profile;
}

NvxFlightProfile NvxFlightProfileManager::sanitizeProfile(int32_t value)
{
	switch (value) {
	case static_cast<int32_t>(NvxFlightProfile::Cine):
		return NvxFlightProfile::Cine;

	case static_cast<int32_t>(NvxFlightProfile::Sport):
		return NvxFlightProfile::Sport;

	case static_cast<int32_t>(NvxFlightProfile::Normal):
	default:
		return NvxFlightProfile::Normal;
	}
}

const char *NvxFlightProfileManager::profileName(NvxFlightProfile profile)
{
	switch (profile) {
	case NvxFlightProfile::Cine: return "CINE";
	case NvxFlightProfile::Sport: return "SPORT";
	case NvxFlightProfile::Normal:
	default: return "NORMAL";
	}
}

NvxFlightProfile NvxFlightProfileManager::profileForSwitch(uint16_t value) const
{
	switch (_active_profile) {
	case NvxFlightProfile::Cine:
		if (value >= 1750) { return NvxFlightProfile::Sport; }
		if (value > 1350) { return NvxFlightProfile::Normal; }
		return NvxFlightProfile::Cine;

	case NvxFlightProfile::Sport:
		if (value < 1250) { return NvxFlightProfile::Cine; }
		if (value < 1650) { return NvxFlightProfile::Normal; }
		return NvxFlightProfile::Sport;

	case NvxFlightProfile::Normal:
	default:
		if (value < 1250) { return NvxFlightProfile::Cine; }
		if (value > 1750) { return NvxFlightProfile::Sport; }
		return NvxFlightProfile::Normal;
	}
}

NvxFlightProfileManager::Limits NvxFlightProfileManager::limitsForProfile(NvxFlightProfile profile) const
{
	const Limits normal {
		positiveOr(_param_mpc_vel_manual.get(), kCineHorizontalSpeed),
		positiveOr(_param_mpc_acc_hor.get(), kCineHorizontalAcceleration),
		positiveOr(_param_mpc_jerk_max.get(), kCineHorizontalJerk),
		positiveOr(_param_mpc_man_y_max.get(), kCineYawRateDegS),
		positiveOr(_param_mpc_man_y_tau.get(), kCineYawResponseTimeS),
		positiveOr(_param_mpc_z_vel_max_up.get(), kCineClimbSpeed),
		positiveOr(_param_mpc_z_vel_max_dn.get(), kCineDescentSpeed)
	};

	if (profile != NvxFlightProfile::Cine) {
		// SPORT intentionally equals the approved NORMAL baseline in this first hardware build.
		return normal;
	}

	return Limits {
		math::min(normal.horizontal_speed, kCineHorizontalSpeed),
		math::min(normal.horizontal_acceleration, kCineHorizontalAcceleration),
		math::min(normal.horizontal_jerk, kCineHorizontalJerk),
		math::min(normal.yaw_rate_deg_s, kCineYawRateDegS),
		math::max(normal.yaw_response_time_s, kCineYawResponseTimeS),
		math::min(normal.climb_speed, kCineClimbSpeed),
		math::min(normal.descent_speed, kCineDescentSpeed)
	};
}

void NvxFlightProfileManager::processRcInput(hrt_abstime now)
{
	_input_rc_sub.update(&_input_rc);
	const bool fresh = _input_rc.timestamp_last_signal > 0
			   && now >= _input_rc.timestamp_last_signal
			   && now - _input_rc.timestamp_last_signal <= kRcFreshTime;
	const bool channel_available = _input_rc.channel_count > kProfileChannelIndex;
	const uint16_t value = channel_available ? _input_rc.values[kProfileChannelIndex] : 0;
	const bool valid_value = value >= 800 && value <= 2200;
	_switch_valid = fresh && channel_available && valid_value && !_input_rc.rc_lost && !_input_rc.rc_failsafe;

	if (!_switch_valid) {
		if (_switch_was_valid) {
			PX4_WARN("flight profile switch unavailable; retaining %s", profileName(_last_valid_profile));
			events::send(events::ID("nvx_profile_switch_unavailable"), events::Log::Warning,
				     "Flight profile switch unavailable; retaining active profile");
		}

		_switch_was_valid = false;
		_candidate_since = 0;
		return;
	}

	_switch_was_valid = true;
	_switch_value = value;
	const NvxFlightProfile candidate = profileForSwitch(value);
	_requested_profile = candidate;

	if (candidate != _candidate_profile) {
		_candidate_profile = candidate;
		_candidate_since = now;
		return;
	}

	if (_candidate_since == 0) {
		_candidate_since = now;
	}
}

void NvxFlightProfileManager::processAppRequest(const vehicle_status_s &vehicle_status, hrt_abstime now)
{
	if (_switch_valid || vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED) {
		return;
	}

	const int32_t raw_profile = _param_nvx_flt_profile.get();
	const NvxFlightProfile requested = sanitizeProfile(raw_profile);

	if (raw_profile != static_cast<int32_t>(requested)) {
		PX4_WARN("invalid NVX_FLT_PROFILE %" PRId32 "; using NORMAL", raw_profile);
		_param_nvx_flt_profile.set(static_cast<int32_t>(NvxFlightProfile::Normal));
		_param_nvx_flt_profile.commit();
	}

	_requested_profile = requested;

	if (requested != _active_profile) {
		confirmProfile(requested, false, now, vehicle_status);
	}
}

void NvxFlightProfileManager::confirmProfile(NvxFlightProfile profile, bool source_rc, hrt_abstime now,
		const vehicle_status_s &vehicle_status)
{
	if (profile == _active_profile) {
		_source_rc = source_rc;
		_last_valid_profile = profile;
		return;
	}

	_active_profile = profile;
	_last_valid_profile = profile;
	_source_rc = source_rc;
	_profile_change_timestamp = now;
	_transition_from = _effective_limits;
	_target_limits = limitsForProfile(profile);
	_transition_start = now;
	_transition_active = true;
	_param_nvx_flt_profile.set(static_cast<int32_t>(profile));
	_param_nvx_flt_profile.commit();
	logTransition("start", vehicle_status);
}

void NvxFlightProfileManager::updateEffectiveLimits(hrt_abstime now)
{
	const Limits target = limitsForProfile(_active_profile);

	if (!_limits_initialized) {
		_effective_limits = target;
		_target_limits = target;
		_transition_from = target;
		_limits_initialized = true;
		return;
	}

	if (!limitsEqual(target, _target_limits)) {
		_transition_from = _effective_limits;
		_target_limits = target;
		_transition_start = now;
		_transition_active = true;
	}

	if (!_transition_active) {
		_effective_limits = _target_limits;
		return;
	}

	const float alpha = math::constrain(static_cast<float>(now - _transition_start) / static_cast<float>(kTransitionTime),
			    0.f, 1.f);
	_effective_limits.horizontal_speed = interpolate(_transition_from.horizontal_speed, _target_limits.horizontal_speed, alpha);
	_effective_limits.horizontal_acceleration = interpolate(_transition_from.horizontal_acceleration,
			_target_limits.horizontal_acceleration, alpha);
	_effective_limits.horizontal_jerk = interpolate(_transition_from.horizontal_jerk, _target_limits.horizontal_jerk, alpha);
	_effective_limits.yaw_rate_deg_s = interpolate(_transition_from.yaw_rate_deg_s, _target_limits.yaw_rate_deg_s, alpha);
	_effective_limits.yaw_response_time_s = interpolate(_transition_from.yaw_response_time_s,
			_target_limits.yaw_response_time_s, alpha);
	_effective_limits.climb_speed = interpolate(_transition_from.climb_speed, _target_limits.climb_speed, alpha);
	_effective_limits.descent_speed = interpolate(_transition_from.descent_speed, _target_limits.descent_speed, alpha);

	if (alpha >= 1.f) {
		_transition_active = false;
	}
}

void NvxFlightProfileManager::publishStatus(hrt_abstime now, bool force)
{
	if (!force && now - _last_publish < kPublishInterval) {
		return;
	}

	nvx_flight_profile_status_s status{};
	status.timestamp = now;
	status.profile_change_timestamp = _profile_change_timestamp;
	status.switch_value = _switch_value;
	status.requested_profile = static_cast<uint8_t>(_requested_profile);
	status.active_profile = static_cast<uint8_t>(_active_profile);
	status.switch_valid = _switch_valid;
	status.source_rc = _source_rc;
	status.transition_active = _transition_active;
	status.horizontal_speed = _effective_limits.horizontal_speed;
	status.horizontal_acceleration = _effective_limits.horizontal_acceleration;
	status.horizontal_jerk = _effective_limits.horizontal_jerk;
	status.yaw_rate_deg_s = _effective_limits.yaw_rate_deg_s;
	status.yaw_response_time_s = _effective_limits.yaw_response_time_s;
	status.climb_speed = _effective_limits.climb_speed;
	status.descent_speed = _effective_limits.descent_speed;
	_status_pub.publish(status);
	_last_publish = now;
}

void NvxFlightProfileManager::logTransition(const char *phase, const vehicle_status_s &vehicle_status) const
{
	PX4_INFO("profile %s t=%" PRIu64 " ch5=%u req=%s active=%s nav=%u armed=%u src=%s "
		 "vxy=%.2f acc=%.2f jerk=%.2f yaw=%.1f tau=%.2f up=%.2f dn=%.2f",
		 phase, _profile_change_timestamp, _switch_value, profileName(_requested_profile), profileName(_active_profile),
		 vehicle_status.nav_state, vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED,
		 _source_rc ? "rc" : "app", (double)_target_limits.horizontal_speed,
		 (double)_target_limits.horizontal_acceleration, (double)_target_limits.horizontal_jerk,
		 (double)_target_limits.yaw_rate_deg_s, (double)_target_limits.yaw_response_time_s,
		 (double)_target_limits.climb_speed, (double)_target_limits.descent_speed);
}

bool NvxFlightProfileManager::limitsEqual(const Limits &a, const Limits &b) const
{
	return fabsf(a.horizontal_speed - b.horizontal_speed) < 0.001f
	       && fabsf(a.horizontal_acceleration - b.horizontal_acceleration) < 0.001f
	       && fabsf(a.horizontal_jerk - b.horizontal_jerk) < 0.001f
	       && fabsf(a.yaw_rate_deg_s - b.yaw_rate_deg_s) < 0.001f
	       && fabsf(a.yaw_response_time_s - b.yaw_response_time_s) < 0.001f
	       && fabsf(a.climb_speed - b.climb_speed) < 0.001f
	       && fabsf(a.descent_speed - b.descent_speed) < 0.001f;
}

void NvxFlightProfileManager::update(float dt, const vehicle_status_s &vehicle_status)
{
	(void)dt;
	const hrt_abstime now = hrt_absolute_time();

	if (!_limits_initialized) {
		updateEffectiveLimits(now);
	}

	processRcInput(now);
	if (_switch_valid && _param_nvx_flt_profile.get() != static_cast<int32_t>(_active_profile)) {
		PX4_WARN("rejecting app profile while RC switch is active");
		_param_nvx_flt_profile.set(static_cast<int32_t>(_active_profile));
		_param_nvx_flt_profile.commit();
	}

	if (_switch_valid && _candidate_since > 0 && now - _candidate_since >= kDebounceTime) {
		if (_candidate_profile != _active_profile) {
			confirmProfile(_candidate_profile, true, now, vehicle_status);

		} else {
			_source_rc = true;
			_last_valid_profile = _active_profile;
		}

		_candidate_since = 0;

	} else {
		processAppRequest(vehicle_status, now);
	}

	const bool transition_was_active = _transition_active;
	updateEffectiveLimits(now);

	if (transition_was_active && !_transition_active) {
		logTransition("complete", vehicle_status);
	}

	publishStatus(now, transition_was_active != _transition_active);
}
