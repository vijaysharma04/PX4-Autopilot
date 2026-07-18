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

#pragma once

#include <drivers/drv_hrt.h>
#include <px4_platform_common/module_params.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/nvx_flight_profile_status.h>
#include <uORB/topics/vehicle_status.h>

enum class NvxFlightProfile : uint8_t {
	Cine = 0,
	Normal = 1,
	Sport = 2
};

class NvxFlightProfileManager : public ModuleParams
{
public:
	explicit NvxFlightProfileManager(ModuleParams *parent);

	void update(float dt, const vehicle_status_s &vehicle_status);

private:
	struct Limits {
		float horizontal_speed;
		float horizontal_acceleration;
		float horizontal_jerk;
		float yaw_rate_deg_s;
		float yaw_response_time_s;
		float climb_speed;
		float descent_speed;
	};

	static constexpr uint8_t kProfileChannelIndex{4};
	static constexpr hrt_abstime kDebounceTime{200000};
	static constexpr hrt_abstime kRcFreshTime{500000};
	static constexpr hrt_abstime kTransitionTime{750000};
	static constexpr hrt_abstime kPublishInterval{100000};

	static NvxFlightProfile sanitizeProfile(int32_t value);
	static const char *profileName(NvxFlightProfile profile);
	NvxFlightProfile profileForSwitch(uint16_t value) const;
	Limits limitsForProfile(NvxFlightProfile profile) const;
	void processRcInput(hrt_abstime now);
	void processAppRequest(const vehicle_status_s &vehicle_status, hrt_abstime now);
	void confirmProfile(NvxFlightProfile profile, bool source_rc, hrt_abstime now,
			    const vehicle_status_s &vehicle_status);
	void updateEffectiveLimits(hrt_abstime now);
	void publishStatus(hrt_abstime now, bool force = false);
	void logTransition(const char *phase, const vehicle_status_s &vehicle_status) const;
	bool limitsEqual(const Limits &a, const Limits &b) const;

	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Publication<nvx_flight_profile_status_s> _status_pub{ORB_ID(nvx_flight_profile_status)};

	input_rc_s _input_rc{};
	NvxFlightProfile _requested_profile{NvxFlightProfile::Normal};
	NvxFlightProfile _candidate_profile{NvxFlightProfile::Normal};
	NvxFlightProfile _active_profile{NvxFlightProfile::Normal};
	NvxFlightProfile _last_valid_profile{NvxFlightProfile::Normal};
	uint16_t _switch_value{0};
	bool _switch_valid{false};
	bool _switch_was_valid{false};
	bool _source_rc{false};
	hrt_abstime _candidate_since{0};
	hrt_abstime _profile_change_timestamp{0};
	hrt_abstime _transition_start{0};
	hrt_abstime _last_publish{0};
	Limits _transition_from{};
	Limits _effective_limits{};
	Limits _target_limits{};
	bool _limits_initialized{false};
	bool _transition_active{false};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::NVX_FLT_PROFILE>) _param_nvx_flt_profile,
		(ParamFloat<px4::params::MPC_VEL_MANUAL>) _param_mpc_vel_manual,
		(ParamFloat<px4::params::MPC_ACC_HOR>) _param_mpc_acc_hor,
		(ParamFloat<px4::params::MPC_JERK_MAX>) _param_mpc_jerk_max,
		(ParamFloat<px4::params::MPC_MAN_Y_MAX>) _param_mpc_man_y_max,
		(ParamFloat<px4::params::MPC_MAN_Y_TAU>) _param_mpc_man_y_tau,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_UP>) _param_mpc_z_vel_max_up,
		(ParamFloat<px4::params::MPC_Z_VEL_MAX_DN>) _param_mpc_z_vel_max_dn
	)
};
