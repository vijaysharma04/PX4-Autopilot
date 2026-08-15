/****************************************************************************
 *
 *   Copyright (c) 2026 NVX Development Team. All rights reserved.
 *
 ****************************************************************************/

#pragma once

#include "../Common.hpp"

#include <lib/hysteresis/hysteresis.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/adc_report.h>

#include <cmath>

class McuTemperatureChecks : public HealthAndArmingCheckBase
{
public:
	McuTemperatureChecks();
	~McuTemperatureChecks() = default;

	void checkAndReport(const Context &context, Report &reporter) override;

private:
	bool readTemperature(float &temperature_c);

	uORB::Subscription _adc_report_sub{ORB_ID(adc_report)};
	systemlib::Hysteresis _overheat_hysteresis{false};
	float _filtered_temperature_c{NAN};
};
