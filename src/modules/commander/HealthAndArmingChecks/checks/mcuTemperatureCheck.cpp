/****************************************************************************
 *
 *   Copyright (c) 2026 NVX Development Team. All rights reserved.
 *
 ****************************************************************************/

#include "mcuTemperatureCheck.hpp"

#include <px4_platform_common/defines.h>

#include <cmath>

using namespace time_literals;

namespace
{
constexpr float kWarningTemperatureC = 60.f;
constexpr float kClearTemperatureC = 55.f;
constexpr float kFilterAlpha = 0.25f;

#if defined(CONFIG_ARCH_CHIP_STM32F7)
constexpr int16_t kTemperatureAdcChannel = 18;
constexpr uintptr_t kTemperatureCal30Address = 0x1FF0F44C;
constexpr uintptr_t kTemperatureCal110Address = 0x1FF0F44E;
constexpr float kCalibrationVoltage = 3.3f;
constexpr float kCalibrationLowC = 30.f;
constexpr float kCalibrationHighC = 110.f;
#endif
}

McuTemperatureChecks::McuTemperatureChecks()
{
	_overheat_hysteresis.set_hysteresis_time_from(false, 2_s);
	_overheat_hysteresis.set_hysteresis_time_from(true, 0_s);
}

bool McuTemperatureChecks::readTemperature(float &temperature_c)
{
#if defined(CONFIG_ARCH_CHIP_STM32F7)
	adc_report_s adc{};

	if (!_adc_report_sub.copy(&adc) || hrt_elapsed_time(&adc.timestamp) > 2_s || adc.resolution == 0) {
		return false;
	}

	int32_t raw_temperature = -1;

	for (unsigned i = 0; i < sizeof(adc.channel_id) / sizeof(adc.channel_id[0]); ++i) {
		if (adc.channel_id[i] == kTemperatureAdcChannel) {
			raw_temperature = adc.raw_data[i];
			break;
		}
	}

	if (raw_temperature <= 0 || raw_temperature >= static_cast<int32_t>(adc.resolution)) {
		return false;
	}

	const uint16_t calibration_30 = *reinterpret_cast<const volatile uint16_t *>(kTemperatureCal30Address);
	const uint16_t calibration_110 = *reinterpret_cast<const volatile uint16_t *>(kTemperatureCal110Address);

	if (calibration_30 == 0 || calibration_30 == UINT16_MAX || calibration_110 == 0
	    || calibration_110 == UINT16_MAX || calibration_30 == calibration_110) {
		return false;
	}

	const float reference_voltage = PX4_ISFINITE(adc.v_ref) && adc.v_ref > 0.f ? adc.v_ref : kCalibrationVoltage;
	const float raw_at_calibration_voltage = raw_temperature * reference_voltage / kCalibrationVoltage;
	temperature_c = kCalibrationLowC
			+ (raw_at_calibration_voltage - calibration_30)
			* (kCalibrationHighC - kCalibrationLowC)
			/ static_cast<float>(calibration_110 - calibration_30);

	return PX4_ISFINITE(temperature_c) && temperature_c > -40.f && temperature_c < 150.f;
#else
	(void)temperature_c;
	return false;
#endif
}

void McuTemperatureChecks::checkAndReport(const Context &context, Report &reporter)
{
	float temperature_c = NAN;

	if (!readTemperature(temperature_c)) {
		_filtered_temperature_c = NAN;
		_overheat_hysteresis.set_state_and_update(false, hrt_absolute_time());
		return;
	}

	if (!PX4_ISFINITE(_filtered_temperature_c)) {
		_filtered_temperature_c = temperature_c;

	} else {
		_filtered_temperature_c += kFilterAlpha * (temperature_c - _filtered_temperature_c);
	}

	const bool threshold_exceeded = _overheat_hysteresis.get_state()
					? _filtered_temperature_c > kClearTemperatureC
					: _filtered_temperature_c >= kWarningTemperatureC;
	_overheat_hysteresis.set_state_and_update(threshold_exceeded, hrt_absolute_time());

	if (_overheat_hysteresis.get_state()) {
		/* EVENT
		 */
		reporter.armingCheckFailure<float>(NavModes::All, health_component_t::system, events::ID("check_mcu_temperature_high"),
				events::Log::Error,
				"Aircraft temperature too high: {1:.1} C. Please turn off the drone",
				_filtered_temperature_c);

		if (reporter.mavlink_log_pub()) {
			if (context.isArmed()) {
				mavlink_log_critical(reporter.mavlink_log_pub(),
						     "Aircraft temperature too high. Land, then turn off the drone.");

			} else {
				mavlink_log_critical(reporter.mavlink_log_pub(),
						     "Aircraft temperature too high. Please turn off the drone.");
			}
		}
	}
}
