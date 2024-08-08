/****************************************************************************
 *
 *   Copyright (c) 2021-2022 PX4 Development Team. All rights reserved.
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

#include "FunctionProviderBase.hpp"

#include <px4_platform_common/module.h>
#include <uORB/topics/actuator_tilt_motors.h>

/**
 * Functions: Motor1 ... MotorMax
 */
class FunctionTiltMotors : public FunctionProviderBase
{
public:
	static_assert(actuator_tilt_motors_s::NUM_CONTROLS == (int)OutputFunction::Tilt_Motor_ThrottleMax - (int)OutputFunction::Tilt_Motor_Throttle1 + 1,
		      "Unexpected num motors");

	static_assert(actuator_tilt_motors_s::ACTUATOR_FUNCTION_MOTOR1 == (int)OutputFunction::Tilt_Motor_Throttle1, "Unexpected motor idx");

	FunctionTiltMotors(const Context &context) :
		_topic(&context.work_item, ORB_ID(actuator_tilt_motors)),
		_thrust_factor(context.thrust_factor),
		_xy_thrust_factor(context.xy_thrust_factor)
	{
		for (int i = 0; i < actuator_tilt_motors_s::NUM_CONTROLS * 3; ++i) {
			_data.control[i] = NAN;
		}
	}

	static FunctionProviderBase *allocate(const Context &context) { return new FunctionTiltMotors(context); }

	void update() override
	{
		if (_topic.update(&_data)) {
			updateValues(_data.reversible_flags, _thrust_factor, _xy_thrust_factor, _data.control, actuator_tilt_motors_s::NUM_CONTROLS);
		}
	}

	float value(OutputFunction func) override { return _data.control[(int)func - (int)OutputFunction::Tilt_Motor_Throttle1]; }

	bool allowPrearmControl() const override { return false; }

	uORB::SubscriptionCallbackWorkItem *subscriptionCallback() override { return &_topic; }

	bool getLatestSampleTimestamp(hrt_abstime &t) const override { t = _data.timestamp_sample; return t != 0; }

	static inline void updateValues(uint32_t reversible, float thrust_factor, float xy_angle_factor, float *values, int num_values)
	{
		if (thrust_factor > 0.f && thrust_factor <= 1.f) {
			// angle factor
			// rpm (which is based on motor's map) 0-1
			// -1 - 1 x/y thrust, rpm, angle factor -> pulsing amount
			// thrust / cos(angle input) = final thrust
			// a * rpm^2 = final thrust
			// des_rpm = sqrt(final thrust / a)
			// scale this based on param for motor max RPM control
			// for a given angular velocity, a fixed percentage of excitation will result in a fixed angle
			//
			for (int i = 0; i < num_values; ++i) { // going through 5, so set throttle, x, and y
				float relative_z_thrust = values[i];
				float relative_x_thrust = values[i + num_values];
				float relative_y_thrust = values[i + 2 * num_values];

				float relative_thrust_magnitude = sqrtf(relative_x_thrust * relative_x_thrust + relative_y_thrust * relative_y_thrust + relative_z_thrust * relative_z_thrust);
				float x_angle = atan2f(relative_x_thrust, relative_z_thrust);
				float y_angle = atan2f(relative_y_thrust, relative_z_thrust);
				values[i] = thrust_factor * sqrtf(relative_thrust_magnitude);
				values[i + num_values] = x_angle * xy_angle_factor * values[i]; // pi/6rad * 0.248 * 200rad/s = 50 rad/s
				values[i + 2 * num_values] = y_angle * xy_angle_factor * values[i];
			}
		}
	}

	bool reversible(OutputFunction func) const override { return _data.reversible_flags & (1u << ((int)func - (int)OutputFunction::Motor1)); }

private:
	uORB::SubscriptionCallbackWorkItem _topic;
	// DEFINE_PARAMETERS(
	// 	(ParamInt<px4::params::TILT_THRUST_FACT>) _param_tilt_thrust_factor,
	// 	(ParamInt<px4::params::THRUST_FACT>) _param_thrust_factor
	// )
	actuator_tilt_motors_s _data{};
	const float &_thrust_factor;
	const float &_xy_thrust_factor;
};
