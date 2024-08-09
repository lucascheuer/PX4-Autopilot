/****************************************************************************
 *
 *   Copyright (c) 2021 PX4 Development Team. All rights reserved.
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

#include "ActuatorEffectiveness.hpp"

#include <px4_platform_common/module_params.h>
#include <lib/slew_rate/SlewRate.hpp>


class ActuatorEffectivenessPulsingRotor : public ModuleParams, public ActuatorEffectiveness
{
public:

	static constexpr int NUM_PULSING_ROTOR_MAX = 5;

	struct PulsingGeometry {
		matrix::Vector3f position;
		matrix::Vector3f axis;
		float thrust_coef;
		float xy_thrust_coef;
		float torque_coef;
	};

	struct Geometry {
		PulsingGeometry rotors[NUM_PULSING_ROTOR_MAX];
		int num_rotors{0};
		bool propeller_torque_disabled{true};
	};

	ActuatorEffectivenessPulsingRotor(ModuleParams *parent);
	virtual ~ActuatorEffectivenessPulsingRotor() = default;

	static int computeEffectivenessMatrix(const Geometry &geometry,
					      EffectivenessMatrix &effectiveness, int actuator_start_index = 0);

	bool addActuators(Configuration &configuration);

	const char *name() const override { return "Pulsing Rotors"; }

	int count() const { return _geometry.num_rotors; }

	const PulsingGeometry &config(int idx) const { return _geometry.rotors[idx]; }

private:
	void updateParams() override;
	struct ParamHandles {
		param_t position[3];
		param_t axis[3];
		param_t thrust_coef;
		param_t xy_thrust_coef;
		param_t torque_coef;
	};
	ParamHandles _param_handles[NUM_PULSING_ROTOR_MAX];
	param_t _count_handle;

	Geometry _geometry{};

};
