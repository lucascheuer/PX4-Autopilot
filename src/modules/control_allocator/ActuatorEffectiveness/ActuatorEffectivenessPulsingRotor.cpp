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

#include <px4_platform_common/log.h>

#include "ActuatorEffectivenessPulsingRotor.hpp"

using namespace matrix;

ActuatorEffectivenessPulsingRotor::ActuatorEffectivenessPulsingRotor(ModuleParams *parent)
	: ModuleParams(parent)
{
	for (int i = 0; i < NUM_FT_MAX; ++i) {
		// param_t position[3];
		// param_t axis[3];
		// param_t thrust_coef;
		// param_t torque_coef;
		char buffer[17];

		snprintf(buffer, sizeof(buffer), "CA_PR%u_PX", i);
		_param_handles[i].position[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_PY", i);
		_param_handles[i].position[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_PZ", i);
		_param_handles[i].position[2] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_PR%u_AX", i);
		_param_handles[i].axis[0] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_AY", i);
		_param_handles[i].axis[1] = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_AZ", i);
		_param_handles[i].axis[2] = param_find(buffer);

		snprintf(buffer, sizeof(buffer), "CA_PR%u_CT", i);
		_param_handles[i].thrust_coef = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_XYT", i);
		_param_handles[i].xy_thrust_coef = param_find(buffer);
		snprintf(buffer, sizeof(buffer), "CA_PR%u_CM", i);
		_param_handles[i].torque_coef = param_find(buffer);

	}

	_count_handle = param_find("CA_PR_COUNT");
	updateParams();
}

void ActuatorEffectivenessPulsingRotor::updateParams()
{
	ModuleParams::updateParams();

	int32_t count = 0;

	if (param_get(_count_handle, &count) != 0) {
		PX4_ERR("param_get failed");
		return;
	}

	_geometry.num_rotors = count;

	for (int i = 0; i < _geometry.num_rotors; i++) {
		Vector3f &position = _geometry.rotors[i].position;
		Vector3f &axis = _geometry.rotors[i].axis;

		for (int n = 0; n < 3; ++n) {
			param_get(_param_handles[i].position[n], &position(n));
			param_get(_param_handles[i].axis[n], &axis(n));
		}

		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].thrust_coef, &_geometry.rotors[i].thrust_coef);
		param_get(_param_handles[i].torque_coef, &_geometry.rotors[i].torque_coef);
	}
}

bool ActuatorEffectivenessPulsingRotor::addActuators(Configuration &configuration)
{
	int num_actuators = computeEffectivenessMatrix(_geometry,
			    configuration.effectiveness_matrices[configuration.selected_matrix],
			    configuration.num_actuators_matrix[configuration.selected_matrix]);
	configuration.actuatorsAdded(ActuatorType::PULSING, num_actuators);

	return true;
}

int ActuatorEffectivenessPulsingRotor::computeEffectivenessMatrix(const Geometry &geometry,
		EffectivenessMatrix &effectiveness, int actuator_start_index)
{
	int num_actuators = 0;

	for (int i = 0; i < geometry.num_rotors; i++) {

		if (i + actuator_start_index >= NUM_ACTUATORS) {
			break;
		}

		++num_actuators;

		// Get rotor axis
		Vector3f f_axis = geometry.rotors[i].axis;

		// Normalize axis
		float f_axis_norm = f_axis.norm();

		if (f_axis_norm > FLT_EPSILON) {
			f_axis /= f_axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		Vector3f t_axis = geometry.rotors[i].torque_axis;

		float t_axis_norm = t_axis.norm();

		if (t_axis_norm > FLT_EPSILON) {
			t_axis /= t_axis_norm;

		} else {
			// Bad axis definition, ignore this rotor
			continue;
		}

		// Get rotor position
		const Vector3f &position = geometry.rotors[i].position;

		// Get coefficients
		float ct = geometry.rotors[i].thrust_coef;
		float cm = geometry.rotors[i].torque_coef;

		if (fabsf(ct) < FLT_EPSILON) {
			continue;
		}

		// Compute thrust generated by this rotor
		const::Vector3f thrust = ct * t_axis;

		// Compute total moment generated by this rotor
		matrix::Vector3f moment = ct * position.cross(f_axis) + ct * cm * t_axis;


		// Fill corresponding items in effectiveness matrix
		for (size_t j = 0; j < 3; j++) {
			effectiveness(j, i + actuator_start_index) = moment(j);
			effectiveness(j + 3, i + actuator_start_index) = thrust(j);
		}
	}

	return num_actuators;
}
