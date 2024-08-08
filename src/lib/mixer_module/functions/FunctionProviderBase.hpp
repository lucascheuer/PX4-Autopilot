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

#include <limits.h>

#include <mixer_module/output_functions.hpp>

#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>

#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

class FunctionProviderBase
{
public:
	struct Context {
		Context(px4::WorkItem &_work_item, const float &_thrust_factor)
		: work_item(_work_item), thrust_factor(_thrust_factor), xy_thrust_factor(default_factor)
		{}

		Context(px4::WorkItem &_work_item, const float &_thrust_factor, const float &_xy_thrust_factor)
		: work_item(_work_item), thrust_factor(_thrust_factor), xy_thrust_factor(_xy_thrust_factor)
		{}

		const float default_factor = 0;
		px4::WorkItem &work_item;
		const float &thrust_factor;
		const float &xy_thrust_factor;
	};

	FunctionProviderBase() = default;
	virtual ~FunctionProviderBase() = default;

	virtual void update() = 0;

	/**
	 * Get the current output value for a given function
	 * @return NAN (=disarmed) or value in range [-1, 1]
	 */
	virtual float value(OutputFunction func) = 0;

	virtual float defaultFailsafeValue(OutputFunction func) const { return NAN; }
	virtual bool allowPrearmControl() const { return true; }

	virtual uORB::SubscriptionCallbackWorkItem *subscriptionCallback() { return nullptr; }

	virtual bool getLatestSampleTimestamp(hrt_abstime &t) const { return false; }

	/**
	 * Check whether the output (motor) is configured to be reversible
	 */
	virtual bool reversible(OutputFunction func) const { return false; }
};
