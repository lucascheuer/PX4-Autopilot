#ifndef VERTIQ_CLLIENT_MANAGER_HPP
#define VERTIQ_CLLIENT_MANAGER_HPP

#include <drivers/device/device.h>
#include <lib/led/led.h>
#include <lib/mixer_module/mixer_module.hpp>
#include <lib/perf/perf_counter.h>

#include <parameters/param.h>

#include <px4_log.h>
#include <px4_platform_common/module.h>

#include "vertiq_serial_interface.hpp"
#include "ifci.hpp"

#include "iq-module-communication-cpp/inc/propeller_motor_control_client.hpp"
#include "iq-module-communication-cpp/inc/brushless_drive_client.hpp"
#include "iq-module-communication-cpp/inc/arming_handler_client.hpp"

#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
#include "iq-module-communication-cpp/inc/system_control_client.hpp"
#endif

class VertiqClientManager{
	public:
	VertiqClientManager(VertiqSerialInterface * serial_interface);

	void Init(uint8_t object_id);

	/**
	* @brief Handle the IQUART interface. Make sure that we update TX and RX buffers
	*/
	void HandleClientCommunication();

	IFCI * GetMotorInterface();

	void SendSetForceArm();
	void SendSetForceDisarm();
	void SendSetCoast();
	void SendSetVelocitySetpoint(uint16_t velocity_setpoint);

	uint8_t GetNumberOfClients();

	#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
	void UpdateSystemControlEntries();
	void WaitForSystemControlResponses(hrt_abstime timeout = 100_ms);
	#endif

	private:
	//We need a serial handler in order to talk over the serial port
	VertiqSerialInterface * _serial_interface;

	//IQUART Client configuration
	IFCI _motor_interface;

	//Vertiq client information
	//Some constants to help us out
	static const uint8_t _kBroadcastID = 63;
	static const uint8_t MINIMUM_NUM_CLIENTS = 2;
	static const uint8_t MAXIMUM_NUM_CLIENTS = 15;

	//Array information
	ClientAbstract * _client_array[MAXIMUM_NUM_CLIENTS];
	uint8_t _clients_in_use = MINIMUM_NUM_CLIENTS;

	//Clients
	PropellerMotorControlClient _broadcast_prop_motor_control;
	ArmingHandlerClient _broadcast_arming_handler;

	#ifdef CONFIG_USE_SYSTEM_CONTROL_CLIENT
	SystemControlClient * _system_control;
	#endif
};

#endif
