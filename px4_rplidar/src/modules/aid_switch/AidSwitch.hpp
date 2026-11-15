/**
 * @file AidSwitch.hpp
 *
 * Selects the EKF2 horizontal position source (GPS, GPS + optical flow, optical flow only)
 * from a 3-position RC switch, by setting EKF2_GPS_CTRL and EKF2_OF_CTRL.
 *
 * It only acts when the switch moves, so a source chosen from the ground station
 * (by setting the same two parameters) stays until the switch is moved again.
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <systemlib/mavlink_log.h>

#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/vehicle_optical_flow.h>

using namespace time_literals;

class AidSwitch : public ModuleBase<AidSwitch>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AidSwitch();
	~AidSwitch() override = default;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

	enum class Source : int {
		Unknown = -1,
		Gps = 0,
		GpsAndFlow = 1,
		Flow = 2,
	};

	static const char *source_name(Source s);

private:
	void Run() override;

	Source read_switch();          // switch position, Unknown if no valid RC
	bool flow_valid();             // recent optical flow with quality, within its height range
	Source current_source() const; // from the EKF2 parameters
	bool apply(Source s);          // set the EKF2 parameters, true if they changed

	uORB::Subscription _rc_channels_sub{ORB_ID(rc_channels)};
	uORB::Subscription _optical_flow_sub{ORB_ID(vehicle_optical_flow)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	orb_advert_t _mavlink_log_pub{nullptr};

	param_t _param_gps_ctrl{PARAM_INVALID};
	param_t _param_of_ctrl{PARAM_INVALID};

	vehicle_optical_flow_s _flow{};

	Source _switch_pos{Source::Unknown};   // last seen switch position
	Source _requested{Source::Unknown};    // what the switch asked for (may be refused)
	bool _flow_refused_reported{false};
	hrt_abstime _flow_bad_since{0};
	float _rc_value{NAN};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ASW_RC_CHAN>) _param_asw_rc_chan,
		(ParamInt<px4::params::ASW_GPS_CTRL>) _param_asw_gps_ctrl
	)
};
