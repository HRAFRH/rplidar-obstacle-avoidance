/**
 * @file AidSwitch.cpp
 *
 * See AidSwitch.hpp.
 */

#include "AidSwitch.hpp"

#include <math.h>
#include <string.h>

// EKF2_GPS_CTRL and EKF2_OF_CTRL are bitmask/enum parameters; 0 disables the source.
static constexpr int32_t OF_CTRL_ON = 1;

// switch thresholds on the -1..1 channel value, with a dead band between positions
static constexpr float SWITCH_LOW = -0.5f;
static constexpr float SWITCH_HIGH = 0.5f;
static constexpr float SWITCH_MID = 0.25f;

static constexpr hrt_abstime RC_TIMEOUT = 500_ms;
static constexpr hrt_abstime FLOW_TIMEOUT = 500_ms;
static constexpr hrt_abstime FLOW_LOST_DELAY = 1_s;

AidSwitch::AidSwitch() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

bool AidSwitch::init()
{
	_param_gps_ctrl = param_find("EKF2_GPS_CTRL");
	_param_of_ctrl = param_find("EKF2_OF_CTRL");

	if (_param_gps_ctrl == PARAM_INVALID || _param_of_ctrl == PARAM_INVALID) {
		PX4_ERR("EKF2_GPS_CTRL or EKF2_OF_CTRL not found");
		return false;
	}

	ScheduleOnInterval(100_ms);
	return true;
}

const char *AidSwitch::source_name(Source s)
{
	switch (s) {
	case Source::Gps: return "GPS";

	case Source::GpsAndFlow: return "GPS + optical flow";

	case Source::Flow: return "optical flow only";

	default: return "unknown";
	}
}

AidSwitch::Source AidSwitch::read_switch()
{
	const int chan = _param_asw_rc_chan.get();

	if (chan < 1 || chan > 18) {
		_rc_value = NAN;
		return Source::Unknown;
	}

	rc_channels_s rc{};

	if (!_rc_channels_sub.copy(&rc) || rc.signal_lost || rc.channel_count < chan
	    || hrt_elapsed_time(&rc.timestamp) > RC_TIMEOUT) {
		_rc_value = NAN;
		return Source::Unknown;
	}

	const float v = rc.channels[chan - 1];
	_rc_value = v;

	if (v < SWITCH_LOW) {
		return Source::Gps;
	}

	if (v > SWITCH_HIGH) {
		return Source::Flow;
	}

	if (fabsf(v) < SWITCH_MID) {
		return Source::GpsAndFlow;
	}

	return _switch_pos; // in a dead band: keep the last position
}

bool AidSwitch::flow_valid()
{
	_optical_flow_sub.copy(&_flow);

	if (_flow.timestamp == 0 || hrt_elapsed_time(&_flow.timestamp) > FLOW_TIMEOUT) {
		return false;
	}

	if (_flow.quality == 0) {
		return false;
	}

	// too high for the flow sensor (on the ground below its minimum height is fine)
	if (PX4_ISFINITE(_flow.distance_m) && PX4_ISFINITE(_flow.max_ground_distance)
	    && _flow.max_ground_distance > 0.f && _flow.distance_m > _flow.max_ground_distance) {
		return false;
	}

	return true;
}

AidSwitch::Source AidSwitch::current_source() const
{
	int32_t gps = 0;
	int32_t of = 0;
	param_get(_param_gps_ctrl, &gps);
	param_get(_param_of_ctrl, &of);

	if (gps != 0 && of == 0) { return Source::Gps; }

	if (gps != 0 && of != 0) { return Source::GpsAndFlow; }

	if (gps == 0 && of != 0) { return Source::Flow; }

	return Source::Unknown; // both off
}

bool AidSwitch::apply(Source s)
{
	int32_t gps_want = 0;
	int32_t of_want = 0;

	switch (s) {
	case Source::Gps:
		gps_want = _param_asw_gps_ctrl.get();
		of_want = 0;
		break;

	case Source::GpsAndFlow:
		gps_want = _param_asw_gps_ctrl.get();
		of_want = OF_CTRL_ON;
		break;

	case Source::Flow:
		gps_want = 0;
		of_want = OF_CTRL_ON;
		break;

	default:
		return false;
	}

	int32_t gps = 0;
	int32_t of = 0;
	param_get(_param_gps_ctrl, &gps);
	param_get(_param_of_ctrl, &of);

	bool changed = false;

	if (gps != gps_want) {
		param_set(_param_gps_ctrl, &gps_want);
		changed = true;
	}

	if (of != of_want) {
		param_set(_param_of_ctrl, &of_want);
		changed = true;
	}

	if (changed) {
		mavlink_log_info(&_mavlink_log_pub, "Position source: %s\t", source_name(s));
	}

	return changed;
}

void AidSwitch::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	if (_parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	const bool flow_ok = flow_valid();

	// 1. the switch: act only when it moves (or on the first valid reading)
	const Source pos = read_switch();

	if (pos != Source::Unknown && pos != _switch_pos) {
		_switch_pos = pos;
		_requested = pos;
		_flow_refused_reported = false;
	}

	if (_requested != Source::Unknown) {
		if (_requested == Source::Flow && !flow_ok) {
			if (!_flow_refused_reported) {
				mavlink_log_critical(&_mavlink_log_pub, "Optical flow only refused: no valid flow, keeping %s\t",
						     source_name(current_source()));
				_flow_refused_reported = true;
			}
			// keep the request: it is applied once the flow becomes valid while the switch stays there

		} else {
			apply(_requested);
			_requested = Source::Unknown;
		}
	}

	// 2. safety: flow only (from the switch or the ground station) but the flow is gone -> add GPS back
	if (current_source() == Source::Flow && !flow_ok) {
		if (_flow_bad_since == 0) {
			_flow_bad_since = hrt_absolute_time();

		} else if (hrt_elapsed_time(&_flow_bad_since) > FLOW_LOST_DELAY) {
			mavlink_log_critical(&_mavlink_log_pub, "Optical flow lost, switching to GPS + optical flow\t");
			apply(Source::GpsAndFlow);
			_flow_bad_since = 0;
		}

	} else {
		_flow_bad_since = 0;
	}
}

int AidSwitch::print_status()
{
	PX4_INFO("switch channel: %d (0 = off), value %.2f, position %s",
		 (int)_param_asw_rc_chan.get(), (double)_rc_value, source_name(_switch_pos));
	PX4_INFO("active source: %s", source_name(current_source()));
	PX4_INFO("optical flow: %s (quality %d, distance %.2f m, max %.2f m)",
		 flow_valid() ? "valid" : "not valid", (int)_flow.quality, (double)_flow.distance_m,
		 (double)_flow.max_ground_distance);
	return 0;
}

int AidSwitch::task_spawn(int argc, char *argv[])
{
	AidSwitch *instance = new AidSwitch();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int AidSwitch::custom_command(int argc, char *argv[])
{
	if (argc >= 2 && !strcmp(argv[0], "set") && is_running()) {
		Source s = Source::Unknown;

		if (!strcmp(argv[1], "gps")) { s = Source::Gps; }

		else if (!strcmp(argv[1], "both")) { s = Source::GpsAndFlow; }

		else if (!strcmp(argv[1], "flow")) { s = Source::Flow; }

		if (s == Source::Unknown) {
			return print_usage("set gps|both|flow");
		}

		if (s == Source::Flow && !get_instance()->flow_valid()) {
			PX4_ERR("refused: no valid optical flow");
			return 1;
		}

		get_instance()->apply(s);
		PX4_INFO("active source: %s", source_name(get_instance()->current_source()));
		return 0;
	}

	return print_usage("unknown command");
}

int AidSwitch::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Selects the EKF2 horizontal position source from a 3-position RC switch (ASW_RC_CHAN):
low = GPS, middle = GPS + optical flow, high = optical flow only.
It sets EKF2_GPS_CTRL and EKF2_OF_CTRL and only acts when the switch moves.
Optical flow only is refused without valid flow, and GPS is added back if the flow is lost.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("aid_switch", "estimator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("set", "Select the source now");
	PRINT_MODULE_USAGE_ARG("gps|both|flow", "Source", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int aid_switch_main(int argc, char *argv[])
{
	return AidSwitch::main(argc, argv);
}
