px4_add_module(
	MODULE modules__aid_switch
	MAIN aid_switch
	SRCS
		AidSwitch.cpp
		AidSwitch.hpp
	DEPENDS
		px4_work_queue
	MODULE_CONFIG
		module.yaml
	)
