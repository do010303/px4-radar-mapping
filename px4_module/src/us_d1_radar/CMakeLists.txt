px4_add_module(
	MODULE examples__px4_mapping__us_d1_radar
	MAIN us_d1_radar
	SRCS
		UsD1Radar.cpp
		UsD1Radar.hpp
		us_d1_radar_main.cpp
	)

# External modules are configured before PX4's own libraries exist, so the libraries
# the driver uses are linked once the whole tree has been processed.
cmake_language(DEFER DIRECTORY ${PX4_SOURCE_DIR} CALL cmake_policy SET CMP0079 NEW)
cmake_language(DEFER DIRECTORY ${PX4_SOURCE_DIR} CALL target_link_libraries
	examples__px4_mapping__us_d1_radar PRIVATE drivers_rangefinder px4_work_queue)
