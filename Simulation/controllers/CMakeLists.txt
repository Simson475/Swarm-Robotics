add_library(footbot_diffusion MODULE footbot_diffusion.h footbot_diffusion.cpp connector.h connector.cpp)


target_link_libraries(footbot_diffusion
	/usr/include/nlohmann/json.hpp
	argos3core_simulator
	argos3plugin_simulator_entities
	argos3plugin_simulator_footbot
	argos3plugin_simulator_genericrobot
	argos3plugin_simulator_qtopengl
)

