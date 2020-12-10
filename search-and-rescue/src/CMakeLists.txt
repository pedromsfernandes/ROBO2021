add_library(src MODULE search_and_rescue.h search_and_rescue.cpp)
target_link_libraries(src
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
