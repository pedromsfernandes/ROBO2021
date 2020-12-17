add_library(src MODULE searcher.h searcher.cpp target.h target.cpp)
target_link_libraries(src
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
