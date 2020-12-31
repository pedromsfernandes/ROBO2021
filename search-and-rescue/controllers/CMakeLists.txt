add_library(controllers SHARED searcher.h searcher.cpp)
target_link_libraries(controllers
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot)
