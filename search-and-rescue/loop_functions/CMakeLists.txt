link_directories(${CMAKE_BINARY_DIR}/controllers)
set(search_loop_functions_SOURCES search_loop_functions.cpp)

if(ARGOS_COMPILE_QTOPENGL)
  set(search_loop_functions_SOURCES
    ${search_loop_functions_SOURCES}
    id_qtuser_functions.cpp)
endif(ARGOS_COMPILE_QTOPENGL)

add_library(search_loop_functions MODULE ${search_loop_functions_SOURCES})

target_link_libraries(search_loop_functions
  argos3core_simulator
  argos3plugin_simulator_entities
  argos3plugin_simulator_footbot)

if(ARGOS_COMPILE_QTOPENGL)
  target_link_libraries(search_loop_functions argos3plugin_simulator_qtopengl)
endif(ARGOS_COMPILE_QTOPENGL)