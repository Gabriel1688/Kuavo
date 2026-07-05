SET (ModelPath ${PROJECT_SOURCE_DIR}/Simulator/SimulationModel/)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_VERBOSE_MAKEFILE ON)

CONFIGURE_FILE(${PROJECT_SOURCE_DIR}//srSimulator/srconfig.h.cmake ${PROJECT_SOURCE_DIR}/srSimulator/srConfiguration.h)


include_directories ("${PROJECT_SOURCE_DIR}/srSimulator")
include_directories ("/usr/local/include/DynaCoRE")
include_directories ("/usr/local/include/eigen3")

add_subdirectory(srDyn)
add_subdirectory(common)
add_subdirectory(LieGroup)
add_subdirectory(srSysGenerator)
add_subdirectory(Systems/Mercury)
