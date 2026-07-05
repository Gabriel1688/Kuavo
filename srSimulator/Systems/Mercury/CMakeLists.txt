FILE(GLOB_RECURSE header *.h)
file(GLOB_RECURSE source *.cpp)
add_executable (simulation_mercury ${header} ${source} )

target_link_libraries (simulation_mercury
  common
  LieGroup
  SR_SysGenerator
  srDyn
#  srg
#  Renderer
  util
#  ${OPENGL_LIBRARIES}
  dynacore_urdf
  Mercury_Controller
  dynacore_utils
  dynacore_param_handler
  pthread
)
#
#if (CMAKE_SYSTEM_NAME MATCHES Linux)
#  target_link_libraries(simulation_mercury ${GLUT_glut_LIBRARIES})
#  include (CheckIncludeFileCXX)
#endif (CMAKE_SYSTEM_NAME MATCHES Linux)

