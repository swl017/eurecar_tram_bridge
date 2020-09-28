#set(CMAKE_VERBOSE_MAKEFILE on)


#only compile if odeint is installed (this is a workaround for now since odeint
#does not come in a version fo boost that ROS depends on yet (need boost >= 1.53)

include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
add_executable(eurecar_tram_bridge eurecar_tram_bridge.cpp)
# add_dependencies(eurecar_tram_bridge ${PROJECT_NAME}_gencfg)
target_link_libraries(eurecar_tram_bridge ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(eurecar_tram_bridge ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(eurecar_tram_bridge
  ${catkin_LIBRARIES}
)

#catkin_install_python(PROGRAMS GenerateWaypoints
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


install(TARGETS
  eurecar_tram_bridge
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
