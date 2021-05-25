add_executable(planning_scene planning_scene.cpp)
target_link_libraries(planning_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(moveit_scene moveit_scene.cpp)
target_link_libraries(moveit_scene ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(code_analysis code_analysis.cpp)
target_link_libraries(code_analysis ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${PCL_LIBRARIES})

add_subdirectory(pcl_tools)