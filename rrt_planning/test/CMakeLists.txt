add_executable(test_kinematics TestKinematics.cpp)
target_link_libraries(test_kinematics rrt_planner ${catkin_LIBRARIES})

add_executable(test_node TestNode.cpp)
target_link_libraries(test_node ${catkin_LIBRARIES})

add_executable(test_controller TestController.cpp)
target_link_libraries(test_controller rrt_planner ${catkin_LIBRARIES})

##add_executable(replay_node ReplayNode.cpp)
##target_link_libraries(replay_node ${catkin_LIBRARIES})
