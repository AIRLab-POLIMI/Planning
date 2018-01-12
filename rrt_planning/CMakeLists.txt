cmake_minimum_required(VERSION 2.8.3)

find_package(catkin REQUIRED COMPONENTS
	 pluginlib
	 costmap_2d
	 angles
	 dynamicvoronoi 
	 voronoi_planner
	 )
project(rrt_planning)

generate_dynamic_reconfigure_options(
  cfg/VoronoiPlanner.cfg
)

catkin_package(CATKIN_DEPENDS voronoi_planner dynamicvoronoi)

# ----------------------- INCLUDES ----------------------------

include_directories(include ${catkin_INCLUDE_DIRS})

# ----------------------- LIBRARIES ---------------------------

find_package(Eigen3 REQUIRED)
find_package(Boost 1.53 REQUIRED)
find_package(voronoi_planner REQUIRED)
find_package(dynamicvoronoi REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIR})



# ----------------------- GCC FLAGS ----------------------------
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g -std=c++11")

# ------------------------ MESSAGES ----------------------------


# ------------------------ BUILD -------------------------------

file(GLOB_RECURSE rrt_SOURCE src/*.cpp)
list(REMOVE_ITEM rrt_SOURCE "src/Experiment.cpp")

add_library(rrt_planner ${rrt_SOURCE})                        
target_link_libraries(rrt_planner ${catkin_LIBRARIES})    

add_executable(experiment "src/Experiment.cpp") 
target_link_libraries(experiment rrt_planner ${catkin_LIBRARIES})
                        
                        
# ------------------------ TESTS -------------------------------  
add_subdirectory(test)                      

# ------------------------ INSTALLATION ------------------------

# Mark executables and/or libraries for installation
install(TARGETS rrt_planner
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
        )
        
# Install headers
install(DIRECTORY include
        DESTINATION ${CATKIN_GLOBAL_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )

# Install plugin files
install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


# Install launch files
install(DIRECTORY launch config maps rviz_config
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
        PATTERN ".svn" EXCLUDE
        )


# ------------------------ DOCUMENTATION ------------------------
#add_subdirectory(doc EXCLUDE_FROM_ALL)




