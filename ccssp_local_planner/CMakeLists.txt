cmake_minimum_required(VERSION 2.8.3)
project(ccssp_local_planner)

find_package(catkin REQUIRED
        COMPONENTS
            angles
            base_local_planner
            cmake_modules
            costmap_2d
            dynamic_reconfigure
            nav_core
            nav_msgs
            pluginlib
            sensor_msgs
            roscpp
            tf2
            tf2_geometry_msgs
            tf2_ros
        )

find_package(Eigen3 REQUIRED)
remove_definitions(-DDISABLE_LIBUSB-1.0)
include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})

# dynamic reconfigure
generate_dynamic_reconfigure_options(
    cfg/CCSSPPlanner.cfg
)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ccssp_local_planner
    CATKIN_DEPENDS
        base_local_planner
        dynamic_reconfigure
        nav_msgs
        pluginlib
        sensor_msgs
        roscpp
        tf2
        tf2_ros
)


set(GUROBI_CPP 
        src/cpp/attrprivate.cpp src/cpp/attrprivate.h src/cpp/Batch.cpp src/cpp/Batch.h src/cpp/Callback.cpp src/cpp/Callback.h src/cpp/Column.cpp src/cpp/Column.h src/cpp/Common.h src/cpp/Constr.cpp
        src/cpp/Constr.h src/cpp/Env.cpp src/cpp/Env.h src/cpp/Exception.cpp src/cpp/Exception.h src/cpp/Expr.h src/cpp/GenConstr.cpp src/cpp/GenConstr.h src/cpp/LinExpr.cpp src/cpp/LinExpr.h src/cpp/Model.cpp src/cpp/Model.h src/cpp/parprivate.h src/cpp/public.h src/cpp/QConstr.cpp src/cpp/QConstr.h src/cpp/QuadExpr.cpp src/cpp/QuadExpr.h src/cpp/SOS.cpp src/cpp/SOS.h src/cpp/TempConstr.cpp src/cpp/TempConstr.h src/cpp/Var.cpp src/cpp/Var.h)

include_directories(/opt/gurobi912/linux64/include/)




add_library(ccssp_local_planner src/ccssp_planner.cpp src/ccssp_planner_ros.cpp src/ccssp_solver.cpp src/scored_planner.cpp ${GUROBI_CPP})



add_dependencies(ccssp_local_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


find_library(gurobilib1 gurobi91 /opt/gurobi912/linux64/lib/)
target_link_libraries(ccssp_local_planner LINK_PUBLIC ${gurobilib1} ${catkin_LIBRARIES})

#target_link_libraries(ccssp_local_planner ${catkin_LIBRARIES})

install(TARGETS ccssp_local_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
       )

install(FILES blp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)
