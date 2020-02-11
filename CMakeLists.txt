cmake_minimum_required(VERSION 2.8.3)
project(m20ia_pinv_opt)

add_compile_options(-std=c++11)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(Boost REQUIRED COMPONENTS filesystem system regex)
find_package(Pagmo REQUIRED)

find_package(catkin REQUIRED cmake_modules kdl_parser)
find_package(TinyXML2 REQUIRED)

catkin_package(
)

include_directories(
include
${catkin_INCLUDE_DIRS}
)

set(PAGMO_LIBS Pagmo::pagmo  ${Boost_FILESYSTEM_LIBRARIES}  ${Boost_SYSTEM_LIBRARIES}  ${CMAKE_DL_LIBS})

add_library(PInv_Solver SHARED src/PseudoInv_Solver.cpp src/MatrixUtils.cpp src/TrajData.cpp src/Robot_mocap_opt_problem.cpp src/CostComp.cpp src/XMLParser.cpp src/InputParser.cpp  src/ConfigParser.cpp src/CsvWriter.cpp src/RobotKDL.cpp)

add_executable(batch_solver src/batch_solver.cpp)
target_link_libraries(batch_solver PInv_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(batch_solver_varseed src/batch_solver_varseed.cpp)
target_link_libraries(batch_solver_varseed PInv_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(singleTraj_MultFrames_solver src/singleTraj_MultFrames_Solver.cpp)
target_link_libraries(singleTraj_MultFrames_solver PInv_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(singleTraj_MultSeeds_solver src/singleTraj_MultSeeds_Solver.cpp)
target_link_libraries(singleTraj_MultSeeds_solver PInv_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})

add_executable(folderSolver_MultFrames src/folderSolver_MultFrames.cpp)
target_link_libraries(folderSolver_MultFrames PInv_Solver ${PAGMO_LIBS} ${Boost_REGEX_LIBRARIES} ${catkin_LIBRARIES} ${TinyXML2_LIBRARIES})
