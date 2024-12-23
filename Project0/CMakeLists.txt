cmake_minimum_required(VERSION 3.10.1)
project(Project0)

set(CMAKE_CXX_STANDARD 11)

find_package(ompl REQUIRED)

# add_executable(Point2DPlanning Point2DPlanning.cpp)
# #If you install from master you will have to link the new way by uncommentting the following line and commenting the rest.
# #target_link_libraries(Point2DPlanning ompl::ompl)
# target_link_libraries(Point2DPlanning PRIVATE ${OMPL_LIBRARIES})
# target_include_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
# target_link_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})

add_executable(Point2DPlanning Point2DPlanning_custom_path.cpp)
#If you install from master you will have to link the new way by uncommentting the following line and commenting the rest.
#target_link_libraries(Point2DPlanning ompl::ompl)
target_link_libraries(Point2DPlanning PRIVATE ${OMPL_LIBRARIES})
target_include_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})
target_link_directories(Point2DPlanning PRIVATE ${OMPL_INCLUDE_DIRS})

# add_executable(TriangulationDemo TriangulationDemo.cpp)
# #If you install from master you will have to link the new way by uncommentting the following line and commenting the rest.
# #target_link_libraries(Point2DPlanning ompl::ompl)
# target_link_libraries(TriangulationDemo PRIVATE ${OMPL_LIBRARIES})
# target_include_directories(TriangulationDemo PRIVATE ${OMPL_INCLUDE_DIRS})
# target_link_directories(TriangulationDemo PRIVATE ${OMPL_INCLUDE_DIRS})