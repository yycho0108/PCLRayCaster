cmake_minimum_required(VERSION 2.8)
project(PCLRayCaster)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_EXPORT_COMPILE_COMMANDS 1)

find_package(PCL REQUIRED COMPONENTS COMMON IO OCTREE SEARCH FEATURES)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(fmt REQUIRED)

set(SRC_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/src/main.cpp
    )

set(HDR_FILES
    ${CMAKE_CURRENT_SOURCE_DIR}/include/raycaster/raycaster.hpp
    ${CMAKE_CURRENT_SOURCE_DIR}/include/raycaster/twist.hpp
    )

add_executable(main ${SRC_FILES} ${HDR_FILES})
target_include_directories(main PUBLIC "${CMAKE_CURRENT_SOURCE_DIR}/include/" ${PCL_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
target_link_libraries(main ${PCL_LIBRARIES} Eigen3::Eigen Boost::system fmt)

# Dummy target for always building...
add_custom_target(copy_python_scripts ALL
    DEPENDS
        copy_python_scripts_rsync
    )

# Dummy target for always building...
add_custom_command(
    OUTPUT always_rebuild
    COMMAND cmake -E echo
    )

# Copy files in scripts/ to binary directory as well.
add_custom_command(
    OUTPUT copy_python_scripts_rsync
    COMMAND rsync
    ARGS -ravP
    "${CMAKE_CURRENT_SOURCE_DIR}/scripts/"
    "$<TARGET_FILE_DIR:main>/scripts/"
    DEPENDS always_rebuild
    )
