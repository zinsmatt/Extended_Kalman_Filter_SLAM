cmake_minimum_required(VERSION 2.8)

set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")

project(EKF_SLAM)

find_package(SFML 2 REQUIRED graphics window system)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)

include_directories(${SFML_INCLUDE_DIR})

add_executable(${PROJECT_NAME}
    main.cxx
    application.cxx
    application.h
    landmark.cxx
    landmark.h
    )

target_link_libraries(${PROJECT_NAME} ${SFML_LIBS} sfml-graphics sfml-window sfml-system)
