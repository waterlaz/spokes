CMAKE_MINIMUM_REQUIRED(VERSION 3.7)
PROJECT(rectificationTest)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -ggdb -lpthread")

FIND_PACKAGE(Eigen3 3.3 REQUIRED NO_MODULE )

INCLUDE_DIRECTORIES(
    ${EIGEN3_INCLUDE_DIRS}
    ${CMAKE_SOURCE_DIR}/src
)

set(SOURCE
    spokes.cpp
    PostScript.hpp
    )

set(LINK_LIBS
    Eigen3::Eigen
)


ADD_EXECUTABLE(spokes spokes.cpp)


TARGET_LINK_LIBRARIES(spokes
    ${LINK_LIBS}
)


SET(CMAKE_BUILD_TYPE Release)
#SET(CMAKE_BUILD_TYPE Debug)

