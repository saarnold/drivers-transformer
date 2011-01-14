project(transformer)
set(PROJECT_VERSION 1.0)

set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR})

cmake_minimum_required(VERSION 2.6)
include(FindPkgConfig)

add_definitions (-ggdb)

# Enable -Wall for compilers that know it
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-Wall" CXX_SUPPORTS_WALL)
if (CXX_SUPPORTS_WALL)
    add_definitions (-Wall)
endif (CXX_SUPPORTS_WALL)

include_directories(BEFORE .)

find_package(Boost REQUIRED)
include_directories(${Boost_INCLUDE_DIRS})
link_directories(${Boost_LIBRARY_DIRS})

pkg_check_modules(DFKI REQUIRED "base-types")
include_directories(${DFKI_INCLUDE_DIRS})
link_directories(${DFKI_LIBRARY_DIRS})

pkg_check_modules(AGGREGATOR REQUIRED "aggregator")
include_directories(${AGGREGATOR_INCLUDE_DIRS})
link_directories(${AGGREGATOR_LIBRARY_DIRS})

ADD_LIBRARY(transformer SHARED
	    Transformer.cpp)

find_package( Boost COMPONENTS unit_test_framework )

add_executable(TransformTest TestTransformationMaker.cpp)
target_link_libraries(TransformTest transformer  ${Boost_LIBRARIES} )

configure_file(transformer.pc.in transformer.pc @ONLY)

INSTALL(TARGETS transformer
    RUNTIME DESTINATION bin
    LIBRARY DESTINATION lib)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/transformer.pc
    DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/pkgconfig)

install(FILES Transformer.h
    DESTINATION ${CMAKE_INSTALL_PREFIX}/include/transformer/)

install(FILES ${PROJECT_SOURCE_DIR}/orogen_plugin/transformer_plugin.rb
    DESTINATION ${CMAKE_INSTALL_PREFIX}/share/orogen/plugins)