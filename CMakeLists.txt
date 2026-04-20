cmake_minimum_required(VERSION 3.14)
project(dbc-can)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(FetchContent)
FetchContent_Declare(
  cpp-can-parser
  GIT_REPOSITORY https://github.com/umdloop/cpp-can-parser.git
  GIT_TAG        master
)
set(CPP_CAN_PARSER_USE_STATIC TRUE)
set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(cpp-can-parser)

target_include_directories(cpp-can-parser PRIVATE
    ${cpp-can-parser_SOURCE_DIR}/include/cpp-can-parser)


add_executable(dbc-can
    src/main.cpp
    src/generator.cpp)

target_link_libraries(dbc-can PRIVATE cpp-can-parser)
target_include_directories(dbc-can PRIVATE include)

set(GENERATED_DIR ${CMAKE_CURRENT_BINARY_DIR}/generated)
set(GENERATED_HPP ${GENERATED_DIR}/UMDLoopCANProtocol.hpp)
file(MAKE_DIRECTORY ${GENERATED_DIR})

set(DBC_FILE "${CMAKE_CURRENT_SOURCE_DIR}/dbc/protocol.dbc")
add_custom_command(
    OUTPUT  ${GENERATED_HPP}
    COMMAND dbc-can "${DBC_FILE}" "${GENERATED_DIR}"
    DEPENDS dbc-can "${DBC_FILE}"
    COMMENT "Generating UMDLoopCANProtocol.hpp"
    VERBATIM
)
add_custom_target(generate-protocol DEPENDS ${GENERATED_HPP})

add_executable(protocol-test example/main.cpp)
add_dependencies(protocol-test generate-protocol)
target_include_directories(protocol-test PRIVATE ${GENERATED_DIR})
set_target_properties(protocol-test PROPERTIES CXX_STANDARD 17)
