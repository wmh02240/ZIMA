cmake_minimum_required(VERSION 3.2)
project(zima_core)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -Wall")
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build for ${CMAKE_BUILD_TYPE}")

find_package(Protobuf 3.0.0 REQUIRED)

# For generate proto headers and source files.
# add_subdirectory(zima/proto)
file(GLOB_RECURSE PROTOBUF_FILES 
  "${PROJECT_SOURCE_DIR}/zima/*.proto"
)

set(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND PROTO_FLAGS
  -I${PROJECT_SOURCE_DIR}
  -I${CMAKE_INSTALL_PREFIX}/include
)

message(STATUS "Using Protocol Buffers ${Protobuf_VERSION}")

FOREACH(FIL ${PROTOBUF_FILES})

    GET_FILENAME_COMPONENT(FIL_WE ${FIL} NAME_WE)
    GET_FILENAME_COMPONENT(FILE_PATH ${FIL} DIRECTORY)
    # message(STATUS "FILE_PATH ${FILE_PATH}")

    # string(REGEX REPLACE ".+/(.+)\\..*" "\\1" FILE_NAME ${FIL})
    # string(REGEX REPLACE "(.+)\\${FILE_NAME}.*" "\\1" FILE_PATH ${FIL})

    string(REGEX MATCH "(zima/proto.*)" OUT_PATH ${FILE_PATH})
    # message(STATUS "OUT_PATH ${OUT_PATH}")

    # message(STATUS "CMAKE_CURRENT_BINARY_DIR ${CMAKE_CURRENT_BINARY_DIR}")
    # message(STATUS "FIL ${FIL}")
    # message(STATUS "FIL_WE ${FIL_WE}")
    # message(STATUS "PROTO_META_BASE_DIR ${PROTO_META_BASE_DIR}")

    set(PROTO_SRC "${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}/${FIL_WE}.pb.cc")
    set(PROTO_HDR "${CMAKE_CURRENT_BINARY_DIR}/${OUT_PATH}/${FIL_WE}.pb.h")

    EXECUTE_PROCESS(
        COMMAND ${PROTOBUF_PROTOC_EXECUTABLE} ${PROTO_FLAGS} --cpp_out=${PROTO_META_BASE_DIR} ${FIL}
    )
    # message("Copying " ${PROTO_SRC} " to " ${FILE_PATH})
    # message("Copying " ${PROTO_HDR} " to " ${FILE_PATH})

    file(COPY ${PROTO_SRC} DESTINATION ${FILE_PATH})
    file(COPY ${PROTO_HDR} DESTINATION ${FILE_PATH})

ENDFOREACH()

include_directories(
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/zima
    ${PROTOBUF_INCLUDE_DIRS}
)

file(GLOB_RECURSE COMPONENT_SRCS
  "${PROJECT_SOURCE_DIR}/zima/*.cpp"
  "${PROJECT_SOURCE_DIR}/zima/*.cc"
  "${PROJECT_SOURCE_DIR}/zima/*.c"
)
# foreach(src_file ${COMPONENT_SRCS})
#     message(STATUS "src_file: ${src_file}")
# endforeach()

set(ZIMA_LINK_LIB "zima_core")
set(EXTERNAL_LINK_LIB "-lgflags -lglog -lpthread -lzima_base")

add_library(zima_core SHARED ${COMPONENT_SRCS})
target_link_libraries(zima_core ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

add_library(zima_core_static STATIC ${COMPONENT_SRCS})

add_executable(zima_main src/main.cpp)
target_link_libraries(zima_main ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# For test program
# add_executable(zima_matcher_test src/zima_matcher_test.cpp)
# target_link_libraries(zima_matcher_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_path_planner_test src/zima_path_planner_test.cpp)
# target_link_libraries(zima_path_planner_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_nav_data_checker src/zima_nav_data_checker.cpp)
# target_link_libraries(zima_nav_data_checker ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_auto_cleaning_mode_test src/zima_auto_cleaning_mode_test.cpp)
# target_link_libraries(zima_auto_cleaning_mode_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_line_segment_test src/zima_line_segment_test.cpp)
# target_link_libraries(zima_line_segment_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_pose_interpolator_test src/zima_pose_interpolator_test.cpp)
# target_link_libraries(zima_pose_interpolator_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_probability_map_test src/zima_probability_map_test.cpp)
# target_link_libraries(zima_probability_map_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

install(TARGETS zima_core LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(TARGETS zima_core_static ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(DIRECTORY 
    ${PROJECT_SOURCE_DIR}/zima INCLUDE DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h"
)
