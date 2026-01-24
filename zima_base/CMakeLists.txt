cmake_minimum_required(VERSION 3.2)
project(zima_base)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -Wall")
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build for ${CMAKE_BUILD_TYPE}")

find_package(Protobuf 3.0.0 REQUIRED)

# For generate proto headers and source files.
file(GLOB_RECURSE PROTOBUF_FILES 
  "${PROJECT_SOURCE_DIR}/zima/*.proto"
)

set(PROTO_META_BASE_DIR ${CMAKE_CURRENT_BINARY_DIR})
list(APPEND PROTO_FLAGS
  -I${PROJECT_SOURCE_DIR}
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

set(EXTERNAL_LINK_LIB "-lgflags -lglog -lpthread")

add_library(zima_base SHARED ${COMPONENT_SRCS})
target_link_libraries(zima_base ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

add_library(zima_base_static STATIC ${COMPONENT_SRCS})

add_executable(zima_main src/main.cpp)

set(ZIMA_LINK_LIB "zima_base")

target_link_libraries(zima_main ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# For test program
# add_executable(zima_time_test src/zima_time_test.cpp)
# target_link_libraries(zima_time_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_map_test src/zima_map_test.cpp)
# target_link_libraries(zima_map_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_map_checker src/zima_map_checker.cpp)
# target_link_libraries(zima_map_checker ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_lock_test src/zima_lock_test.cpp)
# target_link_libraries(zima_lock_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_math_test src/zima_math_test.cpp)
# target_link_libraries(zima_math_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_thread_test src/zima_thread_test.cpp)
# target_link_libraries(zima_thread_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_json_test src/zima_json_test.cpp)
# target_link_libraries(zima_json_test ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

# add_executable(zima_kobuki_checker src/zima_kobuki_checker.cpp)
# target_link_libraries(zima_kobuki_checker ${ZIMA_LINK_LIB} ${PROTOBUF_LIBRARY} ${EXTERNAL_LINK_LIB})

install(TARGETS zima_base LIBRARY DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(TARGETS zima_base_static ARCHIVE DESTINATION ${CMAKE_INSTALL_PREFIX}/lib)

install(DIRECTORY 
    ${PROJECT_SOURCE_DIR}/zima INCLUDE DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.h"
)
install(DIRECTORY 
    ${PROJECT_SOURCE_DIR}/zima INCLUDE DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.hpp"
)
install(DIRECTORY 
    ${PROJECT_SOURCE_DIR}/zima INCLUDE DESTINATION ${CMAKE_INSTALL_PREFIX}/include
    FILES_MATCHING PATTERN "*.proto"
)
