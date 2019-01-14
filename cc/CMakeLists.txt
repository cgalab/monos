project (offsetter VERSION "1.99" DESCRIPTION "offsetter")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(offsetter
                       main.cpp
                       )
TARGET_LINK_LIBRARIES( offsetter
                       offsetlib )
target_include_directories(offsetter PRIVATE ../offsetter/inc)
target_include_directories(offsetter PRIVATE ../offsetter/src)
