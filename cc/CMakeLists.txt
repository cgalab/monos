project (monos VERSION "1.99" DESCRIPTION "monos")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

add_executable(monos
                       main.cpp
                       )
TARGET_LINK_LIBRARIES( monos
                       monoslib )
target_include_directories(monos PRIVATE ../monos/inc)
target_include_directories(monos PRIVATE ../monos/src)
