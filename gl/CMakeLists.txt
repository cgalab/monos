project (monosfrontend VERSION "1.99" DESCRIPTION "monoslib")

find_package(Qt5Widgets)
find_package(CGAL COMPONENTS Qt5 Core Debug REQUIRED)

find_package(Qt5Widgets REQUIRED)

################################
ADD_EXECUTABLE( dummy
                src/dummy.cpp )
TARGET_LINK_LIBRARIES( dummy
                       monoslib )
target_include_directories(dummy PRIVATE ../monos/inc)
target_include_directories(dummy PRIVATE ../monos/src)

################################
ADD_EXECUTABLE( monostool
                src/main.cpp )

TARGET_LINK_LIBRARIES( monostool
                       monoslib )
target_include_directories(monostool PRIVATE ../monos/inc)
target_include_directories(monostool PRIVATE ../monos/src)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

qt5_use_modules(monostool Widgets)
