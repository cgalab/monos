project (offsetfrontend VERSION "1.99" DESCRIPTION "offsetlib")

find_package(Qt5Widgets)
find_package(CGAL COMPONENTS Qt5 Core Debug REQUIRED)

find_package(Qt5Widgets REQUIRED)

################################
ADD_EXECUTABLE( dummy
                src/dummy.cpp )
TARGET_LINK_LIBRARIES( dummy
                       offsetlib )
target_include_directories(dummy PRIVATE ../offsetter/inc)
target_include_directories(dummy PRIVATE ../offsetter/src)

################################
ADD_EXECUTABLE( offset
                src/main.cpp )

TARGET_LINK_LIBRARIES( offset
                       offsetlib )
target_include_directories(offset PRIVATE ../monos/inc)
target_include_directories(offset PRIVATE ../monos/src)

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

qt5_use_modules(offset Widgets)
