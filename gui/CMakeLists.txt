project (monosgui VERSION "1.99" DESCRIPTION "monosgui")

find_package(Qt5Widgets REQUIRED)
find_package(CGAL COMPONENTS Qt5 Core Debug REQUIRED)

include(${CGAL_USE_FILE})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(monosgui
                       main.cpp
                       mainwindow.cpp
                       InputGraphicsItem.cpp
                       ArcGraphicsItem.cpp
                       )
target_link_libraries(monosgui ${QT_LIBRARIES})
target_link_libraries(monosgui ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
TARGET_LINK_LIBRARIES( monosgui
                       monoslib )
target_include_directories(monosgui PRIVATE ../monos/inc)
target_include_directories(monosgui PRIVATE ../monos/src)

qt5_use_modules(monosgui Widgets)
