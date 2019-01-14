project (offsetgui VERSION "1.99" DESCRIPTION "offsetgui")

find_package(Qt5Widgets REQUIRED)
find_package(CGAL COMPONENTS Qt5 Core Debug REQUIRED)

include(${CGAL_USE_FILE})

set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)

add_executable(offsetgui
                       main.cpp
                       mainwindow.cpp
                       InputGraphicsItem.cpp
                       )
target_link_libraries(offsetgui ${QT_LIBRARIES})
target_link_libraries(offsetgui ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})
TARGET_LINK_LIBRARIES( offsetgui
                       offsetlib )
target_include_directories(offsetgui PRIVATE ../monos/inc)
target_include_directories(offsetgui PRIVATE ../monos/src)

qt5_use_modules(offsetgui Widgets)
