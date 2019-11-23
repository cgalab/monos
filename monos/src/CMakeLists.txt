set(SRC_LIST  
               main.cpp cgTypes.cpp Data.cpp Skeleton.cpp
               Wavefront.cpp tools.cpp Monos.cpp Config.cpp
               BasicInput.cpp BGLGraph.cpp
               EventQueue.cpp
               easyloggingpp/src/easylogging++.cc
)

set_source_files_properties(easyloggingpp/src/easylogging++.cc PROPERTIES COMPILE_FLAGS "-Wno-float-equal -Wno-missing-noreturn")

find_package( CGAL COMPONENTS MPFI Core )
if ( NOT CGAL_FOUND )
  message(STATUS "CGAL notfound.")
  return()
endif()
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}. CGAL_USE_FILE is ${CGAL_USE_FILE}" )
message("CMAKE_BUILD_TYPE is ${CMAKE_BUILD_TYPE}." )
set ( CGAL_BUILD_TYPE_INIT ${CMAKE_BUILD_TYPE} )
include( ${CGAL_USE_FILE} )
add_definitions(-DCGAL_USE_CORE)

find_package( Boost REQUIRED graph)
if ( NOT Boost_FOUND )
   message(STATUS "Boost not found.")
   return()
endif()
include_directories(${Boost_INCLUDE_DIRS})

add_executable(monos ${SRC_LIST})

target_link_libraries(monos ${Boost_LIBRARIES}
                            ${CGAL_LIBRARIES}
                            ${CGAL_3RD_PARTY_LIBRARIES}
                            ${MAP_VIEWER_LIBRARIES}
                     )

