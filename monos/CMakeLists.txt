project (surf VERSION "1.99" DESCRIPTION "monoslib")

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

#find_package(MPFI)
#if ( NOT MPFR_FOUND )
#  message(STATUS "MPFI not found.")
#  return()
#endif()
#include(${MPFI_USE_FILE})

#include( CGAL_CreateSingleSourceCGALProgram )

include_directories(${Boost_INCLUDE_DIRS})

# COMPILER SETTINGS

#set( CMAKE_CXX_FLAGS_DEBUG  "${CMAKE_CXX_FLAGS_DEBUG} -Werror" )
# BINARIES AND SOURCES

# cause easylogging to pretty please don't print to stdout.

add_library(monoslib
  src/cgTypes.cpp
  src/BGLGraph.cpp
  src/BasicInput.cpp
  src/Config.cpp
  src/tools.cpp
  src/Data.cpp
  src/Skeleton.cpp
  src/Wavefront.cpp
  src/Monos.cpp
  src/EventQueue.cpp 
  easyloggingpp/src/easylogging++.cc
  )
set_target_properties(monoslib PROPERTIES VERSION ${PROJECT_VERSION})

target_link_libraries(monoslib ${Boost_LIBRARIES})
target_link_libraries(monoslib ${CGAL_LIBRARIES} ${CGAL_3RD_PARTY_LIBRARIES})

set_target_properties(monoslib PROPERTIES PUBLIC_HEADER inc/tools.h)

target_compile_definitions(monoslib PRIVATE ELPP_CUSTOM_COUT=std::cerr)

target_include_directories(monoslib PRIVATE inc)
target_include_directories(monoslib PRIVATE src)

include(GNUInstallDirs)

