CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

SET(CMAKE_LEGACY_CYGWIN_WIN32 0)

PROJECT(ozo-slam C CXX)


# default built type
IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE(STATUS "BUILD TYPE:" ${CMAKE_BUILD_TYPE})

# OZO Slam
file(GLOB_RECURSE NOKIA_CXX_SRC "src/*.cpp")
file(GLOB_RECURSE NOKIA_C_SRC "src/*.cc")
#set(NOKIA_SRC ${NOKIA_CXX_SRC} ${NOKIA_C_SRC})
#MESSAGE(STATUS ${NOKIA_SRC})
INCLUDE_DIRECTORIES("${Boost_INCLUDE_DIRS}" "include")

add_executable(ozo-slam ${NOKIA_CXX_SRC})
set_target_properties(ozo-slam PROPERTIES LINKER_LANGUAGE CXX)
add_executable(nokia-convert ${NOKIA_C_SRC})

install(
  TARGETS ozo-slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  COMPONENT ozo-slam)
target_include_directories(
  ozo-slam PUBLIC
  ${PROJECT_SOURCE_DIR}/include)
target_link_libraries(ozo-slam MultiCol-SLAM)

target_link_libraries(nokia-convert MultiCol-SLAM)
