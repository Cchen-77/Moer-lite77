cmake_minimum_required(VERSION 3.15)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

project(Moer-lite LANGUAGES CXX)

if(MSVC)
  add_compile_options("/MP")
  add_compile_options("$<$<C_COMPILER_ID:MSVC>:/utf-8>")
  add_compile_options("$<$<CXX_COMPILER_ID:MSVC>:/utf-8>")
  add_definitions(-D_USE_MATH_DEFINES)
endif(MSVC)

if(WIN32)
  add_definitions(-DUNICODE -D_UNICODE)
  add_definitions(-DNOMINMAX)
endif()

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/target/bin)

add_subdirectory(./externals)

file(GLOB_RECURSE sources_list "${PROJECT_SOURCE_DIR}/src/*.cpp" "${PROJECT_SOURCE_DIR}/src/*.h")

source_group(TREE ${PROJECT_SOURCE_DIR} FILES ${sources_list})

add_executable(Moer ${sources_list})

target_include_directories(Moer PUBLIC ./externals)
target_include_directories(Moer PUBLIC ./src)
target_include_directories(Moer PUBLIC ./externals/json/include)
target_include_directories(Moer PUBLIC ./externals/embree/include)

if(WIN32)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/win)
  set(EMBREE_DLL ${LIBS_PATH}/embree3.dll ${LIBS_PATH}/tbb12.dll)
endif()

if(UNIX AND NOT APPLE)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/linux)
  set(EMBREE_DLL ${LIBS_PATH}/libembree3.so.3 ${LIBS_PATH}/libtbb.so.12.2)
endif()

if(APPLE)
  set(LIBS_PATH ${PROJECT_SOURCE_DIR}/libs/macos)
  set(EMBREE_DLL ${LIBS_PATH}/libembree3.3.dylib ${LIBS_PATH}/libtbb.12.2.dylib)
endif()

target_link_directories(Moer PUBLIC ${LIBS_PATH})
target_link_directories(Moer PUBLIC ${PROJECT_SOURCE_DIR}/target/lib)

add_custom_command(TARGET Moer POST_BUILD
  DEPENDS ${LIBS_PATH}/embree3.dll
  COMMAND ${CMAKE_COMMAND} -E copy_if_different
  ${EMBREE_DLL} $<TARGET_FILE_DIR:Moer>)

target_link_libraries(Moer PUBLIC embree3)
target_link_libraries(Moer PUBLIC tinyobjloader)

if(APPLE)
  execute_process(
    COMMAND uname -m
    RESULT_VARIABLE result
    OUTPUT_VARIABLE MOER_OSX_NATIVE_ARCHITECTURE
    OUTPUT_STRIP_TRAILING_WHITESPACE
  )

  if(${MOER_OSX_NATIVE_ARCHITECTURE} STREQUAL "arm64")
    set_target_properties(Moer PROPERTIES LINK_FLAGS "-undefined dynamic_lookup")
  endif()
endif()