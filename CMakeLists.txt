cmake_minimum_required(VERSION 3.8 FATAL_ERROR)
project(MCL LANGUAGES CXX C)
set(CMAKE_CXX_STANDARD 17)

include(GNUInstallDirs)
set(PACKAGE_NAME MCL)
set(INSTALL_CONFIGDIR ${CMAKE_INSTALL_LIBDIR}/${PACKAGE_NAME}/cmake)


# Library target
add_library(matplotlib_cpp INTERFACE)
target_include_directories(matplotlib_cpp
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/examples>
    $<INSTALL_INTERFACE:include>
)
target_compile_features(matplotlib_cpp INTERFACE
  cxx_std_11
)

find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
target_link_libraries(matplotlib_cpp INTERFACE
  Python3::Python
  Python3::Module
)
find_package(Python3 COMPONENTS NumPy)
if(Python3_NumPy_FOUND)
  target_link_libraries(matplotlib_cpp INTERFACE
    Python3::NumPy
  )
else()
  target_compile_definitions(matplotlib_cpp INTERFACE WITHOUT_NUMPY)
endif()
install(
  TARGETS matplotlib_cpp
  EXPORT install_targets
)

add_executable(MCL main.cpp)
target_link_libraries(MCL PRIVATE matplotlib_cpp)
set_target_properties(MCL PROPERTIES RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin")