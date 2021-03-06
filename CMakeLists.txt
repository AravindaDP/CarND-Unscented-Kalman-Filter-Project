project(UnscentedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++11)

set(UKF_ENABLE_CUKE OFF CACHE BOOL "Enable Cucumber Tests")

set(CXX_FLAGS "-Wall")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${CXX_FLAGS}")
set(sources src/ukf.cpp src/main.cpp src/tools.cpp)

if(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 
include_directories(/usr/local/include)
include_directories(/usr/local/opt/openssl/include)
link_directories(/usr/local/lib)
link_directories(/usr/local/opt/openssl/lib)
link_directories(/usr/local/Cellar/libuv/1*/lib)
endif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin") 

add_executable(UnscentedKF ${sources})
if(${CMAKE_SYSTEM_NAME} MATCHES "Linux")
  target_link_libraries(UnscentedKF z ssl uv uWS python2.7)
else()
  target_link_libraries(UnscentedKF z ssl uv uWS python)
endif()

# Download and unpack googletest at configure time
configure_file(CMakeLists.txt.in googletest-download/CMakeLists.txt)
execute_process(COMMAND ${CMAKE_COMMAND} -G "${CMAKE_GENERATOR}" .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "CMake step for googletest failed: ${result}")
endif()
execute_process(COMMAND ${CMAKE_COMMAND} --build .
  RESULT_VARIABLE result
  WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/googletest-download )
if(result)
  message(FATAL_ERROR "Build step for googletest failed: ${result}")
endif()

# Prevent overriding the parent project's compiler/linker
# settings on Windows
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)

# Add googletest directly to our build. This defines
# the gtest and gtest_main targets.
add_subdirectory(${CMAKE_CURRENT_BINARY_DIR}/googletest-src
                 ${CMAKE_CURRENT_BINARY_DIR}/googletest-build
                 EXCLUDE_FROM_ALL)

# The gtest/gtest_main targets carry header search path
# dependencies automatically when using CMake 2.8.11 or
# later. Otherwise we have to add them here ourselves.
if (CMAKE_VERSION VERSION_LESS 2.8.11)
  include_directories("${gtest_SOURCE_DIR}/include")
endif()

enable_testing()

# Now simply link against gtest or gtest_main as needed. Eg
add_executable(UnscentedKFTests src/tools.cpp src/ukf.cpp test/tools_test.cpp test/ukf_test.cpp)
target_link_libraries(UnscentedKFTests gmock_main)
add_test(NAME ukf_tests COMMAND UnscentedKFTests)

if (UKF_ENABLE_CUKE)
  include_directories(/include)
  set(Boost_USE_STATIC_LIBS OFF)
  set(Boost_USE_MULTITHREADED ON)
  set(Boost_USE_STATIC_RUNTIME OFF)
  find_package(Boost 1.45.0 COMPONENTS thread system regex date_time program_options filesystem)
  add_executable(UnscentedKFSteps src/tools.cpp src/ukf.cpp features/step_definitions/helpers.cpp 
                 features/step_definitions/tools_steps.cpp features/step_definitions/ukf_steps.cpp)
  target_link_libraries(UnscentedKFSteps cucumber-cpp gmock_main ${Boost_LIBRARIES})
endif()