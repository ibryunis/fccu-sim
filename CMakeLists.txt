cmake_minimum_required(VERSION 3.20)
project(fccu_sim CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(Threads REQUIRED)

add_library(fccu_core STATIC
    src/fuel_cell.cpp
    src/plant.cpp
    src/state_machine.cpp
    src/control_loop.cpp
    src/safety.cpp
    src/runtime.cpp
)
target_include_directories(fccu_core PUBLIC include)
target_link_libraries(fccu_core PUBLIC Threads::Threads)
target_compile_options(fccu_core PRIVATE -Wall -Wextra)

add_executable(fccu_sim src/main.cpp src/http_api.cpp)
target_include_directories(fccu_sim PRIVATE src third_party)
target_link_libraries(fccu_sim PRIVATE fccu_core)
target_compile_options(fccu_sim PRIVATE -Wall -Wextra)
if(WIN32)
    target_link_libraries(fccu_sim PRIVATE ws2_32)
endif()

add_executable(fccu_tests
    tests/test_fuel_cell.cpp
    tests/test_tank.cpp
    tests/test_actuators.cpp
    tests/test_sensors.cpp
    tests/test_state_machine.cpp
    tests/test_control_loop.cpp
    tests/test_safety.cpp
    tests/test_integration.cpp
    tests/test_http.cpp
    src/http_api.cpp
    third_party/catch_amalgamated.cpp
)
target_include_directories(fccu_tests PRIVATE src third_party)
target_link_libraries(fccu_tests PRIVATE fccu_core)
target_compile_options(fccu_tests PRIVATE -Wall -Wextra)
if(WIN32)
    target_link_libraries(fccu_tests PRIVATE ws2_32)
endif()

enable_testing()
add_test(NAME unit COMMAND fccu_tests)
