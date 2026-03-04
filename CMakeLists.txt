cmake_minimum_required(VERSION 3.28)
project(HAL VERSION 0.1 LANGUAGES CXX)

option(WORK_QUEUE_PREEMPTIVE "Enable preemptive WorkQueue scheduling" ON)

# Platform selection - choose one of ARDUINO, STM32 or MOCK.  The
# cache variable remembers the choice between invocations and the
# STRINGS property limits possible values shown by ccmake/gui.
set(USST_PLATFORM "ARDUINO" CACHE STRING "Target platform (ARDUINO, STM32, MOCK)")
set_property(CACHE USST_PLATFORM PROPERTY STRINGS
    ARDUINO STM32 MOCK)

message(STATUS "HAL: Building for platform: ${USST_PLATFORM}")
message(STATUS "HAL: WORK_QUEUE_PREEMPTIVE=${WORK_QUEUE_PREEMPTIVE}")

add_library(${PROJECT_NAME} STATIC)

if(USST_PLATFORM STREQUAL "ARDUINO")
    add_subdirectory(implementations/arduino)
elseif(USST_PLATFORM STREQUAL "STM32")
    add_subdirectory(implementations/stm32)
elseif(USST_PLATFORM STREQUAL "MOCK")
    add_subdirectory(implementations/mock)
else()
    message(FATAL_ERROR "Unknown USST_PLATFORM: ${USST_PLATFORM}. Valid options: ARDUINO, STM32, MOCK")
endif()

target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_20)
set_target_properties(${PROJECT_NAME}
    PROPERTIES
    SOVERSION 0
    VERSION 0.1
)

target_include_directories(${PROJECT_NAME}
    PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

# Define the platform macro for compile-time selection
target_compile_definitions(${PROJECT_NAME} PUBLIC
    USST_PLATFORM_${USST_PLATFORM}
    WORK_QUEUE_PREEMPTIVE=$<BOOL:${WORK_QUEUE_PREEMPTIVE}>
)

if(CMAKE_SOURCE_DIR STREQUAL PROJECT_SOURCE_DIR)
    add_subdirectory(Test)
endif()
