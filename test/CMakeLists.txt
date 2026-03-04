# Enable testing
enable_testing()

# Force MOCK platform for tests
set(USST_PLATFORM "MOCK" CACHE STRING "Target platform (ARDUINO, STM32, MOCK)" FORCE)

include(FetchContent)
FetchContent_Declare(
  googletest
  URL https://github.com/google/googletest/archive/50b8600c63c5487e901e2845a0f64d384a65f75d.zip
)
# For Windows: Prevent overriding the parent project's compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# Add your test executable
set(TEST_RUNNER test_runner)

add_executable(${TEST_RUNNER})

# ---------- Compiler Configuration ----------
target_compile_features(${TEST_RUNNER} PRIVATE cxx_std_20)

# Enable warnings
if (CMAKE_CXX_COMPILER_ID MATCHES "Clang|GNU")
  target_compile_options(${TEST_RUNNER} PRIVATE 
    -Wall
    -Wextra
    -Wpedantic
    -Wshadow
    -Wunused
    -Wconversion
    # Address Sanitizer
    -fsanitize=address
    -fno-omit-frame-pointer
  )
  target_link_options(${TEST_RUNNER} PRIVATE -fsanitize=address)

elseif (CMAKE_CXX_COMPILER_ID STREQUAL "MSVC")
  target_compile_options(${TEST_RUNNER} PRIVATE 
    /W4 
    /Zi 
    /RTC1 
    /EHsc
    # Address Sanitizer (MSVC 16.9+)
    /fsanitize=address
  )
else()
  message(WARNING "Unknown compiler: ${CMAKE_CXX_COMPILER_ID}")
endif()

# Link with GoogleTest libraries
target_link_libraries(${TEST_RUNNER} PRIVATE 
  gtest 
  gtest_main 
  gmock 
  gmock_main 
  ${PROJECT_NAME}
)

# Include HAL headers
target_include_directories(${TEST_RUNNER} PRIVATE 
  ${CMAKE_CURRENT_SOURCE_DIR}/..
)

# Use WORK_QUEUE_PREEMPTIVE from top-level CMake configuration
target_compile_definitions(${TEST_RUNNER} PRIVATE
  WORK_QUEUE_PREEMPTIVE=$<BOOL:${WORK_QUEUE_PREEMPTIVE}>
  $<$<CXX_COMPILER_ID:MSVC>:_DISABLE_VECTOR_ANNOTATION=1>
  $<$<CXX_COMPILER_ID:MSVC>:_DISABLE_STRING_ANNOTATION=1>
)

# Register tests with CTest
include(GoogleTest)
gtest_discover_tests(${TEST_RUNNER})

# Declare your test source files
target_sources(${TEST_RUNNER} PRIVATE
  TestWorkQueue.cpp
)