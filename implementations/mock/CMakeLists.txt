# CMake support for mock implementations.  This directory is added when
# USST_PLATFORM==MOCK.

# list mock implementation files explicitly
target_sources(${PROJECT_NAME} 
    PRIVATE
    "${CMAKE_CURRENT_SOURCE_DIR}/MockFile.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MockFileSystem.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/MockWorkQueue.cpp"
)

 target_include_directories(${PROJECT_NAME} PUBLIC
     ${CMAKE_CURRENT_SOURCE_DIR}
 )