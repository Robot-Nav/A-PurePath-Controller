cmake_minimum_required(VERSION 3.10)
project(graceful_controller_standalone)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

add_compile_options(-Wall -Wextra -Wpedantic)

add_library(graceful_controller_standalone STATIC
    graceful_controller_standalone.cpp
)

target_include_directories(graceful_controller_standalone PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)

install(TARGETS graceful_controller_standalone
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
)

install(FILES 
    graceful_controller_standalone.hpp
    DESTINATION include
)

install(DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

