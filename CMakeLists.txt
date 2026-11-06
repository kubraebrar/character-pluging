cmake_minimum_required(VERSION 3.20)
project(student_character_plugin LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Proje içi include dizinini tanımla
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# 1. DLL Hedefi (öğrenci numaran ile çıktı üretir)
add_library(student_character_plugin SHARED
    src/StudentController.cpp
)

set_target_properties(student_character_plugin PROPERTIES
    OUTPUT_NAME "character_plugin_220201031"
    PREFIX ""
)

# 2. Standalone Test Çalıştırıcısı (Test Harness)
add_executable(standalone_test
    tests/standalone_test.cpp
    src/StudentController.cpp
)