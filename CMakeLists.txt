cmake_minimum_required(VERSION 3.8.0)

project(algorithm_learning)

add_compile_options(-g)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/3rd
)

add_subdirectory(mpc)
