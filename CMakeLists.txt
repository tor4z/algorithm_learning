cmake_minimum_required(VERSION 3.10.0)

project(algorithm_learning)

add_compile_options(-g)

include_directories(
    ${CMAKE_CURRENT_LIST_DIR}/algorithm_utils
)

add_subdirectory(mpc)
add_subdirectory(a_star)
add_subdirectory(bicycle_model)
