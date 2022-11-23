cmake_minimum_required(VERSION 3.20)
project(cassie_mujoco_sim LANGUAGES C CXX)

# -------------------------------------------------
# Configuration
# -------------------------------------------------
set(cassie_mujoco_dir ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)
set(CMAKE_C_VISIBILITY_PRESET hidden)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})

find_package(Threads REQUIRED)
find_library(libdl NAMES dl REQUIRED)

# DRL provides a cassie library pre-compiled and shipped
# with the repository.
add_library(AgilityCassie INTERFACE)
find_library(agility_cassie_lib_
  NAMES agilitycassie
  HINTS "${cassie_mujoco_dir}/external/agilitycassie/lib"
  REQUIRED)
target_link_libraries(AgilityCassie INTERFACE
  ${agility_cassie_lib_})
target_include_directories(AgilityCassie INTERFACE 
  "${cassie_mujoco_dir}/external/agilitycassie/include")

add_subdirectory(external/pybind11)

# Mujoco might exist as a target if this project
# is being built as part of a larger project and 
# mujoco is built from source. Otherwise, find the 
# installed package.
if(TARGET mujoco::mujoco)
  message(STATUS "CassieSim: Using mujoco currently being built")
  set(mujoco_dep_ mujoco::mujoco)
else()
  message(STATUS "CassieSim: Using mujoco from prebuilt package")
  find_package(mujoco REQUIRED)
  set(mujoco_dep_ mujoco::mujoco)  
endif()

add_subdirectory(external/mujoco/simulate)
include_directories(external/agilitycassie/include)

# -------------------------------------------------
# Targets
# -------------------------------------------------
include(cmake/Targets.cmake)
add_compile_definitions(CASSIE_SIM_MODEL_DIR="${CMAKE_CURRENT_LIST_DIR}/model/")

add_subdirectory(lib)
add_subdirectory(tools)
add_subdirectory(test)
add_subdirectory(python)