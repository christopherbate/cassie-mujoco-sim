cmake_minimum_required(VERSION 3.20)
project(cassie_mujoco_sim LANGUAGES C CXX)

# -------------------------------------------------
# Configuration
# -------------------------------------------------
set(cassie_mujoco_dir ${CMAKE_CURRENT_LIST_DIR})

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

include(cmake/CPM.cmake)
include(cmake/Tools.cmake)

# -------------------------------------------------
# Dependencies
# -------------------------------------------------
find_package(glfw3 3.3 REQUIRED)

find_package(Threads REQUIRED)
find_library(libdl NAMES dl REQUIRED)

# DRL provides a cassie library pre-compiled and shipped
# with the repository.
add_library(AgilityCassie INTERFACE)
find_library(agility_cassie_lib_
  NAMES agilitycassie
  HINTS "${cassie_mujoco_dir}/external/agilitycassie/lib"
        "${cassie_mujoco_dir}/src"
  REQUIRED)
target_link_libraries(AgilityCassie INTERFACE
  ${agility_cassie_lib_})
target_include_directories(AgilityCassie INTERFACE 
  "${cassie_mujoco_dir}/external/agilitycassie/include"
  "${cassie_mujoco_dir}/include")

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
add_subdirectory(third_party/pybind11)
include_directories(include)

# -------------------------------------------------
# Targets
# -------------------------------------------------
include(cmake/Targets.cmake)

add_subdirectory(src)
add_subdirectory(example)
add_subdirectory(python)
