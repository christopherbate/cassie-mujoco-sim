add_cassie_library(cassie_mujoco_sim SRC
  cassie.cpp  
  mujoco_context.cpp
  renderer.cpp
  simulation.cpp
  udp.cpp  

  LINK_PUBLIC
  dl
  m
  Threads::Threads
  mujoco::mujoco
  AgilityCassie
  GLFWDispatch)

target_include_directories(cassie_mujoco_sim
  PUBLIC "${cassie_mujoco_dir}/include"
  PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

# Write out mytools_export.h to the current binary directory
include(GenerateExportHeader)
generate_export_header(cassie_mujoco_sim
  EXPORT_FILE_NAME cassie_mujoco_export.h
  EXPORT_MACRO_NAME CASMUJ_PUBLIC)