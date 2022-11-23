enable_testing()

# add_cassie_executable(cassietest SRC 
#   cassietest.c
  
#   LINK_PUBLIC
#   cassie_mujoco_sim
#   cassie_mujoco_sim_visualization)

# add_cassie_executable(cassie_hfield_test SRC 
#   test_hfield.c
  
#   LINK_PUBLIC
#   cassie_mujoco_sim
#   cassie_mujoco_sim_visualization)

add_cassie_executable(test_initialization SRC 
  test_initialization.cpp
  
  LINK_PUBLIC
  cassie_mujoco_sim)  

add_cassie_executable(test_renderer SRC 
  test_renderer.cpp
  
  LINK_PUBLIC
  cassie_mujoco_sim)
  

add_test(NAME TestInitialization COMMAND test_initialization)
add_test(NAME TestRenderer COMMAND test_renderer)
# add_test(NAME CassieHfieldTest COMMAND cassie_hfield_test)