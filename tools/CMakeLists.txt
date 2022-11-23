# add_executable(cassiectrl cassiectrl.c)
# target_link_libraries(cassiectrl PRIVATE
#   cassie_mujoco_sim)

add_executable(cassie-sim cassie_sim.cpp)
target_link_libraries(cassie-sim PRIVATE
  cassie_mujoco_sim)
  