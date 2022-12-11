add_cassie_library(CassieMujocoSim
  SRC cassiemujoco.c udp.c
  LINK_PUBLIC AgilityCassie                            
              mujoco::mujoco
              glfw     
              Threads::Threads
              dl m
)

target_link_libraries(CassieMujocoSim PUBLIC
  -Wl,--whole-archive
  ${agility_cassie_lib_}
  -Wl,--no-whole-archive
)
