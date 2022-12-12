add_cassie_library(CassieMujocoSim
  SRC cassiemujoco.c udp.c
  ALIAS CassieMujoco::Sim
  LINK_PUBLIC AgilityCassie                            
              mujoco::mujoco
              glfw     
              Threads::Threads
)

# Link in the whole agility cassie library. 
# The math and dl libraries must come after it on the command line.
target_link_libraries(CassieMujocoSim PUBLIC
  -Wl,--whole-archive
  ${agility_cassie_lib_}
  -Wl,--no-whole-archive
  m dl
)

install(
  TARGETS CassieMujocoSim
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
  PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}
)