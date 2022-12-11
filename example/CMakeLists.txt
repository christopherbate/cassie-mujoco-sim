add_cassie_executable(test_terrain SRC
  test_terrain.c

  LINK_PUBLIC
  CassieMujocoSim
)

add_cassie_executable(test_cassie SRC
  cassietest.c

  LINK_PUBLIC
  CassieMujocoSim
)

add_cassie_executable(cassiesim SRC
  cassiesim.c

  LINK_PUBLIC
  CassieMujocoSim
)

add_cassie_executable(cassiectrl SRC
  cassiectrl.c

  LINK_PUBLIC
  CassieMujocoSim
)