add_executable(
  test_terrain test_terrain.c
)
target_link_libraries(test_terrain PRIVATE
  CassieMujocoSim
)

add_executable(
  test_cassie cassietest.c
)
target_link_libraries(test_cassie PRIVATE
  CassieMujocoSim
)

add_executable(
  cassiesim cassiesim.c
)
target_link_libraries(cassiesim PRIVATE
  CassieMujocoSim
)

add_executable(cassiectrl cassiectrl.c)
target_link_libraries(cassiectrl PRIVATE
  CassieMujocoSim
)