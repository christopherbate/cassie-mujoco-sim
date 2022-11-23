add_library(GLFWDispatch glfw_dispatch.cc)
target_link_libraries(GLFWDispatch PUBLIC 
  ${GLFW_LIBRARIES})
target_include_directories(GLFWDispatch PUBLIC
  ${CMAKE_CURRENT_LIST_DIR}/../..)