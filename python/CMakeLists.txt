find_package(Python3 COMPONENTS Interpreter REQUIRED)
# Handle where to install the resulting Python package
if(CALL_FROM_SETUP_PY)
    # The CMakeExtension will set CMAKE_INSTALL_PREFIX to the root
    # of the resulting wheel archive
    set(CASSIE_MUJOCO_SIM_INSTALL_PREFIX ${CMAKE_INSTALL_PREFIX})
else()
    # The Python package is installed directly in the folder of the
    # detected interpreter (system, user, or virtualenv)
    set(CASSIE_MUJOCO_SIM_INSTALL_PREFIX ${Python3_SITELIB})
endif()

find_program(clang2py
  NAMES clang2py
  REQUIRED
)

set(_headers
  cassie_core_sim.h
  cassie_in_t.h
  cassiemujoco.h
  cassie_mujoco_sim.h
  cassie_out_t.h
  cassie_user_in_t.h
  pd_input.h
  pd_in_t.h
  state_output.h
  state_out_t.h
  udp.h)

list(TRANSFORM _headers PREPEND "${CMAKE_CURRENT_LIST_DIR}/../include/")

set(_ctypes_filename cassiemujoco_ctypes.py)

# Generate the `ctypes` output variable.
add_custom_command(OUTPUT "${_ctypes_filename}"
  COMMAND ${clang2py} ${_headers}
          -l $<TARGET_FILE:CassieMujocoSim> -o "${_ctypes_filename}"  
)

set(_tgt_path "")

add_custom_target(cassie_mujoco_python ALL
  # Correct the library path in the generated ctypes file.
  COMMAND sed -i "\"s|'$<TARGET_FILE:CassieMujocoSim>'|'%s/$<TARGET_FILE_NAME:CassieMujocoSim>'%_file_path|g\""
          "${CMAKE_CURRENT_BINARY_DIR}/${_ctypes_filename}"
  COMMAND sed -i "\"s|# LONGDOUBLE_SIZE is: 16|import os\\n_file_path=os.path.join(os.path.dirname(os.path.realpath(__file__)),'lib')|g\""
          "${CMAKE_CURRENT_BINARY_DIR}/${_ctypes_filename}"
  DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/${_ctypes_filename}  
)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/${_ctypes_filename}
  DESTINATION ${CASSIE_MUJOCO_SIM_INSTALL_PREFIX}
)


# Install all model data if doing Python packaging.
if(CALL_FROM_SETUP_PY)
  install(DIRECTORY
    ${CMAKE_CURRENT_LIST_DIR}/../model
    DESTINATION ${CASSIE_MUJOCO_SIM_INSTALL_PREFIX}
  )
endif()