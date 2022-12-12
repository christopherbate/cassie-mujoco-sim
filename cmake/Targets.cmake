include(CMakeParseArguments)

function(add_cassie_library lib_name)
  cmake_parse_arguments(ARG "" "ALIAS" "LINK_PUBLIC;SRC" ${ARGN} )
  add_library(${lib_name} SHARED ${ARG_SRC})
  target_link_libraries(${lib_name} PUBLIC
    ${ARG_LINK_PUBLIC})
  target_include_directories(${lib_name} PRIVATE 
    $<BUILD_INTERFACE:${crl_source_dir}/src>)
  target_include_directories(${lib_name} PUBLIC
    $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>
    $<BUILD_INTERFACE:${crl_source_dir}/src>)
  add_library(${ARG_ALIAS} ALIAS ${lib_name})
endfunction()

function(add_cassie_executable lib_name)
  cmake_parse_arguments(ARG "" "" "LINK_PUBLIC;SRC" ${ARGN} )
  add_executable(${lib_name} ${ARG_SRC})
  target_link_libraries(${lib_name} PUBLIC
    ${ARG_LINK_PUBLIC})
  target_include_directories(${lib_name} PRIVATE 
  $<BUILD_INTERFACE:${crl_source_dir}/src>)
endfunction()