import sys
from pathlib import Path

import cmake_build_extension

from setuptools import setup

# The information here can also be placed in setup.cfg - better separation of
# logic and declaration, and simpler if you include description/version in a file.
setup(
    ext_modules=[
        cmake_build_extension.CMakeExtension(
            # This could be anything you like, it is used to create build folders
            name="CassieMujocoSimPythonBindings",
            # Name of the resulting package name (import cassie_mujoco_swig)
            install_prefix="cassie_mujoco_sim",
            expose_binaries=[],
            # Writes the content to the top-level __init__.py
            # write_top_level_init=init_py,
            # Selects the folder where the main CMakeLists.txt is stored
            # (it could be a subfolder)
            source_dir=str((Path(__file__).parent / "..").absolute()),
            cmake_configure_options=[
                # This option points CMake to the right Python interpreter, and helps
                # the logic of FindPython3.cmake to find the active version
                f"-DPython3_ROOT_DIR={Path(sys.prefix)}",
                "-DCALL_FROM_SETUP_PY:BOOL=ON",
                "-DBUILD_SHARED_LIBS:BOOL=ON",
            ]
        )
    ],
    cmdclass=dict(
        # Enable the CMakeExtension entries defined above
        build_ext=cmake_build_extension.BuildExtension,
        # If the setup.py or setup.cfg are in a subfolder wrt the main CMakeLists.txt,
        # you can use the following custom command to create the source distribution.
        sdist=cmake_build_extension.GitSdistFolder
    ),
)
