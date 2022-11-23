#!/bin/bash -xe

if [[ -z ${VIRTUAL_ENV} ]] && [[ -z ${CONDA_DEFAULT_ENV} ]]; then
  echo "This script must be run from within a Python virtual environment"
  exit 1
fi

package_dir="$(cd -- "$(dirname "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
readonly tmp_dir="$(mktemp -d)"

python -m pip install --upgrade pip setuptools
python -m pip install absl-py
pushd ${tmp_dir}
cp -r "${package_dir}"/* .

# Generate header files.
old_pythonpath="${PYTHONPATH}"
if [[ "$(uname)" == CYGWIN* || "$(uname)" == MINGW* ]]; then
  export PYTHONPATH="${old_pythonpath};${package_dir}/.."
else
  export PYTHONPATH="${old_pythonpath}:${package_dir}/.."
fi
# python "${package_dir}"/mujoco/codegen/generate_enum_traits.py > \
#     mujoco/enum_traits.h
# python "${package_dir}"/mujoco/codegen/generate_function_traits.py > \
#     mujoco/function_traits.h
export PYTHONPATH="${old_pythonpath}"

# Copy over CMake scripts.
mkdir cmake
# cp "${package_dir}"/../cmake/*.cmake cmake

python setup.py sdist --formats=gztar
tree -L 2 .
tar -tf dist/cassiemujoco-*.tar.gz || true
popd

mkdir -p "${package_dir}"/dist
mv "${tmp_dir}"/dist/* "${package_dir}"/dist
