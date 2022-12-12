buildDir := $(shell pwd)/build
configure:
	mkdir -p $(buildDir)
	cmake -B $(buildDir) -S . -GNinja \
		-DBUILD_SHARED_LIBS=ON \
		-DUSE_SANITIZER=Address

build:
	cmake --build $(buildDir) -j $(shell nproc)

test: build
	cd $(buildDir)/test && ctest --output-on-failure

.ONESHELL:
wheel:
	cd python
	rm -rf build dist | true
	python setup.py bdist_wheel
	cd dist && unzip *.whl
	pip install *.whl --no-deps --force-reinstall

.PHONY: configure build test wheel
