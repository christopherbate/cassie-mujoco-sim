buildDir := $(shell pwd)/build
configure:
	mkdir -p $(buildDir)
	cmake -B $(buildDir) -S . -GNinja -DBUILD_SHARED_LIBS=ON

build:
	cmake --build $(buildDir) -j $(shell nproc)

test: build
	cd $(buildDir)/test && ctest --output-on-failure

.PHONY: configure build test
