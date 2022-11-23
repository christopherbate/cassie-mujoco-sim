buildDir := $(shell pwd)/build
configure:
	mkdir -p $(buildDir)
	cmake -B $(buildDir) -S . -GNinja

build:
	cmake --build $(buildDir) -j $(shell nproc)

test: build
	cd $(buildDir)/test && ctest --output-on-failure

.PHONY: configure build test