# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class Traact(ConanFile):
    name = "traact_vision"
    version = "0.1.0"

    description = "Image datatype and vision functions for traact using opencv"
    url = "https://github.com/traact/traact_vision"
    license = "MIT"
    author = "Frieder Pankratz"

    short_paths = True

    generators = "cmake"
    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"
    options = {
        "shared": [True, False],
        "with_cuda": [True, False],
        "with_tests": [True, False]
    }

    default_options = {
        "shared": True,
        "with_cuda": True,
        "with_tests": True
    }

    exports_sources = "src/*", "tests/*", "CMakeLists.txt"

    def requirements(self):

        self.requires("cuda_dev_config/[>=2.0]@camposs/stable")
        self.requires("traact_spatial/[>=0.1.0]@camposs/stable")
        self.requires("opencv/4.5.0@camposs/stable")

        if self.options.with_tests:
            self.requires("gtest/1.10.0")

    def _configure_cmake(self):
        cmake = CMake(self)
        cmake.verbose = True

        def add_cmake_option(option, value):
            var_name = "{}".format(option).upper()
            value_str = "{}".format(value)
            var_value = "ON" if value_str == 'True' else "OFF" if value_str == 'False' else value_str
            cmake.definitions[var_name] = var_value

        for option, value in self.options.items():
            add_cmake_option(option, value)

        cmake.configure()
        return cmake

    def configure(self):
        self.options['traact_core'].shared = self.options.shared
        self.options['opencv'].shared = self.options.shared

        self.options['opencv'].with_cuda = self.options.with_cuda
        if self.settings.os == "Linux":
            self.options['opencv'].with_gtk = True
        # self.options['opencv'].with_tbb = True

    def build(self):
        cmake = self._configure_cmake()
        cmake.build()

    def package(self):
        cmake = self._configure_cmake()
        cmake.install()

    def package_info(self):
        self.cpp_info.libs = [self.name]
