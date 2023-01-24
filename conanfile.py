# /usr/bin/python3
import os
from conans import ConanFile, CMake, tools


class TraactPackage(ConanFile):
    python_requires = "traact_run_env/1.0.0@traact/latest"
    python_requires_extend = "traact_run_env.TraactPackageCmake"

    name = "traact_vision"
    description = "Image datatype and vision functions for traact using opencv"
    url = "https://github.com/traact/traact_vision.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "tests/*", "CMakeLists.txt"

    def _options(self):
        self.options["with_cuda"] = [True, False]
        self.default_options["with_cuda"] = True

    def requirements(self):
        self.traact_requires("traact_spatial", "latest")
        self.requires("cuda_dev_config/[>=2.0]@camposs/stable")
        self.requires("opencv/4.5.5@camposs/stable")
        if self.options.with_tests:
            self.requires("gtest/cci.20210126")

    def configure(self):
        self.options['opencv'].shared = self.options.shared
        self.options['opencv'].with_cuda = self.options.with_cuda
        # self.options['opencv'].with_tbb = True
