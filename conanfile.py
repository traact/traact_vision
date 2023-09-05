# /usr/bin/python3
import os
from conan import ConanFile


class TraactPackage(ConanFile):
    python_requires = "traact_base/0.0.0@traact/latest"
    python_requires_extend = "traact_base.TraactPackageCmake"

    name = "traact_vision"
    version = "0.0.0"
    description = "Image datatype and vision functions for traact using opencv"
    url = "https://github.com/traact/traact_vision.git"
    license = "MIT"
    author = "Frieder Pankratz"

    settings = "os", "compiler", "build_type", "arch"
    compiler = "cppstd"

    exports_sources = "src/*", "include/*", "tests/*", "CMakeLists.txt"

    options = {
        "shared": [True, False],
        "with_tests": [True, False],
        "trace_logs_in_release": [True, False],
        "with_cuda": [True, False]
    }

    default_options = {
        "shared": True,
        "with_tests": True,
        "trace_logs_in_release": True,
        "with_cuda" : True,
        "opencv/*:with_jpeg": "libjpeg-turbo",
        "opencv/*:with_quirc": False,
        "libtiff/*:jpeg": "libjpeg-turbo"

    }    
    

    def requirements(self):
        self.requires("traact_base/0.0.0@traact/latest")
        self.requires("traact_core/0.0.0@traact/latest")
        self.requires("traact_spatial/0.0.0@traact/latest", transitive_headers=True, transitive_libs=True)        
        self.requires("opencv/4.8.0@camposs/stable", transitive_headers=True, transitive_libs=True)
        self.requires("libwebp/1.3.1", override=True)        

    def configure(self):
        self.options['opencv'].shared = self.options.shared
        self.options['opencv'].with_cuda = self.options.with_cuda
        # self.options['opencv'].with_tbb = True
        if self.settings.os == "Linux":            
            self.options['opencv/*'].with_gtk = True            

    def _after_package_info(self):
        self.cpp_info.libs = ["traact_vision"]