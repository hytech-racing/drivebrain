from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from os import path

class DrivebrainSoftware(ConanFile):
    name = "_foxglove"
    version = "1.0.0-dev"
    license = "zlib"
    settings = ["os", "compiler", "build_type", "arch"]
    exports_sources = "CMakeLists.txt", "LICENSE", "src/*", "include/*"
    generators = "CMakeDeps", "CMakeToolchain"
    exports = '*'


    def build(self):
        cmake = CMake(self)
        cmake.build()
        cmake.install()

    def requirements(self): 
        self.requires("foxglove-websocket/1.4.0", transitive_headers=True)
        self.requires("protobuf/5.29.3", transitive_headers=True)
        self.requires("boost/1.80.0")
        self.requires("spdlog/1.15.3")
        self.requires("mcap/2.0.2")
        self.requires("dbcppp/3.2.6")
        self.requires("cppzmq/4.11.0")
        self.requires("eigen/3.4.0")
        self.requires("libzip/1.11.4")
        self.requires("libcurl/8.21.0")
        self.requires("openssl/3.6.2")
        
    def build_requirements(self): 
        if not self.settings_build.get_safe("cross_build"):
            self.requires("gtest/1.17.0")
        self.tool_requires("protobuf/5.29.3")
