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
        self.requires("fmt/10.2.1", override=True)
        self.requires("foxglove-websocket/1.4.0", transitive_headers=True)
        self.requires("foxglove-schemas-protobuf/0.25.1", transitive_headers=True)
        self.requires("protobuf/5.29.3", transitive_headers=True)
        self.requires("boost/1.80.0")
        self.requires("spdlog/1.13.0")
        self.requires("mcap/2.0.2")
        self.requires("dbcppp/3.2.6")
        self.requires("cppzmq/4.11.0")
        self.requires("eigen/3.4.0")
        self.requires("libzip/1.11.4")
        self.requires("libcurl/8.21.0")
        self.requires("openssl/3.6.2")
        self.requires("ouster_sdk/1.0.1")
        
    def build_requirements(self): 
        if not self.settings_build.get_safe("cross_build"):
            self.requires("gtest/1.17.0")
        self.tool_requires("protobuf/5.29.3")
        
    def configure(self):
        self.options["hwloc"].shared = True
        self.options["gtsam"].with_TBB = False
        self.options["gtsam"].support_nested_dissection = False  # drops metis/gklib (breaks ARM cross-compile)
        self.options["ouster_sdk"].build_osf = False
        self.options["ouster_sdk"].build_pcap = False
        self.options["ouster_sdk"].build_mapping = False
