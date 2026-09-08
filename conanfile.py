from conan import ConanFile
from conan.tools.cmake import CMakeToolchain, CMake, cmake_layout
from conan.tools.build import cross_building
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
        self.requires("foxglove-schemas-protobuf/0.25.1", transitive_headers=True)
        self.requires("protobuf/5.29.3", transitive_headers=True)
        self.requires("boost/1.84.0", override=True)
        self.requires("spdlog/1.15.3")
        self.requires("mcap/2.0.2")
        self.requires("dbcppp/3.2.6")
        self.requires("gtsam/4.2.1")
        self.requires("cppzmq/4.11.0")
        self.requires("aravis/0.8.33")
        self.requires("opencv/4.14.0", options={
            "shared": False,
            "imgproc": True,
            "imgcodecs": True,
            "with_jpeg": "libjpeg-turbo",
            **dict.fromkeys((
                "calib3d", "dnn", "features2d", "flann", "gapi", "highgui",
                "ml", "objdetect", "photo", "stitching", "video", "videoio",
                "with_eigen", "with_cuda", "with_opencl", "with_png",
                "with_tiff", "with_jpeg2000", "with_openexr", "with_webp",
            ), False),
        })

    def build_requirements(self): 
        if not cross_building(self):
            self.requires("gtest/1.17.0")
        self.tool_requires("protobuf/5.29.3")

    def configure(self):
        # FLIR cameras use GigE; avoid libusb's system libudev dependency.
        self.options["aravis"].usb = False
        self.options["hwloc"].shared = True
        self.options["gtsam"].with_TBB = False
        self.options["gtsam"].support_nested_dissection = False  # drops metis/gklib (breaks ARM cross-compile)

        # TODO: we will need onettb at some point for fast parallelization
