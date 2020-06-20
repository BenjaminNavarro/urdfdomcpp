from conans import ConanFile, CMake, tools


class UrdfdomcppConan(ConanFile):
    name = "urdfdomcpp"
    version = "0.1"
    license = "BSD"
    author = "Benjamin Navarro <navarro.benjamin13@gmail.com>"
    url = "https://github.com/BenjaminNavarro/urdfdomcpp"
    description = "A non-ROS version of urdfdom"
    topics = ("c++", "urdf")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False]}
    default_options = {"shared": False}
    generators = "cmake"
    requires = "tinyxml2/8.0.0"

    def source(self):
        self.run("git clone https://github.com/BenjaminNavarro/urdfdomcpp.git")

    def build(self):
        cmake = CMake(self)
        cmake.definitions["urdfdomcpp_ENABLE_CONAN"] = True
        cmake.definitions["ENABLE_TESTING"] = True
        cmake.configure(source_folder="urdfdomcpp")
        cmake.build()
        cmake.test()
        cmake.install()

    def package(self):
        self.copy("*.h", dst="include", src="urdfdomcpp/include")
        self.copy("*.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.so", dst="lib", keep_path=False)
        self.copy("*.dylib", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["urdfdomcpp"]

