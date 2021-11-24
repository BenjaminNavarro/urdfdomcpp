from conans import ConanFile, CMake, tools


class UrdfdomcppConan(ConanFile):
    name = "urdfdomcpp"
    version = "1.0"
    license = "BSD"
    author = "Benjamin Navarro <navarro.benjamin13@gmail.com>"
    url = "https://github.com/BenjaminNavarro/urdfdomcpp"
    description = "A non-ROS version of urdfdom"
    topics = ("c++", "urdf")
    settings = "os", "compiler", "build_type", "arch"
    options = {"shared": [True, False], "fPIC": [
        True, False], "build_tests": [True, False]}
    default_options = {"shared": False, "fPIC": True, "build_tests": False}
    generators = "cmake"
    requires = "tinyxml2/8.0.0"
    # build_requires = "cmake/[>=3.10]"

    def requirements(self):
        if self.options.build_tests:
            self.requires("gtest/1.10.0")
            # self.requires("cppcheck/2.6")

    def configure(self):
        if self.settings.compiler == 'Visual Studio':
            del self.options.fPIC

    def source(self):
        self.run(
            "git clone https://github.com/BenjaminNavarro/urdfdomcpp.git --branch v1.0.0")

    def build(self):
        cmake = CMake(self)
        if self.options.build_tests:
            cmake.definitions["ENABLE_TESTING"] = True
        cmake.configure(source_folder="urdfdomcpp")
        cmake.build()
        if self.options.build_tests:
            cmake.test()
        cmake.install()

    def package(self):
        # self.copy("*.h", dst="include", src="urdfdomcpp/include")
        self.copy("bin/check_urdf", dst="bin", keep_path=False)
        self.copy("bin/urdf_to_graphiz", dst="bin", keep_path=False)
        self.copy("*.lib", dst="lib", keep_path=False)
        self.copy("*.dll", dst="bin", keep_path=False)
        self.copy("*.so", dst="lib", keep_path=False)
        self.copy("*.dylib", dst="lib", keep_path=False)
        self.copy("*.a", dst="lib", keep_path=False)

    def package_info(self):
        self.cpp_info.libs = ["urdfdomcpp"]
