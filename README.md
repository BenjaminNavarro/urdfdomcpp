# URDFDOMCPP

This library is a partial rewrite of [urdfdom](https://github.com/ros/urdfdom) and [urdfdom_headers](https://github.com/ros/urdfdom_headers) without any ROS code and upgraded to [TinyXML2](https://github.com/leethomason/tinyxml2). It is intended to simplify the parsing of URDF files outside of a ROS environment.

## Installing

urdfdomcpp can either be installed using system dependencies or as a Conan package.

The former allows a traditional CMake build and install process while the later aims at automating everything.

### System install

The only dependency is TinyXML2, which must installed on your system in order for CMake to find it. If you don't have it already, you can run `apt install libtinyxml2-dev` on Ubuntu or `pacman -S tinyxml2` on Archlinux. For other systems, check your package package manager or install it from source.

Then all you have to do is download the sources and run:
```bash
cd urdfdomcpp/build
cmake -DCMAKE_BUILD_TYPE=Release .. # optionally pass -DCMAKE_INSTALL_PREFIX=/your/install/path
cmake --build . --parallel          # remove --parallel for single core compilation
cmake --build . --target install    # use sudo if necessary
```

You can then use the library in a CMake project by adding `find_package(urdfdomcpp)` to your *CMakeLists.txt* and linking your targets with `urdfdomcpp` (e.g `target_link_libraries(my_app PUBLIC urdfdomcpp`)

### Conan package

Before anything else, if you use GCC > 5.1 please make sure that your Conan default profile (*~/.conan/profiles/default*) or the one you use to build this package has `compiler.libcxx=libstdc++11` instead of `compiler.libcxx=libstdc++`. This will enable the C++11 ABI, which is the default for GCC starting with version 5.2. Failing to do so will most likely lead to link time errors in the form of `undefined reference to 'urdf::parseURDF(std::__cxx11::basic_string<char, std::char_traits<char>, std::allocator<char> > const&)'` when consuming the library.


To get the Conan package from [Bintray](https://bintray.com/benjaminnavarro/bnavarro/urdfdomcpp%3Abnavarro), simply add a new remote:
```
conan remote add bnavarro https://api.bintray.com/conan/benjaminnavarro/bnavarro
```
And then add `urdfdomcpp/0.1@bnavarro/stable` in your Conan dependencies.

When running `conan install` in your project, you might need to pass `--build=missing` in order to build urdfdomcpp and its dependencies from sources in the case there are no precompiled binaries matching your platform.