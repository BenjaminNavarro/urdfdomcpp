# URDFDOMCPP

This library is a partial rewrite of [urdfdom](https://github.com/ros/urdfdom) and [urdfdom_headers](https://github.com/ros/urdfdom_headers) without any ROS code and upgraded to [TinyXML2](https://github.com/leethomason/tinyxml2). It is intended to simplify the parsing of URDF files outside of a ROS environment.

## Getting started

urdfdomcpp is packaged using [Conan](https://conan.io/).

If your Conan version is lower than 2.0, it is preferred to set the environment variable `CONAN_V2_MODE` to `1` (e.g `export CONAN_V2_MODE=1`). More on that [here](https://docs.conan.io/en/latest/reference/conan_v2_mode.html).

In order to use urdfdomcpp in your Conan project add the associated remote:
```
conan remote add bnavarro https://api.bintray.com/conan/benjaminnavarro/bnavarro
```
And then add `urdfdomcpp/0.1@bnavarro/stable` to your Conan dependencies.