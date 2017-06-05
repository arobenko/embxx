# Overview
Embxx is Embedded C++ library that is developed with intetion to be used in 
bare metal and Linux based embedded environments. It comes to supplement 
essential functionality that is missing in widely used STL and BOOST C++ 
libraries.

Embxx doesn't use "RTTI" and "exceptions". It makes a significant effort to 
completely eliminate or minimise usage of dynamic memory allocation in order 
to make it usable with embedded system with low memory footprint and/or slow 
CPUs.

# What you need
1. CMake - Mandatory. This library uses cmake to generate Makefiles.
2. gcc v4.7 or greater. Mandatory. This library uses C++11 constructs that may 
   be not fully supported in earlier versions of this compiler.
3. Doxygen - Optional. Used to generate documentation.
4. Boost - Optional. Used in unittesting. 
5. Python - Optional. Used to run cxxtest to perform unittesting

# How to use
1. Create build directory (at any convenient place) and cd there:
   ```
   $> mkdir <embxx_sources>/build
   $> cd <embxx_sources>/build
   ```
2. Generate makefiles using cmake together with the path to embxx sources. 
   If no definitions specified, the makefiles will include compilation of 
   unittests and examples:
   ```
   $> cmake ..
   ```
   To exclude unittests, use -DNO_UNIT_TESTS=1 option:
   ```
   $> cmake -DNO_UNIT_TESTS=1 ..
   ```
   To exclude examples, use -DNO_EXAMPLES=1 option:
   ```
   $> cmake -DNO_EXAMPLES=1
   ```
   To exclude both unittests and examples, use both options:
   ```
   $> cmake -DNO_UNIT_TESTS=1 -DNO_EXAMPLES=1 ..
   ```
3. Compile all the sources and install. The installation root directory is
   <build_dir>/install
   ```
   $> make install
   ```
4. To run the unittests (if such were compiled):
   ```
   $> ctest
   ```
5. Generate doxygen documentation. The documentation will be generated in 
   <build_dir>/doxygen.
   ```
   $> make doxygen 
   ```
6. Add "<build_dir>/install/include" to include search paths for your project 
 
# Using embxx as git submodule
The alternative way to use embxx is simply add its repository as git submodule
in your project, then add <git_embxx_submodule_dir> to your include search paths.

# Current documentation
The latest documentation for the library may also be found at the following link:
[Dropbox](https://www.dropbox.com/s/feknhqmxroxnsi9/doc_embxx.tar.gz) or from
[release artifacts](https://github.com/arobenko/embxx/releases)

# Git branches
"master" - main branch, will always contain latest stable (released) version.
"develop" - current development branch

# Contact information
Author: Alex Robenko
E-mail: arobenko@gmail.com
