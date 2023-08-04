# Requirements

- [GCC](https://gcc.gnu.org/install/)
- [CMake](https://cmake.org/)

# Compilation

If using a IDE or code editor, the included build options are sufficient to compile the project

Manually, a few steps are required. In a terminal in the project root input the following commands:
- `cmake -H. -Bcompiled -DCMAKE_BUILD_TYPE=Release` (This creates a folder called compiled with build type release)
- `cmake --build compiled/ --target all` (This builds the project inside the compiled folder from above)

The compiled binaries will be found inside compiled

# Execution

After compilation, one can test if the library is functioning correctly via the included test programs. 

For example, from the root directory, inputting `./compiled/test/hot_potato` into the command line will execute the included hot potato test

Additional tests to ensure correct funtionality of the library can be added into the folder and executed similarly