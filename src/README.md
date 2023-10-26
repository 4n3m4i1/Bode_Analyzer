# C Source




## Build Instructions
For RP2040 code ensure:
> pico_sdk_import.cmake is in the directory
> CMakeLists.txt is in the directory
> Create a `build` folder
> cd into `build`, run `cmake ..` on Linux/Mac OR `cmake -G "MinGW Makefiles" ..`
> run `make -j4`, if `make` isn't found on Windows then run `mingw32-make -j4`