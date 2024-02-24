# C Source
Please do NOT overwrite any Bandit Template files!!



## Build Instructions
For RP2040 code ensure:
> pico_sdk_import.cmake is in the directory
> CMakeLists.txt is in the directory
> Create a `build` folder
> cd into `build`, run `cmake -DFAMILY=rp2040 ..` on Linux/Mac OR `cmake -G "MinGW Makefiles" -DFAMILY=rp2040 ..`
> run `make -j8`, if `make` isn't found on Windows then run `mingw32-make -j8`
> run with `-j16` options if possible, build is zippy that way :)