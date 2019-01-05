#! /bin/bash

# this script runs the default build and generates some info for clang-tidy
cmake -E make_directory build
cmake -E chdir build cmake ..
cmake --build build/ --config RelWithDebInfo --clean-first -- -j12
BUILD_RESULT=$?
if [ $BUILD_RESULT != 0 ]
then
    exit $BUILD_RESULT
fi
cmake -DCMAKE_EXPORT_COMPILE_COMMANDS=ON build