cmake_minimum_required(VERSION 2.6)
project(REGISTRATION)

#There are lots of scripts with cmake
#for finding external libraries. 
#see /usr/local/share/cmake-2.6/Modules/Find*.cmake for more examples
find_package(GLUT)
find_package(OpenGL REQUIRED)
find_package(FLTK REQUIRED)
#find_package(PythonLibs REQUIRED)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -stdlib=libstdc++") 
set(CMAKE_CXX_FLAGS "-g -Wall")
add_subdirectory(src)
