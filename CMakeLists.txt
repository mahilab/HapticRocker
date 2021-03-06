cmake_minimum_required(VERSION 3.13)

# create your C++ project
project(MyProject VERSION 0.1.0 LANGUAGES CXX)

# find MEL::MEL and all available MEL::xxx modules
find_package(MEL REQUIRED)
find_package(Eigen3 REQUIRED)

# add your include directories
include_directories("include")

include_directories("C:/Git/MEII/include")
link_directories("C:/Git/MEII/lib")

# create an app from your source files
add_executable(my_app
    "include/MyClass.hpp"
    "src/MyClass.cpp"
    "src/my_app.cpp"
)

# link your app to MEL libaries
target_link_libraries(my_app 
    MEL::MEL          # common MEL classes available on all systems
    MEL::quanser      # needed if using MEL Quanser classes
    MEII
    Eigen3::Eigen
    # MEL::myo        # needed if using MEL Myo classes
    # MEL::myrio      # needed if using MEL NI myRIO classes
)
