cmake_minimum_required (VERSION 2.8)
project (RoboCalc_Autogen)

add_executable(MainExe main.cpp ctrl0.h ctrl1.h ctrl2.h ctrl3.h RP1.h)
add_executable(MainExeInteractive main.cpp ctrl0.h ctrl1.h ctrl2.h ctrl3.h RP1.h)
target_compile_definitions(MainExeInteractive PUBLIC -DROBOCALC_INTERACTIVE)

include(CMakeCustom.cmake OPTIONAL)
