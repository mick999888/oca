# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/home/mkarl/esp/esp-idf/components/bootloader/subproject"
  "/home/mkarl/projects/oca/build/bootloader"
  "/home/mkarl/projects/oca/build/bootloader-prefix"
  "/home/mkarl/projects/oca/build/bootloader-prefix/tmp"
  "/home/mkarl/projects/oca/build/bootloader-prefix/src/bootloader-stamp"
  "/home/mkarl/projects/oca/build/bootloader-prefix/src"
  "/home/mkarl/projects/oca/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/home/mkarl/projects/oca/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/home/mkarl/projects/oca/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
