#!/bin/sh

for folder in `find . -type d -name "CMakeFiles"`
do
  rm -rf $folder
done

for install_file in `find . -type f -name "cmake_install.cmake"`
do
  rm -rf $install_file
done

for makefile in `find . -type f -name "Makefile"`
do
  rm -rf $makefile
done

for scriptfolder in `find . -type d -name "CMakeScripts"`
do
  rm -rf $scriptfolder
done

rm -rf topoGen.xcodeproj
rm -f CMakeCache.txt
rm -f rules.ninja build.ninja
rm -f .ninja_log
