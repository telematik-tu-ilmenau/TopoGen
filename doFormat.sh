#!/bin/sh

for srcFile in `find src -name "*.[c|h]pp"`
do
    clang-format -i $srcFile
done
