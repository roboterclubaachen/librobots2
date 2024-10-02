#!/bin/bash

set -x

scons --version
python --version
python3 --version
which arm-none-eabi-g++
arm-none-eabi-g++ --version
lbuild --version
lbuild -h | head -n1

set -e

pwd
ls -ahl

for i in ./*/project.xml
do
    pushd $(dirname $i);
    pwd
    ls -ahl
    lbuild build
    scons
    popd
done
