#!/bin/bash
#C++ 编译
dir=$(pwd)
build_dir="${dir}"/build
rm -rf ${build_dir}
mkdir -p ${build_dir}
cd ${build_dir}
cmake ..
make -j8
#编译失败重新编译
 if [ $? -ne 0 ]
 then
 cd "${dir}"/
 rm -rf ${build_dir}
 mkdir -p ${build_dir}
 cd ${build_dir}
 cmake ..
 make
 fi
make package


