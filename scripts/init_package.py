#! /usr/bin/env python3

import os, math, yaml, sys

# python script to change name of package in package.xml and CMakeLists
package_path = os.path.dirname(os.path.dirname(os.path.realpath(__file__)))
package_name = os.path.split(package_path)
package_name = package_name[1]
# print(package_name)

target_word = "ekf_hsr"
pkg_file_path = package_path + "/package.xml"
cmake_file_path = package_path + "/CMakeLists.txt"
pkg_content = ""
cmake_content = ""

with open(pkg_file_path, 'r') as pkg_file:
    pkg_content = pkg_file.read()
with open(cmake_file_path, 'r') as cmake_file:
    cmake_content = cmake_file.read()


pkg_updated_content = pkg_content.replace(target_word, package_name, 1)
cmake_updated_content = cmake_content.replace(target_word, package_name, 1)
    
with open(pkg_file_path, 'w') as file:
    file.write(pkg_updated_content)
with open(cmake_file_path, 'w') as file:
    file.write(cmake_updated_content)

