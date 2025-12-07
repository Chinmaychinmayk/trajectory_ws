# CMake generated Testfile for 
# Source directory: /home/newuser/trajectory_ws/src/path_smoothing_control
# Build directory: /home/newuser/trajectory_ws/build/path_smoothing_control
# 
# This file includes the relevant testing commands required for 
# testing this directory and lists subdirectories to be tested as well.
add_test(test_controller "/usr/bin/python3" "-u" "/opt/ros/humble/share/ament_cmake_test/cmake/run_test.py" "/home/newuser/trajectory_ws/build/path_smoothing_control/test_results/path_smoothing_control/test_controller.gtest.xml" "--package-name" "path_smoothing_control" "--output-file" "/home/newuser/trajectory_ws/build/path_smoothing_control/ament_cmake_gtest/test_controller.txt" "--command" "/home/newuser/trajectory_ws/build/path_smoothing_control/test_controller" "--gtest_output=xml:/home/newuser/trajectory_ws/build/path_smoothing_control/test_results/path_smoothing_control/test_controller.gtest.xml")
set_tests_properties(test_controller PROPERTIES  LABELS "gtest" REQUIRED_FILES "/home/newuser/trajectory_ws/build/path_smoothing_control/test_controller" TIMEOUT "60" WORKING_DIRECTORY "/home/newuser/trajectory_ws/build/path_smoothing_control" _BACKTRACE_TRIPLES "/opt/ros/humble/share/ament_cmake_test/cmake/ament_add_test.cmake;125;add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest_test.cmake;86;ament_add_test;/opt/ros/humble/share/ament_cmake_gtest/cmake/ament_add_gtest.cmake;93;ament_add_gtest_test;/home/newuser/trajectory_ws/src/path_smoothing_control/CMakeLists.txt;55;ament_add_gtest;/home/newuser/trajectory_ws/src/path_smoothing_control/CMakeLists.txt;0;")
subdirs("gtest")
