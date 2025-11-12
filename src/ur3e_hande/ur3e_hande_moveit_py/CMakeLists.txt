cmake_minimum_required(VERSION 3.8)
project(ur3e_hande_moveit_py)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(robotiq_hande_description REQUIRED)
find_package(gripper_action REQUIRED)
find_package(pymoveit2 REQUIRED)

install(DIRECTORY
  launch scripts
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS 
          scripts/pnp_demo.py 
          scripts/pnp_demo_p.py 
          scripts/pnp_demo_sim.py
   DESTINATION lib/${PROJECT_NAME}
 )

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
