cmake_minimum_required(VERSION 3.8)
project(rbt08pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Legacy source headers have not been assigned a project-wide license yet.
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  urdf
  meshes
  launch
  config
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/grasp_manager.py
  scripts/spawn_planning_scene.py
  scripts/pear_pose_source.py
  scripts/pick_place_manager.py
  scripts/moveit_smoke_test.py
  scripts/camera_pipeline_check.py
  scripts/pear_detector.py
  scripts/pear_surface_estimator.py
  scripts/pear_world_transformer.py
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES
  scripts/pear_segmentation.py
  scripts/pear_depth.py
  scripts/pear_center.py
  scripts/orchard_randomization.py
  scripts/trajectory_checks.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
