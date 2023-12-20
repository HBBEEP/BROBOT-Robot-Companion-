cmake_minimum_required(VERSION 3.8)
project(brobot_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# Install Python executables
install(PROGRAMS
  scripts/brobot_interact.py
  scripts/brobot_face_recognition.py
  scripts/brobot_hand_gesture_recognition.py
  scripts/brobot_speak.py
  scripts/brobot_voice_recognition.py
  scripts/brobot_hand_logic.py
  scripts/brobot_camera.py
  scripts/brobot_current_timer.py
  DESTINATION lib/${PROJECT_NAME}
)


################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  # add directories here
  launch
  model
  scripts
  DESTINATION share/${PROJECT_NAME})

ament_package()
