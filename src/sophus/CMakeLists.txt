cmake_minimum_required(VERSION 3.8)
project(sophus)

set(CMAKE_BUILD_TYPE "Release")
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
include_directories(include/sophus)

set(CLASSES so2
            se2
            se3
            so3
            scso3
            sim3
)

set(SOURCES)

foreach(class ${CLASSES})
  list(APPEND SOURCES src/${class}.cpp include/sophus/${class}.h)
endforeach(class)

ament_auto_add_library(${PROJECT_NAME} SHARED
  ${SOURCES}
)

link_libraries(${PROJECT_NAME} ${LIBS})
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

ament_auto_package()
