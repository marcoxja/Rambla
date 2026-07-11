cmake_minimum_required(VERSION 3.8)
project(rambla_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)

# meshes/generate_bumper_mesh.py is a dev-time generator, not a runtime
# dependency; installing the whole directory just carries the .py alongside
# the committed .stl outputs (harmless - no exec bit, no CMake dependency on
# it running at build time).
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
