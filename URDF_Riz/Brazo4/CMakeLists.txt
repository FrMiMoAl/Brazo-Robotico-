cmake_minimum_required(VERSION 3.8)
project(brazo4)

find_package(ament_cmake REQUIRED)

# Install launch files
install(DIRECTORY
  launch
  urdf
  meshes
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install python scripts
install(PROGRAMS
  scripts/control_brazo.py
  scripts/teleop_brazo.py
  scripts/visor_kinect.py
  scripts/visualizador_camara.py
  scripts/mover_coordenadas.py
  scripts/teleop_cartesiano.py
  scripts/mapeador_360.py
  scripts/mapear_espacio.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
