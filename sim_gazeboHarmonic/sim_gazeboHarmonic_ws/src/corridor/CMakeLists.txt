cmake_minimum_required(VERSION 3.10.2)
project(corridor)

# Requerido para paquetes ROS2
find_package(ament_cmake REQUIRED)

# Buscar la librería gz-transport
find_package(gz-transport13 QUIET REQUIRED OPTIONAL_COMPONENTS log)
set(GZ_TRANSPORT_VER ${gz-transport13_VERSION_MAJOR})

# Incluir directorios
include_directories(${CMAKE_BINARY_DIR})

# Añadir el ejecutable para corridor_follow
if(EXISTS "${CMAKE_SOURCE_DIR}/src/corridor_follow.cc")
  add_executable(corridor_follow src/corridor_follow.cc)
  target_link_libraries(corridor_follow gz-transport${GZ_TRANSPORT_VER}::core)
  install(TARGETS corridor_follow
    DESTINATION lib/${PROJECT_NAME}
  )
endif()

ament_package()

