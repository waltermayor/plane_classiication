# especificar la minima version de cmake
cmake_minimum_required(VERSION 2.8)

# nombre del proyecto puede ser de una carpeta o un el archivo
# todo depende de como se desee organizar

project(segmentation)

#set(HEADER_FILES Registration.h Registration.cpp)

# configurar la ruta donde se desea crear los programas
set (CMAKE_CXX_STANDARD 11)  # funciono para solucionar problema de nullptr pero compilo con un problema 
set(CMAKE_BINARY_DIR ${CMAKE_SOURCE_DIR}/bin)
set(EXECUTABLE_OUTPUT_PATH ${CMAKE_BINARY_DIR})
set(LIBRARY_OUTPUT_PATH ${CMAKE_BINARY_DIR})

# especificar las librerias que se van a usar
# en este caso se estan especificando las librerias de pcl
find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# El siguente folder será incluido
include_directories("${PROJECT_SOURCE_DIR}")


#definir ruta y nombre del ejecutable
add_executable (classificationMap ${PROJECT_SOURCE_DIR}/clasificationMap.cpp ${HEADER_FILES})

# adicionar librerias al ejecutable

target_link_libraries (classificationMap ${PCL_LIBRARIES})

