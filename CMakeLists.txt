cmake_minimum_required(VERSION 3.1 FATAL_ERROR)
project(Monocle)

# Enable C++11
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# YouCompleteMe
set( CMAKE_EXPORT_COMPILE_COMMANDS ON )

# stb
set (stb src/vendor/stb_image/stb_image.cpp)
set (source_dir ${PROJECT_SOURCE_DIR}/src)
set (include_dir ${PROJECT_SOURCE_DIR}/include)
# a.out
add_executable(a.out src/main.cpp include/Reinitializer.cpp ${stb})
target_include_directories(a.out PRIVATE ${source_dir} ${include_dir})
target_include_directories(a.out PRIVATE src/vendor)

# RealSense
find_package(realsense2 REQUIRED)
target_link_libraries(a.out ${realsense2_LIBRARY})

# PyTorch
find_package(Torch REQUIRED)
target_link_libraries(a.out ${TORCH_LIBRARIES})
message(${TORCH_LIBRARIES})

# GLFW
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)
include_directories(${GLFW_INCLUDE_DIRS})
target_link_libraries(a.out ${OPENGL_gl_LIBRARY})
target_link_libraries(a.out ${GLFW_STATIC_LIBRARIES})
 
# OpenGL/Glut
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
include_directories( ${OPENGL_INCLUDE_DIRS}  ${GLUT_INCLUDE_DIRS} )

target_link_libraries(a.out ${OPENGL_LIBRARIES} ${GLUT_LIBRARY} )

# GLEW
find_package(GLEW REQUIRED)
if (GLEW_FOUND)
    include_directories(${GLEW_INCLUDE_DIRS})
    target_link_libraries(a.out ${GLEW_LIBRARIES})
endif()

# AssImp
find_package(ASSIMP REQUIRED)
include_directories(${ASSIMP_INCLUDE_DIRS})
target_link_libraries(a.out ${ASSIMP_LIBRARIES})

# OpenMP
find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# OpenCV
find_package( OpenCV REQUIRED )
include_directories(${OpenCV_INCLUDE_DIRS})
target_link_libraries(a.out ${OpenCV_LIBS})

# target_include_directories(a.out ${OPENGL_INCLUDE_DIR})

