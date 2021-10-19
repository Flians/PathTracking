set(CMAKE_CXX_COMPILER "g++")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -g -DDEBUG -DGLUT_DISABLE_ATEXIT_HACK -L./LG -lopengl32 -lglu32 -lglut32")

set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories("./LG")

file(GLOB SOURCES "*.cpp")

add_executable(CrowdNavigation ${SOURCES})