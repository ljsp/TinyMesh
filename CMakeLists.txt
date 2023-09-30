cmake_minimum_required (VERSION 3.1)

project(AppTinyMesh)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(Qt6_DIR "/usr/local/Qt/6.5.2/gcc_64/lib/cmake/Qt6")
set(Qt6GuiTools_DIR "/usr/local/Qt/6.5.2/gcc_64/lib/cmake/Qt6GuiTools")

find_package(Qt6 REQUIRED COMPONENTS Core Widgets Gui OpenGL OpenGLWidgets)
if (Qt6Widgets_FOUND)
    if (Qt6Widgets_VERSION VERSION_LESS 6.3.0)
        message(FATAL_ERROR "Minimum Qt version is 6.3.0")
    endif()
endif()

# Add dependencies
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

find_package(GLEW REQUIRED)

# ------------------------------------------------------------------------------
include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(AppTinyMesh/Source)
include_directories(AppTinyMesh/Include)
include_directories(AppTinyMesh/UI)

set(SRC_DIR ../AppTinyMesh/Source)
set(INC_DIR ../AppTinyMesh/Include)
set(UI_DIR ../AppTinyMesh/UI)
set(SOURCES
        ${SRC_DIR}/box.cpp
        ${SRC_DIR}/camera.cpp
        ${SRC_DIR}/evector.cpp
        ${SRC_DIR}/implicits.cpp
        ${SRC_DIR}/main.cpp
        ${SRC_DIR}/mesh.cpp
        ${SRC_DIR}/meshcolor.cpp
        ${SRC_DIR}/mesh-widget.cpp
        ${SRC_DIR}/qtemainwindow.cpp
        ${SRC_DIR}/ray.cpp
        ${SRC_DIR}/shader-api.cpp
        ${SRC_DIR}/triangle.cpp
        ${SRC_DIR}/cone.cpp
        ${SRC_DIR}/cylinder.cpp
        ${SRC_DIR}/sphere.cpp
        ${SRC_DIR}/torus.cpp
        ${SRC_DIR}/disc.cpp
        ${SRC_DIR}/face.cpp
        ${SRC_DIR}/matrix3.cpp
        ${SRC_DIR}/pilule.cpp
        ${SRC_DIR}/planet.cpp
        ${SRC_DIR}/terrain.cpp
        #${SRC_DIR}/Node.cpp
        #${SRC_DIR}/Primitives.cpp
)

set(HEADERS
        ${INC_DIR}/box.h
        ${INC_DIR}/camera.h
        ${INC_DIR}/color.h
        ${INC_DIR}/implicits.h
        ${INC_DIR}/mathematics.h
        ${INC_DIR}/mesh.h
        ${INC_DIR}/meshcolor.h
        ${INC_DIR}/qte.h
        ${INC_DIR}/ray.h
        ${INC_DIR}/shader-api.h
        ${INC_DIR}/realtime.h
        ${INC_DIR}/cone.h
        ${INC_DIR}/cylinder.h
        ${INC_DIR}/sphere.h
        ${INC_DIR}/torus.h
        ${INC_DIR}/disc.h
        ${INC_DIR}/face.h
        ${INC_DIR}/matrix3.h
        ${INC_DIR}/pilule.h
        ${INC_DIR}/planet.h
        ${INC_DIR}/terrain.h
        #${INC_DIR}/Node.h
        #${INC_DIR}/Primitives.h
)

set(FORMS
        ${SRC_DIR}/interface.ui
)

qt_add_executable(${PROJECT_NAME}
        MANUAL_FINALIZATION
        ${SOURCES}
        ${HEADERS}
        ${FORMS}
)


target_include_directories(${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})

target_link_libraries(${PROJECT_NAME} PRIVATE
        ${GLEW_LIBRARIES}
        GLU
        glut
        Qt6::Core
        Qt6::Widgets
        Qt6::Gui
        Qt6::OpenGL
        Qt6::OpenGLWidgets
)

set_target_properties(${PROJECT_NAME} PROPERTIES RUNTIME_OUTPUT_DIRECTORY_DEBUG ${CMAKE_CURRENT_BINARY_DIR})

qt_finalize_target(${PROJECT_NAME})

# shader folder copy on post build (all platforms)
#set(DATA_DIR ../AppTinyMesh/Shaders)
#add_custom_command(
#    TARGET ${PROJECT_NAME} POST_BUILD
#    COMMAND ${CMAKE_COMMAND} -E copy_directory
#            ${CMAKE_SOURCE_DIR}/${DATA_DIR}
#            ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/Shaders
#)
