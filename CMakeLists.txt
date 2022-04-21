cmake_minimum_required(VERSION 3.4)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/CMake/Modules/")

project(UTAD-SIM03-PBD-Softbody)

add_executable(UTAD-SIM03-PBD-Softbody
    src/Containers/Field3.h
    src/Containers/MemoryPool.h
    src/Containers/MemoryPool.inl
    src/Gfx/Gfx.cpp
    src/Gfx/Gfx.h
    src/Math/Matrix3f.h
    src/Math/Quaternionf.h
    src/Math/Vector2f.h
    src/Math/Vector3f.h
    src/Std/CMath.h
    src/Std/CStdInt.h
    src/Constraint.cpp
    src/Constraint.h
    src/DistanceConstraint.cpp
    src/DistanceConstraint.h
    src/main.cpp
    src/NeighborsSearch.h
    src/NeighborsSearch.cpp
    src/Particle.cpp
    src/Particle.h
    src/ParticleSystem.cpp
    src/ParticleSystem.h
    src/PbdSolver.cpp
    src/PbdSolver.h
    src/ParticleParticleCollisionConstraint.cpp
    src/ParticleParticleCollisionConstraint.h
    src/ParticleSphereCollisionConstraint.cpp
    src/ParticleSphereCollisionConstraint.h
    src/SoftBody.cpp
    src/SoftBody.h
    src/World.cpp
    src/World.h
    )

find_package(Eigen3 REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

target_include_directories(UTAD-SIM03-PBD-Softbody
    PUBLIC
    ${CMAKE_CURRENT_LIST_DIR}/src
    ${EIGEN3_INCLUDE_DIR}
    ${OPENGL_INCLUDE_DIR}
    ${GLUT_INCLUDE_DIR}
    )

target_link_libraries(UTAD-SIM03-PBD-Softbody
    ${OPENGL_LIBRARIES}
    ${GLUT_LIBRARY}
    )

if (WIN32)
    add_custom_command(
        TARGET UTAD-SIM03-PBD-Softbody
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} ARGS -E copy_if_different
        "${PROJECT_SOURCE_DIR}/External/freeglut-3.2.1/build-vs2019/bin/$<CONFIG>/freeglut$<$<CONFIG:Debug>:d>.dll"
        "${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>"
        )
endif ()
