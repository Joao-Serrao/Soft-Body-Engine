cmake_minimum_required(VERSION 3.5)
project(SBE)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /openmp:llvm")


# Paths to packages - change these to match your folder structure
set(TETGEN_DIR "${CMAKE_SOURCE_DIR}/packages/tetgen")
set(GLM_DIR "${CMAKE_SOURCE_DIR}/packages/glm")
set(FREEGLUT_INC "${CMAKE_SOURCE_DIR}/packages/freeglut/include")
set(FREEGLUT_LIB "${CMAKE_SOURCE_DIR}/packages/freeglut/lib/x64/freeglut.lib")
set(GLEW_INC "${CMAKE_SOURCE_DIR}/packages/glew/include")
set(GLEW_LIB "${CMAKE_SOURCE_DIR}/packages/glew/lib/release/x64/glew32.lib")
set(JSON_INC "${CMAKE_SOURCE_DIR}/packages/json-develop/single_include/nlohmann")

# === Shared sources ===
set(SHARED_SOURCES
        supportingLibrary.cpp
)

# === Executable 1: Main Engine ===
add_executable(SBE
        main.cpp
        supportingLibrary.cpp
)

# Include directories
target_include_directories(SBE PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${TETGEN_DIR}
        ${GLM_DIR}
        ${FREEGLUT_INC}
        ${GLEW_INC}
        ${JSON_INC}
)

# Link libraries
find_package(OpenGL REQUIRED)
find_package(OpenMP REQUIRED)

target_link_libraries(SBE PRIVATE
        ${OPENGL_gl_LIBRARY}
        ${FREEGLUT_LIB}
        ${GLEW_LIB}
        opengl32
        glu32
        OpenMP::OpenMP_CXX
)

# Copy DLLs
add_custom_command(TARGET SBE POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/packages/freeglut/bin/freeglut.dll"
        $<TARGET_FILE_DIR:SBE>
)
add_custom_command(TARGET SBE POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/packages/glew/bin/glew32.dll"
        $<TARGET_FILE_DIR:SBE>
)

# Copy shaders
file(COPY
        ${CMAKE_SOURCE_DIR}/vertex.glsl
        ${CMAKE_SOURCE_DIR}/fragment.glsl
        DESTINATION ${CMAKE_BINARY_DIR}
)

# Copy additional directory
file(COPY
        ${CMAKE_SOURCE_DIR}/levels
        DESTINATION ${CMAKE_BINARY_DIR}
)

# === Executable 2: ModelGenTest ===
add_executable(ModelGenTest
        modelGenerator.cpp
        ${TETGEN_DIR}/tetgen.cxx
        ${TETGEN_DIR}/predicates.cxx
        ${SHARED_SOURCES}
)

target_include_directories(ModelGenTest PRIVATE
        ${CMAKE_SOURCE_DIR}
        ${TETGEN_DIR}
        ${GLM_DIR}
        ${FREEGLUT_INC}
        ${GLEW_INC}
        ${JSON_INC}
)

target_link_libraries(ModelGenTest PRIVATE
        ${OPENGL_gl_LIBRARY}
        ${FREEGLUT_LIB}
        ${GLEW_LIB}
        opengl32
        glu32
)

add_custom_command(TARGET ModelGenTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/packages/freeglut/bin/freeglut.dll"
        $<TARGET_FILE_DIR:ModelGenTest>
)
add_custom_command(TARGET ModelGenTest POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
        "${CMAKE_SOURCE_DIR}/packages/glew/bin/glew32.dll"
        $<TARGET_FILE_DIR:ModelGenTest>
)
