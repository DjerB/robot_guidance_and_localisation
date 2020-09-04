FLTK_WRAP_UI(UI registration.fl)
include_directories(${REGISTRATION_SOURCE_DIR}/src)
link_directories(${REGISTRATION_BINARY_DIR}/src)

set(CORELIBS ${GLUT_LIBRARY} ${OPENGL_LIBRARY} ${FLTK_LIBRARY} m)

#building just a library. 
add_library(UI deformation.cpp
glm.cpp
ogl.cpp
projection.cpp
scene_nodes.cpp
scene_ogl.cpp
shapeshade.cpp
similarity.cpp
tdgeometry.cpp
deformation.cpp
minfit.c
praxis.c
nrutil.c
brent.c
mnbrak.c
powell.c
linmin.c
 ${UI_FLTK_UI_SRCS})

add_executable(Registration main.cpp)
ADD_DEPENDENCIES(Registration UI)
#linking against a custom library
target_link_libraries(Registration UI ${CORELIBS} )

