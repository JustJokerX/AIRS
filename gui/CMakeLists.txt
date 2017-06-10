# Find the OpenGL library
find_package(OpenGL REQUIRED)
include_directories(${OPENGL_INCLUDE_DIR})

# Find the Bullet library
find_package(Bullet REQUIRED)
include_directories(${BULLET_INCLUDE_DIRS})
set(BULLET_EX_LIBS libBullet3Common.a)

# Find the QtWidgets library
# set(CMAKE_PREFIX_PATH "C:\\Qt\\Qt5.7.1\\5.7\\msvc2015_64\\")
find_package(Qt5Widgets REQUIRED)
include_directories(${Qt5Widgets_INCLUDE_DIRS})
# Use the compile definitions defined in the Qt 5 Widgets module
# add_definitions(${Qt5Widgets_DEFINITIONS})
# set(CMAKE_AUTOMOC ON)

# set(QT_USE_QTOPENGL TRUE)

# Include current dir
set(CMAKE_INCLUDE_CURRENT_DIR ON)

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/inc
					${CMAKE_CURRENT_SOURCE_DIR}/src
					${CMAKE_CURRENT_BINARY_DIR}
					)
					
aux_source_directory(${CMAKE_CURRENT_SOURCE_DIR}/src DIR_SRCS)

# Find all "inc/*.h" into "INCS"
file(GLOB INCS "inc/*.h")
# Find all "src/*.h" into "HEADERS"
file(GLOB HEADERS "src/*.h")
source_group("Header Files" FILES ${INCS} ${HEADERS})

file(GLOB MOCS "inc/mainwindow.h")
file(GLOB FORMS "form/*.ui")
# FILE(GLOB RES "qrc/*.qrc")

source_group("Form Files" FILES ${FORMS})

qt5_wrap_cpp(HEADERS_MOC ${MOCS})
qt5_wrap_ui(FORMS_UIC ${FORMS})
# QT5_ADD_RESOURCES(RES_RCC ${RES})

source_group("Generated Files" FILES ${HEADERS_MOC} ${FORMS_UIC} "gui_automoc.cpp")

add_executable(gui ${DIR_SRCS} ${HEADERS} ${INCS} ${HEADERS_MOC} ${FORMS_UIC})

target_link_libraries(gui Qt5::Widgets ${BULLET_LIBRARIES} ${BULLET_EX_LIBS} ${OPENGL_LIBRARIES} )

set_target_properties(gui PROPERTIES FOLDER "App")

# 生成debug版本
SET(CMAKE_BUILD_TYPE "Release")
SET(CMAKE_CXX_FLAGS_DEBUG "$ENV{CXXFLAGS} -O0 -Wall -g -ggdb")
SET(CMAKE_CXX_FLAGS_RELEASE "$ENV{CXXFLAGS} -O3 -Wall")