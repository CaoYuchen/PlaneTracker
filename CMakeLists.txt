add_definitions(-Wall -march=native -O3 -msse3 -mssse3 -fomit-frame-pointer -g -DNDEBUG -Wno-variadic-macros -Wno-deprecated-register )
#add_definitions(-Wall -g -Wno-variadic-macros -Wno-deprecated-register )

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)

# get polyview
include_directories(${PROJECT_SOURCE_DIR}/src/core/include)

# get polyview_basic_io
include_directories( ${PROJECT_SOURCE_DIR}/src/addons/basic_io/include )

# get polyview_launch_helpers
include_directories( ${PROJECT_SOURCE_DIR}/src/addons/launch_helpers/include )

# get polyview_cvvis
include_directories( ${PROJECT_SOURCE_DIR}/src/addons/cvvis/include )

# get polyview_kinect
include_directories( ${PROJECT_SOURCE_DIR}/src/addons/kinect/include )

# get polyview_gl
include_directories( ${PROJECT_SOURCE_DIR}/src/addons/gl/include )

set(ADDON_LIBRARIES  polyview_basic_io polyview_dcv4l polyview_cvvis polyview_launch_helpers polyview_cvvis polyview_gl  polyview_kinect ${OpenCV_LIBS})


set( RGBD_SDVO_SOURCE_FILES
     src/RGBD_PLANETRACKER.cpp )

add_library( rgbd_planeTracker ${RGBD_SDVO_SOURCE_FILES} )
target_link_libraries( rgbd_planeTracker polyview )

add_executable( test_main test/main.cpp )
target_link_libraries( test_main rgbd_planeTracker polyview ${ADDON_LIBRARIES} )
