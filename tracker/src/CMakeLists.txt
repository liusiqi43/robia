
find_package( OpenCV REQUIRED )
add_executable( tracker dyeFilter.cc main.cc )
target_link_libraries( tracker ${OpenCV_LIBS} )
