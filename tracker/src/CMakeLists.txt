
find_package( OpenCV REQUIRED )
add_executable( tracker backgroundFilter.cc dyeFilter.cc main.cc )
target_link_libraries( tracker ${OpenCV_LIBS} )
