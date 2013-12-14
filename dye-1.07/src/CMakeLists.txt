
file(
	GLOB
	headers_files
	*.h
)

# Make sure the compiler can find include files from our library.
include_directories (${CMAKE_SOURCE_DIR}/src)



install (FILES ${headers_files}
	 DESTINATION ${INCLUDE_INSTALL_DIR}
	 COMPONENT devel)
