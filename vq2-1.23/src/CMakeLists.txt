file(
	GLOB
	headers_files
	*.h 
)

include_directories (${CMAKE_SOURCE_DIR}/src)

# Installation targets
install (FILES ${headers_files} 
	 DESTINATION ${INCLUDE_INSTALL_DIR}
	 COMPONENT devel)

