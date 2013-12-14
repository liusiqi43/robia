# Make sure the compiler can find include files from our library.
include_directories (${CMAKE_SOURCE_DIR}/src)

# Define our examples to compile
file(
	GLOB 
	USAGE_EXAMPLES
	*.cc
)

# loop over the list to compile them
foreach(f ${USAGE_EXAMPLES})
    get_filename_component(exampleName ${f} NAME_WE) 
    add_executable (${exampleName} ${f})

    install(PROGRAMS ${CMAKE_BINARY_DIR}/examples/${exampleName} 
            DESTINATION bin
            RENAME ${CMAKE_PROJECT_NAME}-${exampleName}
	    COMPONENT binary)
endforeach(f)
