set(BCM2835_ROOT_DIR "${BCM2835_ROOT_DIR}" CACHE PATH "Directory to search for BCM28352")

if(CMAKE_SIZEOF_VOID_P MATCHES "8")
	set(_LIBSUFFIXES /lib64 /lib)
else()
	set(_LIBSUFFIXES /lib)
endif()

find_path(BCM2835_INCLUDE_DIR
	      NAMES bcm2835.h 
		  PATHS "${BCM2835_ROOT_DIR}"
		  PATH_SUFFIXES "include/")

find_library(BCM2835_LIBRARY
			 NAMES bcm2835)	

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(BCM2835
							      DEFAULT_MSG
							      BCM2835_INCLUDE_DIR)

if(BCM2835_FOUND)
	add_library(bcm2835 STATIC IMPORTED)
	target_include_directories(bcm2835 INTERFACE ${BCM2835_INCLUDE_DIR})
	set_target_properties(bcm2835 PROPERTIES
						  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
						  IMPORTED_LOCATION "${BCM2835_LIBRARY}")

endif()