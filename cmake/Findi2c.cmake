set(I2CDEV_ROOT_DIR "${I2C_ROOT_DIR}" CACHE PATH "Directory to search for libi2c-dev")

if(CMAKE_SIZEOF_VOID_P MATCHES "8")
	set(_LIBSUFFIXES /lib64 /lib)
else()
	set(_LIBSUFFIXES /lib)
endif()

find_path(I2C_INCLUDE_DIR
	      NAMES linux/i2c-dev.h 
		  PATHS "${I2C_ROOT_DIR}"
		  PATH_SUFFIXES "include/")

find_library(I2C_DEV_LIBRARY
			NAMES i2c)	

find_path(SMBUS_INCLUDE_DIR
		  NAMES i2c/smbus.h 
		  PATHS "${I2C_ROOT_DIR}"
		  PATH_SUFFIXES "include/")

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(I2C
	DEFAULT_MSG
	I2C_INCLUDE_DIR
	)

if(I2C_FOUND)
	add_library(i2c SHARED IMPORTED)
	target_include_directories(i2c INTERFACE ${I2C_INCLUDE_DIR} ${SMBUS_INCLUDE_DIR})
	set_target_properties(i2c PROPERTIES
						  IMPORTED_LINK_INTERFACE_LANGUAGES "CXX"
						  IMPORTED_LOCATION "${I2C_DEV_LIBRARY}")

endif()