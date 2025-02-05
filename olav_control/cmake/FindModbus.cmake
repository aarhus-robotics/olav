set(MODBUS_ROOT_DIR
	"${MODBUS_ROOT_DIR}"
	CACHE
	PATH
	"Root directory to search for libmodbus")

if("${CMAKE_SIZEOF_VOID_P}" MATCHES "8")
	set(_libsuffixes lib64 lib)

	# 64-bit dir: only set on win64
	file(TO_CMAKE_PATH "$ENV{ProgramW6432}" _progfiles)
else()
	set(_libsuffixes lib)
	if(NOT "$ENV{ProgramFiles(x86)}" STREQUAL "")
		# 32-bit dir: only set on win64
		file(TO_CMAKE_PATH "$ENV{ProgramFiles(x86)}" _progfiles)
	else()
		# 32-bit dir on win32, useless to us on win64
		file(TO_CMAKE_PATH "$ENV{ProgramFiles}" _progfiles)
	endif()
endif()

# Look for the header file.
find_path(MODBUS_INCLUDE_DIR
	NAMES
	modbus.h
	HINTS
	"${MODBUS_ROOT_DIR}"
	PATH_SUFFIXES
	include
	PATHS
	"${_progfiles}/libmodbus"
	C:/usr/local
	/usr/local
	/usr/include/modbus)

# Look for the library.
find_library(MODBUS_LIBRARY
	NAMES
	libmodbus.lib
	libmodbus.a
	libmodbus.so
	HINTS
	"${MODBUS_ROOT_DIR}"
	PATH_SUFFIXES
	${_libsuffixes}
	PATHS
	"${_progfiles}/libmodbus"
	C:/usr/local
	/usr/local)

# handle the QUIETLY and REQUIRED arguments and set MODBUS_FOUND to TRUE if
# all listed variables are TRUE
include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(Modbus
	DEFAULT_MSG
	MODBUS_LIBRARY
	MODBUS_INCLUDE_DIR)

if(MODBUS_FOUND)
	set(MODBUS_LIBRARIES ${MODBUS_LIBRARY})
	set(MODBUS_INCLUDE_DIRS ${MODBUS_INCLUDE_DIR})

	mark_as_advanced(MODBUS_ROOT_DIR)
else()
	set(MODBUS_LIBRARIES)
	set(MODBUS_INCLUDE_DIRS)
endif()

mark_as_advanced(MODBUS_LIBRARY MODBUS_INCLUDE_DIR)

