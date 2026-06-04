# Try finding libserial via PkgConfig first
find_package(PkgConfig QUIET)
if(PKG_CONFIG_FOUND)
    pkg_check_modules(SERIAL QUIET IMPORTED_TARGET libserial)
endif()

# If PkgConfig failed, search manually
if(NOT SERIAL_FOUND)
    message(STATUS "libserial metadata not found, searching manually...")
    find_path(SERIAL_INCLUDE_DIRS NAMES libserial/SerialPort.h)
    find_library(SERIAL_LIBRARIES NAMES serial)

    if(SERIAL_INCLUDE_DIRS AND SERIAL_LIBRARIES)
        set(SERIAL_FOUND TRUE)
        # Create a clean, custom imported target
        if(NOT TARGET LibSerial::LibSerial)
            add_library(LibSerial::LibSerial INTERFACE IMPORTED)
            set_target_properties(LibSerial::LibSerial PROPERTIES
                INTERFACE_INCLUDE_DIRECTORIES "${SERIAL_INCLUDE_DIRS}"
                INTERFACE_LINK_LIBRARIES "${SERIAL_LIBRARIES}"
            )
        endif()
    endif()
elseif(TARGET PkgConfig::SERIAL)
    # Map the PkgConfig target to our custom namespace for consistency
    add_library(LibSerial::LibSerial INTERFACE IMPORTED)
    set_target_properties(LibSerial::LibSerial PROPERTIES
        INTERFACE_LINK_LIBRARIES PkgConfig::SERIAL
    )
endif()

if(NOT TARGET LibSerial::LibSerial)
  message(FATAL_ERROR "libserial not found! (Tried PkgConfig and manual search)")
endif()