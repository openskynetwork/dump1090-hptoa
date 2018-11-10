
CMAKE_MINIMUM_REQUIRED( VERSION 2.8.4 )
PROJECT(dump1090)
# allow more human readable "if then else" constructs
#SET( CMAKE_ALLOW_LOOSE_LOOP_CONSTRUCTS TRUE )

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -std=gnu++11 -O3")
LIST (APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake/Modules)

execute_process(COMMAND "uname" OUTPUT_VARIABLE UNAME OUTPUT_STRIP_TRAILING_WHITESPACE)


# Determine current architecture
macro(dpkg_arch VAR_NAME)
  find_program(DPKG_PROGRAM dpkg DOC "dpkg program of Debian-based systems")
  if (DPKG_PROGRAM)
    execute_process(
            COMMAND ${DPKG_PROGRAM} --print-architecture
            OUTPUT_VARIABLE ${VAR_NAME}
            OUTPUT_STRIP_TRAILING_WHITESPACE
    )
  endif(DPKG_PROGRAM)
endmacro(dpkg_arch)

dpkg_arch(ARCHITECTURE)
MESSAGE("Architecture detected: ${ARCHITECTURE}")


SET (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ggdb -Wall -std=c++11 -O3 -DMISSING_GETTIME -DMISSING_NANOSLEEP")
SET (CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall -Werror -ggdb -D_GLIBCXX_DEBUG -D_GLIBCXX_DEBUG_PEDANTIC")

#TODO: Check operative system to add clock_sleep functions (see old Makefile)


# Set default build type
IF( NOT CMAKE_BUILD_TYPE )
  SET( CMAKE_BUILD_TYPE "release" )
ENDIF()


FIND_PACKAGE( RTLSDR REQUIRED )
INCLUDE_DIRECTORIES("${RTLSDR_INCLUDE_DIR}")

SET( DEPS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/Deps)
include(${DEPS_DIR}/rtl-sdr/CMakeLists.txt)

# Descend into subdirectories
ADD_SUBDIRECTORY(hp-toa)

# dump1090
SET ( DUMP1090_SOURCE_FILES
        mode_ac.c
        interactive.c
        icao_filter.c
        crc.c
        cpr.c
        convert.c
        anet.c
        net_io.c
        mode_s.c
        util.c
        track.c
        stats.c
        demod_2400.c
        dump1090.c)

add_executable (dump1090 ${DUMP1090_SOURCE_FILES})

TARGET_LINK_LIBRARIES(dump1090
        ${LIBRTLSDR_LIBRARIES}
        hp-toa
        pthread
        rt
        dl
        m
        usb-1.0
        )

# view1090
SET ( VIEW1090_SOURCE_FILES
        view1090.c
        anet.c
        interactive.c
        mode_ac.c
        mode_s.c
        net_io.c
        crc.c
        stats.c
        cpr.c
        icao_filter.c
        track.c
        util.c  )

add_executable (view1090 ${VIEW1090_SOURCE_FILES})

TARGET_LINK_LIBRARIES(view1090
        ${LIBRTLSDR_LIBRARIES}
        pthread
        rt
        dl
        m
        usb-1.0
 )

#faup1090
SET ( FAUP1090_SOURCE_FILES
        faup1090.c
        anet.c
        mode_ac.c
        mode_s.c
        net_io.c
        crc.c
        stats.c
        cpr.c
        icao_filter.c
        track.c
        util.c
        )

add_executable (faup1090 ${FAUP1090_SOURCE_FILES})

TARGET_LINK_LIBRARIES(faup1090
        ${LIBRTLSDR_LIBRARIES}
        pthread
        rt
        dl
        m
        usb-1.0
        )



