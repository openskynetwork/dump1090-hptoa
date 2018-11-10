
if ( ${ARCHITECTURE} MATCHES "armhf" )

    SET (CMAKE_CXX_FLAGS " -O3 -std=c++11 -ggdb  -DMISSING_GETTIME -DMISSING_NANOSLEEP -DNO_COMPAT -DFFT_GPU")
    SET ( SOURCE_FILES upsampling_fft.cpp hp-toa-timestamp.cpp manager.cpp

            gpu_fft_3/gpu_fft.c
            gpu_fft_3/gpu_fft_base.c
            gpu_fft_3/gpu_fft_twiddles.c
            gpu_fft_3/gpu_fft_shaders.c
            gpu_fft_3/gpu_fft_trans.c
            gpu_fft_3/mailbox.c)

    add_library (hp-toa SHARED ${SOURCE_FILES})

    TARGET_LINK_LIBRARIES(hp-toa
            ${Boost_LIBRARIES})


else ( ${ARCHITECTURE} MATCHES "armhf" )

    SET (CMAKE_CXX_FLAGS " -O3 -std=c++11 -ggdb  -Wno-unused-variable -DMISSING_GETTIME -DMISSING_NANOSLEEP -DNO_COMPAT")
    SET ( SOURCE_FILES upsampling_fft.cpp hp-toa-timestamp.cpp manager.cpp)

    add_library (hp-toa SHARED ${SOURCE_FILES})

    TARGET_LINK_LIBRARIES(hp-toa
            ${Boost_LIBRARIES}
            liquid)

endif( ${ARCHITECTURE} MATCHES "armhf" )




SET_PROPERTY(TARGET hp-toa PROPERTY SOVERSION 0.1)