cmake_minimum_required(VERSION 3.22.1)


project(stm32u5xx_valve_tests C ASM CXX)


# message("CMAKE_C_FLAGS = ${CMAKE_C_FLAGS}")
# message("CMAKE_EXE_LINKER_FLAGS = ${CMAKE_EXE_LINKER_FLAGS}")



find_package(stm32u5xx_cmsis_ld REQUIRED)
find_package(stm32u5xx_cmsis_system REQUIRED)
find_package(stm32u545xx_cmsis_startup REQUIRED)

find_package(stm32u5xx_hal REQUIRED)
find_package(stm32u5xx_system REQUIRED)
find_package(newlib_syscalls_default REQUIRED)



###############################################################################
#           Library
###############################################################################
set(LIB_TARGETS "")

set(SOURCES
    src/test.cpp
)

# add library
add_library(${PROJECT_NAME}_support ${HEADERS} ${SOURCES})
list(APPEND LIB_TARGETS ${PROJECT_NAME}_support)

target_include_directories(
    ${PROJECT_NAME}_support
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include>"
)

# target_link_libraries(${LIB_NAME} 
#     INTERFACE stm32u5xx_cmsis 
# )


# installing rules
# install(DIRECTORY include/
#     DESTINATION include
#     FILES_MATCHING PATTERN "*_p.h"
# )

# install(DIRECTORY include/
#     DESTINATION include
#     PATTERN "*_p.h" EXCLUDE
#     PATTERN "*.h"
# )




###############################################################################
#          Tests
###############################################################################

set(STM32_APP_LINKER_SCRIPT original/STM32U545xx_FLASH.ld)
# set(STM32_APP_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/ld/flash.ld)

set(APP_TARGETS
    test_stm32u5xx_valve_single
)

foreach(target ${APP_TARGETS})
    message(STATUS "target = " ${target})

    if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/main_${target}.c")
        add_executable(${target} src/tests/main_${target}.c)
    elseif(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/tests/main_${target}.cpp")
        add_executable(${target} src/tests/main_${target}.cpp)
    else()
        message(FATAL_ERROR "No source file for target ${target}.")
    endif()


    # add_executable(${target} src/tests/main_${target}.c)

    target_include_directories(
        ${target}
        PRIVATE
        "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
        "$<INSTALL_INTERFACE:include>"
    )

    # Will NOT rebuild on linker script file change
    target_link_options(${target} PRIVATE 
        "LINKER:--script=${STM32_APP_LINKER_SCRIPT}"
    )

    target_link_options(${target} PRIVATE 
        "LINKER:--print-memory-usage"
    )

    target_link_libraries(${target}
        ${LIB_TARGETS}
        stm32u5xx_cmsis_ld
        stm32u5xx_cmsis_system          # should define the strong symbols needed in startup
        stm32u545xx_cmsis_startup
        stm32u5xx_hal
        stm32u5xx_system
        newlib_syscalls_default
    )

    # if(target STREQUAL "test_stm32u5xx_hal_cpp_test")
    #     target_link_libraries(${target}
    # endif()

    stm32_target_create_map(${target})
    stm32_target_disasm(${target})

    stm32_target_create_hex(${target})

    stm32_target_add_prog_target(${target})

    list(APPEND APP_TARGETS ${target})

    # list(APPEND APP_TARGETS_HEX ${target}_custom_hex)

endforeach()



###############################################################################
#           Export
###############################################################################


set(COPY_HEX_TARGETS 
        test_stm32u5xx_valve_single
)

set(DEVEL_DIR ${PROJECT_SOURCE_DIR}/devel)


foreach(target ${COPY_HEX_TARGETS})
add_custom_target(devel_${target} ALL
    DEPENDS ${target}
    COMMAND mkdir -p ${DEVEL_DIR}
    COMMAND cp ${PROJECT_BINARY_DIR}/${target}.hex ${DEVEL_DIR}/
    COMMAND cp ${PROJECT_BINARY_DIR}/${target}.map ${DEVEL_DIR}/
    COMMAND cp ${PROJECT_BINARY_DIR}/${target}.lss ${DEVEL_DIR}/
    COMMAND cp ${PROJECT_BINARY_DIR}/${target} ${DEVEL_DIR}/
)
endforeach()


# install hex files into bin
#foreach(target ${APP_TARGETS})
#    install(
#        FILES "${PROJECT_BINARY_DIR}/${target}.hex"
#        RENAME app.hex
#        DESTINATION bin
#    )
#endforeach()


# Create project target for global management
btk_project(
    TARGETS ${LIB_TARGETS} ${APP_TARGETS} ${APP_TARGETS_HEX}
)