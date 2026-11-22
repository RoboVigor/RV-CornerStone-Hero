# GCC/CMake build for the CornerStone firmware (macOS/Linux), alongside the Keil project.
#
# Single source of truth: the C file list and include paths are read from
# mdk/infantry.uvprojx at configure time, so adding a file to the Keil project is
# still the only step needed (re-run cmake to pick it up).
#
#   cmake --preset default        # or: cmake -S . -B build -G Ninja -DCMAKE_TOOLCHAIN_FILE=cmake/arm-none-eabi.cmake
#   cmake --build build           # -> build/infantry.elf/.hex/.bin
#   cmake --build build -t flash  # OpenOCD + CMSIS-DAP
cmake_minimum_required(VERSION 3.20)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/cmake/arm-none-eabi.cmake)
endif()
if(NOT CMAKE_BUILD_TYPE)
    # Keil target uses optimization level 0; keep the same default so timing/volatile
    # behaviour matches the competition binary. Use -DCMAKE_BUILD_TYPE=Release for -O2.
    set(CMAKE_BUILD_TYPE Debug CACHE STRING "" FORCE)
endif()

# Ninja pipes compiler output, so GCC disables color by itself; force it back on.
# Override with -DCMAKE_COLOR_DIAGNOSTICS=OFF (e.g. for CI logs).
if(NOT DEFINED CMAKE_COLOR_DIAGNOSTICS)
    set(CMAKE_COLOR_DIAGNOSTICS ON)
endif()

project(infantry C ASM)

set(UVPROJX ${CMAKE_CURRENT_SOURCE_DIR}/mdk/infantry.uvprojx)
set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS ${UVPROJX})
file(READ ${UVPROJX} _uv)

# ---- sources from uvprojx (.c only; Keil startup .s is replaced by the GCC one) ----
string(REGEX MATCHALL "<FilePath>[^<]+\\.c</FilePath>" _uv_files "${_uv}")
set(SOURCES "")
foreach(_f IN LISTS _uv_files)
    string(REGEX REPLACE "</?FilePath>" "" _f "${_f}")
    string(REPLACE "\\" "/" _f "${_f}")
    # ARMCC (RVDS) FreeRTOS port -> GCC port of the same kernel version (V9.0.0)
    string(REPLACE "portable/RVDS/ARM_CM4F" "portable/GCC/ARM_CM4F" _f "${_f}")
    get_filename_component(_f "${_f}" ABSOLUTE BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mdk)
    list(APPEND SOURCES ${_f})
endforeach()
list(REMOVE_DUPLICATES SOURCES)
list(APPEND SOURCES
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/startup_stm32f427xx_gcc.s
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/gcc_syscalls.c)

# ---- include paths from uvprojx (first non-empty <IncludePath>) ----
string(REGEX MATCH "<IncludePath>[^<]+</IncludePath>" _uv_inc "${_uv}")
string(REGEX REPLACE "</?IncludePath>" "" _uv_inc "${_uv_inc}")
string(REPLACE "\\" "/" _uv_inc "${_uv_inc}")
string(REPLACE "portable/RVDS/ARM_CM4F" "portable/GCC/ARM_CM4F" _uv_inc "${_uv_inc}")
set(INCLUDES "")
foreach(_d IN LISTS _uv_inc)
    get_filename_component(_d "${_d}" ABSOLUTE BASE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/mdk)
    if(IS_DIRECTORY ${_d})
        list(APPEND INCLUDES ${_d})
    endif()
endforeach()
list(REMOVE_DUPLICATES INCLUDES)

list(LENGTH SOURCES _n_src)
list(LENGTH INCLUDES _n_inc)
message(STATUS "uvprojx: ${_n_src} sources, ${_n_inc} include dirs")

# ---- flags ----
set(MCU_FLAGS -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

add_executable(infantry.elf ${SOURCES})
target_include_directories(infantry.elf PRIVATE ${INCLUDES})
# Same defines as the Keil target, minus the ARMCC-only ones (__CC_ARM, __TARGET_FPU_CM4).
target_compile_definitions(infantry.elf PRIVATE
    STM32F427_437xx USE_STDPERIPH_DRIVER ROBOT_LOOP_ONE ARM_MATH_CM4 USE_USB_OTG_FS)
target_compile_options(infantry.elf PRIVATE
    ${MCU_FLAGS}
    $<$<COMPILE_LANGUAGE:C>:-std=gnu99 -ffunction-sections -fdata-sections -fno-common>
    # GCC >= 14 made implicit-int / implicit-function-declaration / int-conversion /
    # incompatible-pointer-types hard errors; ARMCC only warns. Restore warnings.
    $<$<COMPILE_LANGUAGE:C>:-fpermissive>
    $<$<COMPILE_LANGUAGE:ASM>:-x assembler-with-cpp>
    $<$<CONFIG:Debug>:-O0 -g3>
    $<$<CONFIG:Release>:-O2 -g>)
target_link_options(infantry.elf PRIVATE
    ${MCU_FLAGS}
    -T${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F427IITx_FLASH.ld
    --specs=nosys.specs   # full newlib (Keil uses the standard lib, not MicroLIB) + stub syscalls
    -Wl,--gc-sections
    -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/infantry.map,--cref
    -Wl,--print-memory-usage
    -Wl,--no-warn-rwx-segments)
target_link_libraries(infantry.elf PRIVATE m)
set_property(TARGET infantry.elf APPEND PROPERTY LINK_DEPENDS
    ${CMAKE_CURRENT_SOURCE_DIR}/cmake/STM32F427IITx_FLASH.ld)

add_custom_command(TARGET infantry.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O ihex   $<TARGET_FILE:infantry.elf> infantry.hex
    COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:infantry.elf> infantry.bin
    COMMAND ${CMAKE_SIZE} $<TARGET_FILE:infantry.elf>
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    BYPRODUCTS infantry.hex infantry.bin)

# ---- flashing via OpenOCD + CMSIS-DAP ----
find_program(OPENOCD openocd)
set(OPENOCD_CFG ${CMAKE_CURRENT_SOURCE_DIR}/cmake/openocd-cmsis-dap.cfg)
add_custom_target(flash
    COMMAND ${OPENOCD} -f ${OPENOCD_CFG} -c "program $<TARGET_FILE:infantry.elf> verify reset exit"
    DEPENDS infantry.elf
    USES_TERMINAL
    COMMENT "Flashing infantry.elf via OpenOCD (CMSIS-DAP)")
add_custom_target(erase
    COMMAND ${OPENOCD} -f ${OPENOCD_CFG} -c "init" -c "reset halt" -c "stm32f4x mass_erase 0" -c "exit"
    USES_TERMINAL)
add_custom_target(reset
    COMMAND ${OPENOCD} -f ${OPENOCD_CFG} -c "init" -c "reset run" -c "exit"
    USES_TERMINAL)
