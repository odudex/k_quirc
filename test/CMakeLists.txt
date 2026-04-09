cmake_minimum_required(VERSION 3.16)
project(k_quirc_test C)

set(K_QUIRC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(K_QUIRC_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(SAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pgm_samples)

add_executable(k_quirc_test
    test_decode.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
    ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
)

target_include_directories(k_quirc_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
)

target_compile_definitions(k_quirc_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
    SAMPLES_DIR="${SAMPLES_DIR}"
)

target_compile_options(k_quirc_test PRIVATE -Wall -Wextra -O2)
target_link_libraries(k_quirc_test m)
