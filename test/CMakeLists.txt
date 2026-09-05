cmake_minimum_required(VERSION 3.16)
project(k_quirc_test C)
enable_testing()

set(K_QUIRC_SRC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../src)
set(K_QUIRC_INC_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../include)
set(SAMPLES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/pgm_samples)

# Sanitized build of the whole harness.  Configure a second build directory
# with -DK_QUIRC_SANITIZE=ON rather than switching this one: the validation
# suite runs the plain binary, which is an order of magnitude faster.
# Must precede the add_executable() calls to apply to them.
# Ignoring a K_QUIRC_WARN_UNUSED_RESULT return is a build error, not a warning.
add_compile_options(-Werror=unused-result)

option(K_QUIRC_SANITIZE "Build the test binaries with ASan and UBSan" OFF)
if(K_QUIRC_SANITIZE)
    add_compile_options(-fsanitize=address,undefined -fno-omit-frame-pointer
                        -fno-sanitize-recover=all -g)
    add_link_options(-fsanitize=address,undefined)
endif()

add_executable(k_quirc_test
    test_decode.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
    ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
)

add_executable(k_quirc_api_test
    test_api.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
    ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
)

add_executable(k_quirc_version_db_test
    test_version_db.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
)

# NOTE: k_quirc_decode.c is #included by the test rather than listed here -
# the segment decoders under test are static.
add_executable(k_quirc_decode_payload_test
    test_decode_payload.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
)

add_executable(k_quirc_convergence_test
    test_convergence.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
    ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
)

add_executable(k_quirc_bimodal_test
    test_bimodal.c
    ${K_QUIRC_SRC_DIR}/k_quirc.c
    ${K_QUIRC_SRC_DIR}/k_quirc_version.c
    ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
    ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
)

target_include_directories(k_quirc_bimodal_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
)

target_compile_definitions(k_quirc_bimodal_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
    BIMODAL_VECTOR="${CMAKE_CURRENT_SOURCE_DIR}/convergence/vectors/cv_v06_ppm08_sig000_ev00_ink00_gr00_tilt00.png"
)

target_compile_options(k_quirc_bimodal_test PRIVATE -Wall -Wextra -Wno-unused-function -O2)
target_link_libraries(k_quirc_bimodal_test m)

target_include_directories(k_quirc_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
)

target_include_directories(k_quirc_api_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
)

target_compile_definitions(k_quirc_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
    SAMPLES_DIR="${SAMPLES_DIR}"
)

target_compile_definitions(k_quirc_api_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
)

target_include_directories(k_quirc_convergence_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/stubs
)

target_compile_definitions(k_quirc_convergence_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
    CONVERGENCE_VECTORS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/convergence/vectors"
)

target_compile_options(k_quirc_convergence_test PRIVATE -Wall -Wextra -Wno-unused-function -O2)
target_link_libraries(k_quirc_convergence_test m)

target_compile_options(k_quirc_test PRIVATE -Wall -Wextra -Wno-unused-function -O2)
target_compile_options(k_quirc_api_test PRIVATE -Wall -Wextra -O2)
target_link_libraries(k_quirc_test m)
target_link_libraries(k_quirc_api_test m)

# Pure table walk - needs no image data and no feature macros.
target_include_directories(k_quirc_version_db_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
)
target_compile_options(k_quirc_version_db_test PRIVATE -Wall -Wextra -O2)

# Hand-built bitstreams - no images, but the library it links must be built
# in the shipping configuration.
target_include_directories(k_quirc_decode_payload_test PRIVATE
    ${K_QUIRC_INC_DIR}
    ${K_QUIRC_SRC_DIR}
)
target_compile_definitions(k_quirc_decode_payload_test PRIVATE
    K_QUIRC_ADAPTIVE_THRESHOLD
    K_QUIRC_BILINEAR_THRESHOLD
)
target_compile_options(k_quirc_decode_payload_test PRIVATE
    -Wall -Wextra -Wno-unused-function -O2)
target_link_libraries(k_quirc_decode_payload_test m)

# Exercise both the default shared image/pixel buffer and separate 16-bit labels.
# test_resize.c includes k_quirc.c to instrument its allocations and frees.
foreach(regions IN ITEMS 254 1024)
    set(target k_quirc_resize_${regions}_test)
    add_executable(${target}
        test_resize.c
        ${K_QUIRC_SRC_DIR}/k_quirc_version.c
        ${K_QUIRC_SRC_DIR}/k_quirc_identify.c
        ${K_QUIRC_SRC_DIR}/k_quirc_decode.c
    )
    target_include_directories(${target} PRIVATE ${K_QUIRC_INC_DIR} ${K_QUIRC_SRC_DIR})
    target_compile_definitions(${target} PRIVATE
        QUIRC_MAX_REGIONS=${regions}
        K_QUIRC_ADAPTIVE_THRESHOLD
        K_QUIRC_BILINEAR_THRESHOLD
        RESIZE_VECTOR="${CMAKE_CURRENT_SOURCE_DIR}/convergence/vectors/cv_v06_ppm08_sig000_ev00_ink00_gr00_tilt00.png"
    )
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wno-unused-function -O2)
    target_link_libraries(${target} m)
    add_test(NAME ${target} COMMAND ${target})
endforeach()

add_test(NAME api COMMAND k_quirc_api_test)
add_test(NAME version_db COMMAND k_quirc_version_db_test)
add_test(NAME decode_payload COMMAND k_quirc_decode_payload_test)
add_test(NAME bimodal COMMAND k_quirc_bimodal_test)
add_test(NAME convergence COMMAND k_quirc_convergence_test --min-lock 10 --min-yield 100)
