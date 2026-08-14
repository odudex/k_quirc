idf_component_register(
    SRCS
        "src/k_quirc.c"
        "src/k_quirc_version.c"
        "src/k_quirc_identify.c"
        "src/k_quirc_decode.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
)

# Ignoring a K_QUIRC_WARN_UNUSED_RESULT return is a build error, not a warning.
target_compile_options(${COMPONENT_LIB} PRIVATE -Werror=unused-result)

# Adaptive threshold control loop - adjusts threshold based on timing pattern analysis
# Comment out to disable
target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_ADAPTIVE_THRESHOLD)

# Bilinear threshold interpolation - uses four quadrants for uneven lighting
# Comment out to disable (uses single global threshold instead)
target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_BILINEAR_THRESHOLD)

# Uncomment to enable the inverted-colors retry pass when no QR is found.
# target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_INVERTED_RETRY)

# Uncomment to enable QR debug visualization mode
# This shows the thresholded pixel buffer with timing pattern overlays
# target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_DEBUG)
