idf_component_register(
    SRCS
        "src/k_quirc.c"
        "src/k_quirc_version.c"
        "src/k_quirc_identify.c"
        "src/k_quirc_decode.c"
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "src"
)

# Adaptive threshold control loop - adjusts threshold based on timing pattern
# analysis, and provides k_quirc_decode_adaptive() (bootstrap sweep + lock)
# with its tuning surface: media-profile ladders (k_quirc_set_ladder, default
# K_QUIRC_LADDER_DEFAULT = emissive) and the probe budget
# (k_quirc_set_sweep_cap, FAST default K_QUIRC_FAST_CAP_DEFAULT = 4, the
# original probe set). The tuned numbers are #ifndef-guarded defines - add
# e.g. K_QUIRC_FAST_CAP_DEFAULT=6 or K_QUIRC_THRESHOLD_OFFSET_MAX=20 here to
# re-tune without patching source. Comment out to disable (the fixed-threshold
# path remains; the adaptive APIs stay callable and degrade gracefully).
target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_ADAPTIVE_THRESHOLD)

# Bilinear threshold interpolation - uses four quadrants for uneven lighting
# Comment out to disable (uses single global threshold instead)
target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_BILINEAR_THRESHOLD)

# Uncomment to enable the inverted-colors retry pass when no QR is found.
# target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_INVERTED_RETRY)

# Uncomment to enable QR debug visualization mode
# This shows the thresholded pixel buffer with timing pattern overlays
# target_compile_definitions(${COMPONENT_LIB} PUBLIC K_QUIRC_DEBUG)
