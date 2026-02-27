Import("env")

# Ensure the same FPU/float ABI flags are used at link time as at compile time.
# Without this, the linker may select the soft-float newlib variants and fail
# with: "uses VFP register arguments, firmware.elf does not".
env.Append(
    LINKFLAGS=[
        "-mfpu=fpv4-sp-d16",
        "-mfloat-abi=hard",
    ]
)
