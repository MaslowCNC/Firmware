Import('env')

# Run the linker with "-g", to prevent stripping of debugging symbols
env.Append(
  LINKFLAGS=[
      "-g"
  ]
)

# Don't try to upload the firmware
env.Replace(UPLOADHEXCMD="echo Upload is not supported for ${PIOENV}. Skipping")
