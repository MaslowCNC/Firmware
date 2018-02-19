Import('env')

#
# Dump build environment (for debug)
# print env.Dump()
#

env.Append(
  LINKFLAGS=[
      "-g"
  ]
)
