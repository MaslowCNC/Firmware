Import('env')


varname = "TEENSY35_UPLOAD"

if varname in os.environ:
  print "$"+ varname + " is set, enabling upload.   "
else:
  # Don't try to upload the firmware
  env.Replace(UPLOADHEXCMD="echo Upload is disabled by default for ${PIOENV}. Skipping.")
