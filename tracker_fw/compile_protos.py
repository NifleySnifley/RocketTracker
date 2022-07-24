import sys
import os
import distutils.spawn as dspawn

# Create folder for compiled protobufs
if (not os.path.isdir("./include/protobufs")):
    os.mkdir("./include/protobufs")

# if (not dspawn.find_executable("nanopb_generator.exe")):
#     sys.stderr.write(
#         "ERROR: nanopb_generator not found in path and is required for compilation\n")
#     exit(1)
# generator = "nanopb_generator"

generator = "python3 ./.pio/libdeps/teensy40/Nanopb/generator/nanopb_generator.py"

exit(os.system(f"{generator} --output-dir=./include/protobufs -I ../protobufs/ " +
               " ".join(["../protobufs/"+file for file in os.listdir("../protobufs/")])))
