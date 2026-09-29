# embed_file.cmake
#
# cmake -DIN=<file> -DOUT=<file.c> -DNAME=<symbol> -P embed_file.cmake
# Writes <file> into a C array so it can be served from flash.

file(READ "${IN}" hex HEX)
string(LENGTH "${hex}" hex_length)
math(EXPR size "${hex_length} / 2")
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${hex}")
file(WRITE "${OUT}"
     "// Generated from ${IN} by embed_file.cmake. Do not edit.\n"
     "const unsigned char ${NAME}[] = {${bytes}};\n"
     "const unsigned int ${NAME}_size = ${size};\n")
