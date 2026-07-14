# Turns a file into a C++ byte array so the binary carries it around.
# usage: cmake -DIN=<file> -DOUT=<cpp> -DSYM=<symbol> -P embed_file.cmake
file(READ "${IN}" hex HEX)
string(REGEX REPLACE "([0-9a-f][0-9a-f])" "0x\\1," bytes "${hex}")
file(WRITE "${OUT}" "#include <cstddef>
extern const unsigned char ${SYM}[];
extern const std::size_t ${SYM}_LEN;
const unsigned char ${SYM}[] = {${bytes}};
const std::size_t ${SYM}_LEN = sizeof(${SYM});
")
