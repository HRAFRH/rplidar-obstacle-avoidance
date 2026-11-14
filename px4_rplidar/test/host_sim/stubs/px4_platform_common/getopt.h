// Host test stand-in: see host_px4.hpp
#pragma once
#include "host_px4.hpp"
#ifndef EOF
#define EOF (-1)
#endif
int px4_getopt(int argc, char *argv[], const char *options, int *myoptind, const char **myoptarg);
