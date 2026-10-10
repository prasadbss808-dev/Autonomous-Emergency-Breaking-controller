#include "selftest.h"
#include <string.h>
int main(int argc, char **argv)
{
    bool verbose = (argc > 1 && strcmp(argv[1], "-v") == 0);
    return selftest_run_all(verbose) == 0 ? 0 : 1;
}
