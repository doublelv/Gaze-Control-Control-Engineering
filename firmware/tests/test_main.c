/* test_main.c - host test runner: make -C firmware/tests */
#include "testutil.h"
int g_fail = 0, g_checks = 0;
void test_protocol(const char *path);
void test_app(void);
int main(int argc, char **argv)
{
    test_protocol(argc > 1 ? argv[1] : "../../interface/test-vectors/protocol_v1.txt");
    test_app();
    printf("%d checks, %d failed\n", g_checks, g_fail);
    return g_fail ? 1 : 0;
}
