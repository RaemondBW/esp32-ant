/* Host unit test entry point. */
#include "test.h"

int g_tests_run = 0;
int g_tests_failed = 0;
int g_checks = 0;

void run_message_tests(void);
void run_profile_tests(void);
void run_channel_tests(void);
void run_phy_tests(void);
void run_link_tests(void);
void run_mac_tests(void);
void run_known_tests(void);
void run_embedded_tests(void);

int main(void)
{
    printf("=== ANT / ANT+ host test suite ===\n\n");

    printf("[message framing]\n");
    run_message_tests();
    printf("\n[ANT+ profiles]\n");
    run_profile_tests();
    printf("\n[channel state machine + end-to-end]\n");
    run_channel_tests();
    printf("\n[ShockBurst PHY frame]\n");
    run_phy_tests();
    printf("\n[ANT+ <-> ShockBurst link mapping]\n");
    run_link_tests();
    printf("\n[ANT air-interface MAC over virtual air]\n");
    run_mac_tests();
    printf("\n[paired-device table]\n");
    run_known_tests();
    printf("\n[ANT serial bridge: ant_stack over ant_embedded over virtual air]\n");
    run_embedded_tests();

    printf("\n==================================\n");
    printf("tests: %d   checks: %d   failures: %d\n",
           g_tests_run, g_checks, g_tests_failed);
    if (g_tests_failed == 0) {
        printf("RESULT: PASS\n");
        return 0;
    }
    printf("RESULT: FAIL\n");
    return 1;
}
