#include <cmd.h>
#include <bt_test.h>
#include <main.h>

K_THREAD_STACK_DEFINE(connect_stack, 1024);
struct k_thread connect_thread;

// add connect command
static int set_role_to_slave(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "\nSlave role.\n");

    /* initialize work item for test */
    k_thread_create(&connect_thread, connect_stack,
                    K_THREAD_STACK_SIZEOF(connect_stack),
                    adv_start,
                    NULL, NULL, NULL,
                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int set_role_to_master(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "\nMaster role.\n");
    restore_state();

    /* initialize work item for test */
    k_thread_create(&connect_thread, connect_stack,
                    K_THREAD_STACK_SIZEOF(connect_stack),
                    scan_start,
                    NULL, NULL, NULL,
                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_role,
                               SHELL_CMD(master, NULL, "Set master role - start listening for input", set_role_to_master),
                               SHELL_CMD(slave, NULL, "Set slave role - start advertising", set_role_to_slave),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(connect, &sub_role, "Set device role", default_cmd);
