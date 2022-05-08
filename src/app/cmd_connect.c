#include <cmds.h>
#include <bt_test.h>

// add connect command
static int set_role_to_slave(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "\nSlave role. Starting advertising\n");
    adv_start();

    return 0;
}

static int set_role_to_master(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "\nMaster role. Starting scanning\n");
    scan_start();

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_role,
                               SHELL_CMD(master, NULL, "Set master role - start listening for input", set_role_to_master),
                               SHELL_CMD(slave, NULL, "Set slave role - start advertising", set_role_to_slave),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(connect, &sub_role, "Set device role", default_cmd);
