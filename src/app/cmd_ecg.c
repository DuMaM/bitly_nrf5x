#include <cmd.h>
#include <spi_adc.h>

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
// add connect command
static int ecg_data_enable(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "EGC: Data enabled");
    ads129x_start();
    k_msleep(1);
    ads129x_rdatac();

    return 0;
}

static int ecg_data_disable(const struct shell *shell, size_t argc, char **argv)
{
    shell_print(shell, "EGC: Data disabled");
    ads129x_sdatac();
    k_msleep(1);
    ads129x_stop();

    return 0;
}

static int ecg_data_dump(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 1)
    {
        shell_help(shell);
        return SHELL_CMD_HELP_PRINTED;
    }

    if (argc > 2)
    {
        shell_error(shell, "%s: bad parameters count", argv[0]);
        return -EINVAL;
    }

    int8_t data_dump = atob(argv[1]);
    if (data_dump < 0)
    {
        shell_error(shell, "Invalid parameter try true/yes/y/1 or false/no/n/0");
        return -EINVAL;
    }

    shell_print(shell, "Data dump is: %s", data_dump > 0 ? "enabled" : "disabled");
    ads129x_print(data_dump > 0);
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_role,
                               SHELL_CMD(enable, NULL, "Enables continuos data transfer", ecg_data_enable),
                               SHELL_CMD(disable, NULL, "Disables continuos data transfer", ecg_data_disable),
                               SHELL_CMD(print, NULL, "Should data be printed (yes/no)", ecg_data_dump),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ecg, &sub_role, "Drives ECG behaviour", default_cmd);

#endif