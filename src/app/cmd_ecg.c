#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>
#include <cmd.h>
#include <spi_adc.h>
#include <app_utils.h>

#ifdef CONFIG_SPI
LOG_MODULE_DECLARE(main);

// add connect command
static int ecg_data_enable(const struct shell *shell, size_t argc, char **argv)
{
    ads129x_data_enable();
    return 0;
}

static int ecg_data_disable(const struct shell *shell, size_t argc, char **argv)
{
    ads129x_data_disable();
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

/* sets new speed value */
static int ecg_rate_settings(const struct shell *shell, size_t argc, char **argv)
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

    uint16_t data_rate = strtol(argv[1], NULL, 10);
    ads129x_set_data_rate(data_rate);
    if (ads129x_get_status()) {
        ads129x_data_disable();
        ads129x_data_enable();
    }
    return 0;
}


static int ecg_set_channels(const struct shell *shell, size_t argc, char **argv)
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

    bool result = atob(argv[1]);
    if (result) {
        ads129x_enable_test_signal();
    }else {
        ads129x_enable_hrm_signal();
    }
    return 0;
}


SHELL_STATIC_SUBCMD_SET_CREATE(sub_role,
                               SHELL_CMD(test_mode, NULL, "Force device to use tests signals on all leads", ecg_set_channels),
                               SHELL_CMD(enable, NULL, "Enables continuos data transfer", ecg_data_enable),
                               SHELL_CMD(disable, NULL, "Disables continuos data transfer", ecg_data_disable),
                               SHELL_CMD(print, NULL, "Should data be printed (yes/no)", ecg_data_dump),
                               SHELL_CMD(data_rate, NULL, "Data rates (32000, 16000, 8000, 4000, 2000, 1000, 500)", ecg_rate_settings),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ecg, &sub_role, "Drives ECG behaviour", default_cmd);

#endif