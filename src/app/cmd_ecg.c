#include <cmd.h>
#include <spi_adc.h>
#include <zephyr/logging/log.h>
#include <stdio.h>
#include <stdlib.h>

#ifdef CONFIG_BOARD_NRF5340DK_NRF5340_CPUAPP
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
    int16_t data_rate_code = ads129x_get_reg_DR_from_speed(data_rate);
    if (data_rate < 0)
    {
        shell_error(shell, "Invalid parameter value please check help");
        return -EINVAL;
    }

    shell_print(shell, "Setting data rate to: %"PRIu16, data_rate);
    uint8_t reg_val = 0;
    ads129x_read_registers(ADS129X_REG_CONFIG1, 1, &reg_val);
    WRITE_BIT(reg_val, ADS129X_BIT_HR, 1);
    WRITE_BIT(reg_val, ADS129X_BIT_DR0, data_rate_code);
    ads129x_write_registers(ADS129X_REG_CONFIG1, 1, &reg_val);

    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(sub_role,
                               SHELL_CMD(enable, NULL, "Enables continuos data transfer", ecg_data_enable),
                               SHELL_CMD(disable, NULL, "Disables continuos data transfer", ecg_data_disable),
                               SHELL_CMD(print, NULL, "Should data be printed (yes/no)", ecg_data_dump),
                               SHELL_CMD(data_rate, NULL, "Data rates (32000, 16000, 8000, 4000, 2000, 1000, 500)", ecg_rate_settings),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(ecg, &sub_role, "Drives ECG behaviour", default_cmd);

#endif