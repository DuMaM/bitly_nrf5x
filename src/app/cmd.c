/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cmd.h>
#include <bt_test.h>
#include <zephyr/logging/log.h>

LOG_MODULE_DECLARE(main);

int default_cmd(const struct shell *shell, size_t argc, char **argv)
{
    if (argc == 1)
    {
        shell_help(shell);
        return SHELL_CMD_HELP_PRINTED;
    }

    if (argc > 2)
    {
        LOG_ERR("%s: bad parameters count.", argv[0]);
        return -EINVAL;
    }

    if (argc == 2)
    {
        LOG_ERR("Uknown argument: %s", argv[1]);
        return -EINVAL;
    }

    return 0;
}

const char *phy_str(const struct bt_conn_le_phy_param *phy)
{
    static const char *const str[] = {
        "1 Mbps",
        "2 Mbps",
        "Coded S2",
        "Coded S8",
        "Unknown"};

    switch (phy->pref_tx_phy)
    {
    case BT_GAP_LE_PHY_1M:
        return str[0];

    case BT_GAP_LE_PHY_2M:
        return str[1];

    case BT_GAP_LE_PHY_CODED:
        if (phy->options == BT_CONN_LE_PHY_OPT_CODED_S2)
        {
            return str[2];
        }
        else if (phy->options == BT_CONN_LE_PHY_OPT_CODED_S8)
        {
            return str[3];
        }

    default:
        return str[4];
    }
}

// void print_2d_array(uint8_t *num, uint16_t size)
// {
//     int counter = 0;
//     while (counter++ < size)
//     {
//         printk("%c", *num);
//         num++;
//     }
// }

int8_t atob(const char *buffer)
{
#define lookup_size 4
    const static char *lookup_negative[lookup_size] = {"false", "0", "n", "no"};
    const static char *lookup_positive[lookup_size] = {"true", "1", "y", "yes"};

    for (int i = 0; i < lookup_size; i++)
    {
        if (!strcmp(lookup_negative[i], buffer))
            return 0;

        if (!strcmp(lookup_positive[i], buffer))
            return 1;
    }

    return -1;
}

void instruction_print(void)
{
    LOG_INF("");
    LOG_INF("Type 'config' to change the configuration parameters.");
    LOG_INF("You can use the Tab key to autocomplete your input.");
    LOG_INF("Type 'run' when you are ready to run the test.");
}
