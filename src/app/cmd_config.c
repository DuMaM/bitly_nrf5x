/*
 * Copyright (c) 2020 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <cmd.h>
#include <main.h>
#include <bt_test.h>

#include <zephyr/kernel.h>

K_THREAD_STACK_DEFINE(config_stack, 1024);
struct k_thread config_thread;

test_params_t test_params = {
    // BT_LE_CONNECTION IS A WORKAROUND ON CHECKING MULTIPLE PARAM NEGOTIATION
    .conn_param = BT_LE_CONN_PARAM(INTERVAL_MIN, INTERVAL_MAX, CONN_LATENCY, SUPERVISION_TIMEOUT),
    .phy = BT_CONN_LE_PHY_PARAM_2M,
    .data_len = BT_CONN_LE_DATA_LEN_PARAM(251,1064),
    .enable_rssi = true,
};

static int cmd_phy_1m(const struct shell *shell, size_t argc, char **argv)
{
    test_params.phy->options = BT_CONN_LE_PHY_OPT_NONE;
    test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_1M;
    test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_1M;

    shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_phy,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int cmd_phy_2m(const struct shell *shell, size_t argc, char **argv)
{
    test_params.phy->options = BT_CONN_LE_PHY_OPT_NONE;
    test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_2M;
    test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_2M;

    shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_phy,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

#if defined(RADIO_MODE_MODE_Ble_LR500Kbit) || defined(NRF5340_XXAA_APPLICATION)
static int cmd_phy_coded_s2(const struct shell *shell, size_t argc, char **argv)
{
    test_params.phy->options = BT_CONN_LE_PHY_OPT_CODED_S2;
    test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_CODED;
    test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_CODED;

    shell_print(shell, "PHY set to: %s", phy_str(test_params.phy));

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_phy,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}
#endif /*                                            \
        * defined(RADIO_MODE_MODE_Ble_LR1500Kbit) || \
        * defined(NRF5340_XXAA_APPLICATION)          \
        */

#if defined(RADIO_MODE_MODE_Ble_LR125Kbit) || defined(NRF5340_XXAA_APPLICATION)
static int cmd_phy_coded_s8(const struct shell *shell, size_t argc, char **argv)
{
    test_params.phy->options = BT_CONN_LE_PHY_OPT_CODED_S8;
    test_params.phy->pref_rx_phy = BT_GAP_LE_PHY_CODED;
    test_params.phy->pref_tx_phy = BT_GAP_LE_PHY_CODED;

    shell_print(shell, "PHY set to: %s",
                phy_str(test_params.phy));

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_phy,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}
#endif /*                                           \
        * defined(RADIO_MODE_MODE_Ble_LR125Kbit) || \
        * defined(NRF5340_XXAA_APPLICATION)         \
        */

// data len
static int cmd_data_len(const struct shell *shell, size_t argc, char **argv)
{
    uint16_t data_len;

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

    data_len = strtol(argv[1], NULL, 10);

    if ((data_len < BT_GAP_DATA_LEN_DEFAULT) ||
        (data_len > BT_GAP_DATA_LEN_MAX))
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0], data_len);
        shell_error(shell, "LE Data Packet Length must be between: %d and %d", BT_GAP_DATA_LEN_DEFAULT, BT_GAP_DATA_LEN_MAX);
        return -EINVAL;
    }

    test_params.data_len->tx_max_len = data_len;

    shell_print(shell, "LE Data Packet Length set to: %d", data_len);

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_len,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int cmd_data_timing(const struct shell *shell, size_t argc, char **argv)
{
    uint16_t data_time;

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

    data_time = strtol(argv[1], NULL, 10);

    if ((data_time < BT_GAP_DATA_TIME_DEFAULT) || (data_time > BT_GAP_DATA_TIME_MAX))
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0], data_time);
        shell_error(shell, "LE Data Packet time must be between: %d and %d", BT_GAP_DATA_TIME_DEFAULT, BT_GAP_DATA_TIME_MAX);
        return -EINVAL;
    }

    test_params.data_len->tx_max_time = data_time;

    shell_print(shell, "LE Data Packet Time set to: %d", data_time);

    /* initialize work item for test */
    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_len,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int cmd_conn_interval(const struct shell *shell, size_t argc, char **argv)
{
    uint16_t interval;

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

    interval = strtol(argv[1], NULL, 10);

    if ((interval < MIN_CONN_INTERVAL) ||
        (interval > MAX_CONN_INTERVAL))
    {
        shell_error(shell, "%s: Invalid setting: %d", argv[0],
                    interval);
        shell_error(shell,
                    "Connection interval must be between: %d and %d",
                    MIN_CONN_INTERVAL, MAX_CONN_INTERVAL);
        return -EINVAL;
    }

    test_params.conn_param->interval_max = interval;
    test_params.conn_param->interval_min = interval;
    test_params.conn_param->latency = 0;
    test_params.conn_param->timeout = SUPERVISION_TIMEOUT;

    shell_print(shell, "Connection interval set to: %d", interval);

    k_thread_create(&config_thread, config_stack,
                                    K_THREAD_STACK_SIZEOF(config_stack),
                                    config_update_param,
                                    NULL, NULL, NULL,
                                    SHELL_TEST_RUN_PRIO, 0, K_NO_WAIT);

    return 0;
}

static int rssi_cmd(const struct shell *shell, size_t argc, char **argv)
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

    int8_t rssi = atob(argv[1]);
    if (rssi < 0)
    {
        shell_error(shell, "Invalid parameter try true/y/1 or false/n/0");
        return -EINVAL;
    }

    test_params.enable_rssi = (bool)rssi;
    shell_print(shell, "RSSI measurement is set to: %s", rssi > 0 ? "true" : "false");

    return 0;
}

static int print_cmd(const struct shell *shell, size_t argc,
                     char **argv)
{
    shell_print(shell,  "==== Current test configuration ====\n");
    shell_print(shell,  "Data length:\t\t%d\n"
                        "Data time:\t\t%d\n"
                        "Connection interval:\t%d units (ms %d)\n"
                        "Preferred PHY:\t\t%s\n",
                test_params.data_len->tx_max_len,
                test_params.data_len->tx_max_time,
                test_params.conn_param->interval_min,
                (int)(test_params.conn_param->interval_min * 1.25),
                phy_str(test_params.phy));
    return 0;
}

SHELL_STATIC_SUBCMD_SET_CREATE(phy_sub,
                               SHELL_CMD(1M, NULL, "Set preferred PHY to 1Mbps", cmd_phy_1m),
                               SHELL_CMD(2M, NULL, "Set preferred PHY to 2Mbps", cmd_phy_2m),
#if defined(RADIO_MODE_MODE_Ble_LR500Kbit) || defined(NRF5340_XXAA_APPLICATION)
                               SHELL_CMD(coded_s2, NULL, "Set preferred PHY to Coded S2", cmd_phy_coded_s2),
#endif
#if defined(RADIO_MODE_MODE_Ble_LR125Kbit) || defined(NRF5340_XXAA_APPLICATION)
                               SHELL_CMD(coded_s8, NULL, "Set preferred PHY to Coded S8", cmd_phy_coded_s8),
#endif
                               SHELL_SUBCMD_SET_END);

SHELL_STATIC_SUBCMD_SET_CREATE(sub_config,
                               SHELL_CMD(data_length, NULL, "Configure data length", cmd_data_len),
                               SHELL_CMD(data_time, NULL, "Configure data frame timing", cmd_data_timing),
                               SHELL_CMD(conn_interval, NULL, "Configure connection interval <1.25ms units>", cmd_conn_interval),
                               SHELL_CMD(phy, &phy_sub, "Configure connection interval", default_cmd),
                               SHELL_CMD(rssi, NULL, "RSSI configuration y/n", rssi_cmd),
                               SHELL_CMD(print, NULL, "Print current configuration", print_cmd),
                               SHELL_SUBCMD_SET_END);

SHELL_CMD_REGISTER(config, &sub_config, "Configure testing parameters", default_cmd);
