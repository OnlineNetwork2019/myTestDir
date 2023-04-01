/**
 ****************************************************************************************
 *
 * @file lli_test.c
 *
 * @brief Definition of the functions used by ISO test commands
 *
 * Copyright (C) RivieraWaves 2009-2019
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup LLI
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // Stack configuration
#if (BLE_ISO_PRESENT)
#include "ke_task.h"        // For KE_MSG_HANDLER_NO_STATIC macro definition
#include "co_bt.h"          // HCI Definitions
#include "hci.h"            // HCI Interface
#include "lli_int.h"        // Isochronous internals
#include "co_math.h"        // for CO_BIT macro usage

#if (BLE_ISOGEN)
#include "isogen.h"         // Isochronous Payload generator
#endif //(BLE_ISOGEN)

/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (BLE_CIS || BLE_BIS)
int hci_le_iso_tx_test_cmd_handler(struct hci_le_iso_tx_test_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // Command status
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->payload_type <= ISO_TEST_MAX_LEN)
    {
        // Channel handle
        uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
        // Pointer to the indicated Isochronous environment
        struct lli_chan_env *p_env;

        status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

        // Check if ISO channel exists
        if (status == CO_ERROR_NO_ERROR)
        {
            // Should be handled by the expected Isochronous manager
            switch(p_env->type)
            {
                #if (BLE_CIS)
                case LLI_ISO_CHAN_CIS:
                {
                    // Load RX Data Path
                    status = lli_cis_data_path_set(p_env, ISO_SEL_TX, ISO_DP_ISOGEN);

                    // Set payload type
                    isogen_payload_type_set(chan_hdl, ISO_SEL_TX, param->payload_type);
                } break;
                #endif // (BLE_CIS)
                #if (BLE_BIS)
                case LLI_ISO_CHAN_BIS:
                {
                     // Load RX Data Path
                    status = lli_bis_data_path_set(p_env, ISO_SEL_TX, ISO_DP_ISOGEN);

                    // Set payload type
                    isogen_payload_type_set(chan_hdl, ISO_SEL_TX, param->payload_type);
                } break;
                #endif // (BLE_BIS)
                default:
                {
                    status = CO_ERROR_UNSUPPORTED;
                } break;
            }
        }
    }

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_rx_test_cmd_handler(struct hci_le_iso_rx_test_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // Command status
    uint8_t status = CO_ERROR_INVALID_HCI_PARAM;

    if (param->payload_type <= ISO_TEST_MAX_LEN)
    {
        // Channel handle
        uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
        // Pointer to the indicated Isochronous environment
        struct lli_chan_env *p_env;

        status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

        // Check if ISO channel exists
        if (status == CO_ERROR_NO_ERROR)
        {
            // Should be handled by the expected Isochronous manager
            switch(p_env->type)
            {
                #if (BLE_CIS)
                case LLI_ISO_CHAN_CIS:
                {
                    // Load TX Data Path
                    status = lli_cis_data_path_set(p_env, ISO_SEL_RX, ISO_DP_ISOGEN);

                    // Set payload type
                    isogen_payload_type_set(chan_hdl, ISO_SEL_RX, param->payload_type);
                } break;
                #endif // (BLE_CIS)
                #if (BLE_BIS)
                case LLI_ISO_CHAN_BIS:
                {
                     // Load TX Data Path
                    status = lli_bis_data_path_set(p_env, ISO_SEL_RX, ISO_DP_ISOGEN);

                    // Set payload type
                    isogen_payload_type_set(chan_hdl, ISO_SEL_RX, param->payload_type);
                } break;
                #endif // (BLE_BIS)
                default:
                {
                    status = CO_ERROR_UNSUPPORTED;
                } break;
            }
        }
    }

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_read_test_counters_cmd_handler(struct hci_le_iso_read_test_counters_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_le_iso_read_test_counters_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_iso_read_test_counters_cmd_cmp_evt);

    // Channel handle
    uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);

    // Fill the message
    p_evt->conhdl = param->conhdl;
    p_evt->status = isogen_get_rx_stat(chan_hdl, &p_evt->received_packet_count, &p_evt->missed_packet_count, &p_evt->failed_packet_count);

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_iso_test_end_cmd_handler(struct hci_le_iso_test_end_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_le_iso_test_end_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_iso_test_end_cmd_cmp_evt);

    // Channel handle
    uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
    // Pointer to the indicated Isochronous environment
    struct lli_chan_env *p_env;
    // Command status
    uint8_t status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

    if (status == CO_ERROR_NO_ERROR)
    {
        status = isogen_get_rx_stat(chan_hdl, &p_evt->received_packet_count, &p_evt->missed_packet_count, &p_evt->failed_packet_count);

        if (status == CO_ERROR_NO_ERROR)
        {
            // Should be handled by the expected Isochronous manager
            switch(p_env->type)
            {
                #if (BLE_CIS)
                case LLI_ISO_CHAN_CIS:
                {
                    // Remove RX data path on CIS channel
                    status = lli_cis_data_path_remove(p_env, ISO_SEL_RX);

                    // If the Rx direction has not been set up
                    if(status != CO_ERROR_NO_ERROR)
                    {
                        // Remove TX data path on CIS channel
                        status = lli_cis_data_path_remove(p_env, ISO_SEL_TX);
                    }
                } break;
                #endif // (BLE_CIS)
                #if (BLE_BIS)
                case LLI_ISO_CHAN_BIS:
                {
                    // Remove RX data path on BIS channel
                    status = lli_bis_data_path_remove(p_env, ISO_SEL_RX);

                    if(status == CO_ERROR_NO_ERROR)
                    {
                        // Remove TX data path on BIS channel
                        status = lli_bis_data_path_remove(p_env, ISO_SEL_TX);
                    }
                } break;
                #endif // (BLE_BIS)
                default:
                {
                    status = CO_ERROR_UNSUPPORTED;
                } break;
            }
        }
    }

    // Fill the message
    p_evt->status                 = status;
    p_evt->conhdl                 = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

#endif // (BLE_CIS || BLE_BIS)
#endif // (BLE_ISO_PRESENT)
/// @} LLI
