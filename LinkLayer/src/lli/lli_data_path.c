/**
 ****************************************************************************************
 *
 * @file lli_data_path.c
 *
 * @brief Definition of the functions used by the link layer manager
 *
 * Copyright (C) RivieraWaves 2009-2016
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
#include "data_path.h"      // Data path Manager
#include "co_math.h"        // for CO_BIT macro usage

#if BLE_ISOOHCI
#include "isoohci.h"        // ISOOHCI Definitions
#endif // BLE_ISOOHCI


/*
 * DEFINES
 ****************************************************************************************
 */

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

uint8_t lli_data_path_load(uint16_t chan_hdl, uint8_t direction, uint8_t data_path_type, const struct data_path_itf** pp_dp)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    const struct data_path_itf* dp_itf;

    // Retrieve data path interface
    dp_itf = data_path_itf_get(data_path_type, direction);

    // if data path not correctly loaded
    if(dp_itf == NULL)
    {
        status = CO_ERROR_INVALID_HCI_PARAM;
        dp_itf = data_path_itf_get(ISO_DP_DISABLE, direction);
    }

    // update the datapath pointer
    *pp_dp = dp_itf;

    return (status);
}

uint8_t lli_data_path_remove(uint16_t chan_hdl, uint8_t direction, const struct data_path_itf** pp_dp)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // load the default data path interface
    (*pp_dp) = data_path_itf_get(ISO_DP_DISABLE, direction);

    return (status);
}

void lli_data_path_init(uint16_t chan_hdl, uint8_t direction, const struct data_path_itf** pp_dp)
{
    // load the default data path interface
    (*pp_dp) = data_path_itf_get(ISO_DP_DISABLE, direction);
}

/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (BLE_CIS || (BLE_BIS && BLE_BROADCASTER))
int hci_le_rd_iso_tx_sync_cmd_handler(struct hci_basic_conhdl_cmd const *param, uint16_t opcode)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    // Send the command complete event
    struct hci_le_rd_iso_tx_sync_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_iso_tx_sync_cmd_cmp_evt);
    p_evt->conhdl = param->conhdl;
    p_evt->pkt_seq_nb   = 0;
    p_evt->time_stamp   = 0;
    p_evt->time_offset  = 0;

    #if BLE_ISOOHCI
    do
    {
        // Channel handle
        uint8_t channel = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
        // Pointer to the indicated Isochronous environment
        struct lli_chan_env *p_env;

        // Check if ISO channel exists
        status = lli_chan_env_get(channel, LLI_ISO_CHAN_UNDEF, &p_env);
        if (status != CO_ERROR_NO_ERROR)
            break;

        // Get timing information from ISOoHCI Input data path
        status = isoohci_in_tx_sync_get(channel, &p_evt->pkt_seq_nb, &p_evt->time_stamp, &p_evt->time_offset);

        if (status != CO_ERROR_NO_ERROR)
            break;

        // If the Host issues this command before an SDU has been transmitted by the Controller
        if((p_evt->pkt_seq_nb == 0) && (p_evt->time_stamp == 0))
        {
            p_evt->status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // When the Connection_Handle identifies a CIS or BIS that is transmitting unframed PDUs, the value of Time_Offset returned shall be zero
        if(p_env->framing == ISO_UNFRAMED_MODE)
        {
            p_evt->time_offset = 0;
        }

    } while(0);
    #endif // BLE_ISOOHCI

    p_evt->status = status;
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_CIS || (BLE_BIS && BLE_BROADCASTER))

#if (BLE_CIS || BLE_BIS)
int hci_le_rd_buf_size_v2_cmd_handler(void const *param, uint16_t opcode)
{
    // Send the command complete event
    struct hci_le_rd_buf_size_v2_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_buf_size_v2_cmd_cmp_evt);
    p_evt->status                 = CO_ERROR_NO_ERROR;
    p_evt->le_acl_data_packet_length     = BLE_MAX_OCTETS;
    p_evt->total_num_le_acl_data_packets = BLE_ACL_BUF_NB_TX;
    #if BLE_ISOOHCI
    p_evt->iso_data_packet_length        = BLE_HCI_ISO_IN_BUF_SIZE;
    p_evt->total_num_iso_data_packets    = BLE_HCI_ISO_IN_BUF_NB;
    #else // BLE_ISOOHCI
    p_evt->iso_data_packet_length        = 0;
    p_evt->total_num_iso_data_packets    = 0;
    #endif // BLE_ISOOHCI
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}

int hci_le_setup_iso_data_path_cmd_handler(struct hci_le_setup_iso_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_le_setup_iso_data_path_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_setup_iso_data_path_cmd_cmp_evt);
    // Channel handle
    uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
    // Pointer to the indicated Isochronous environment
    struct lli_chan_env *p_env;
    // Command status
    uint8_t status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

    // Check if ISO channel exists
    if (status == CO_ERROR_NO_ERROR)
    {
        // Should be handled by the expected Isochronous manager
        switch(p_env->type)
        {
            #if (BLE_CIS)
            case LLI_ISO_CHAN_CIS:
            {
                if(param->data_path_direction == ISO_DP_INPUT)
                {
                    status = lli_cis_data_path_set(p_env, ISO_SEL_TX, param->data_path_id);
                }
                else if(param->data_path_direction == ISO_DP_OUTPUT)
                {
                    status = lli_cis_data_path_set(p_env, ISO_SEL_RX, param->data_path_id);
                }
                else
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                }
            } break;
            #endif // (BLE_CIS)
            #if (BLE_BIS)
            case LLI_ISO_CHAN_BIS:
            {
                if(param->data_path_direction == ISO_DP_INPUT)
                {
                    status = lli_bis_data_path_set(p_env, ISO_SEL_TX, param->data_path_id);
                }
                else if(param->data_path_direction == ISO_DP_OUTPUT)
                {
                    status = lli_bis_data_path_set(p_env, ISO_SEL_RX, param->data_path_id);
                }
                else
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                }
            } break;
            #endif // (BLE_BIS)
            default:
            {
                status = CO_ERROR_UNSUPPORTED;
            } break;
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_remove_iso_data_path_cmd_handler(struct hci_le_remove_iso_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_le_remove_iso_data_path_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_remove_iso_data_path_cmd_cmp_evt);
    // Channel handle
    uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
    // Pointer to the indicated Isochronous environment
    struct lli_chan_env *p_env;
    // Command status
    uint8_t status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Should be handled by the expected Isochronous manager
        switch(p_env->type)
        {
            #if (BLE_CIS)
            case LLI_ISO_CHAN_CIS:
            {
                if(param->direction & CO_BIT(ISO_SEL_RX))
                {
                    // Remove RX data path on CIS channel
                    status = lli_cis_data_path_remove(p_env, ISO_SEL_RX);
                }

                if((status == CO_ERROR_NO_ERROR) && (param->direction & CO_BIT(ISO_SEL_TX)))
                {
                    // Remove TX data path on CIS channel
                    status = lli_cis_data_path_remove(p_env, ISO_SEL_TX);
                }
            } break;
            #endif // (BLE_CIS)
            #if (BLE_BIS)
            case LLI_ISO_CHAN_BIS:
            {
                if(param->direction & CO_BIT(ISO_SEL_RX))
                {
                    // Remove RX data path on BIS channel
                    status = lli_bis_data_path_remove(p_env, ISO_SEL_RX);
                }

                if((status == CO_ERROR_NO_ERROR) && (param->direction & CO_BIT(ISO_SEL_TX)))
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

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif // (BLE_CIS || BLE_BIS)

int hci_vs_set_iso_data_path_trigger_cmd_handler(struct hci_vs_set_iso_data_path_trigger_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_basic_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);
    // Channel handle
    uint8_t chan_hdl = BLE_ISOHDL_TO_CHANHDL(param->conhdl);
    // Pointer to the indicated Isochronous environment
    struct lli_chan_env *p_env;
    // Command status
    uint8_t status = lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_UNDEF, &p_env);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Should be handled by the expected Isochronous manager
        switch(p_env->type)
        {
            #if (BLE_CIS)
            case LLI_ISO_CHAN_CIS:
            {
                status = lli_cis_trigger_set(p_env, param->direction, param->enable, param->trigger_offset);
            } break;
            #endif // (BLE_CIS)

            #if (BLE_BIS)
            case LLI_ISO_CHAN_BIS:
            {
                status = lli_bis_trigger_set(p_env, param->direction, param->enable, param->trigger_offset);
            } break;
            #endif // (BLE_BIS)

            #if (BLE_ISO_MODE_0)
            case LLI_ISO_CHAN_AM0:
            {
                // TODO not present in AM0
//                status = lli_am0_trigger_set(p_env, param->direction, param->enable, param->trigger_offset);
            } break;
            #endif // (BLE_ISO_MODE_0)
            default:
            {
                status = CO_ERROR_UNSUPPORTED;
            } break;
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}


#endif // (BLE_ISO_PRESENT)
/// @} LLM
