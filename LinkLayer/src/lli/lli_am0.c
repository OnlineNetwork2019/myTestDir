/**
 ****************************************************************************************
 *
 * @file lli_am0_task.c
 *
 * @brief LLI_AM0 task source file
 *
 * Copyright (C) RivieraWaves 2009-2017
 *
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * @addtogroup AUDIO
 * @{
 ****************************************************************************************
 */

/*
 * INCLUDE FILES
 ****************************************************************************************
 */

#include "rwip_config.h"    // stack configuration

#if (BLE_ISO_MODE_0)

#include <string.h>
#include "ke_msg.h"
#include "ke_task.h"
#include "ke_mem.h"

#include "lld.h"

#include "hci.h"            // host controller interface
#include "lli_int.h"        // LL ISO internals
#include "data_path.h"      // Codec management
#include "llm.h"            // Link info

#include  "co_math.h"       // usage of CO_BIT

#include "dbg.h"

/*
 * DEFINES
 ****************************************************************************************
 */
/// Retrieve AM0 environment
#define LLI_AM0_ENV_GET(channel, env_ptr)\
    lli_chan_env_get(channel, LLI_ISO_CHAN_AM0, (struct lli_chan_env**) (env_ptr))



/// AM0 Channel state
enum lli_am0_stream_state
{
    /// Disable state, channel allocated but not enabled
    LLI_AM0_STATE_DISABLE = 0,
    /// Enable state, an Isochronous link is established
    LLI_AM0_STATE_ENABLE,
};


/*
 * TYPES DEFINITION
 ****************************************************************************************
 */


/*
 * STRUCT DEFINITION
 ****************************************************************************************
 */

/// Audio LL task environment structure
struct lli_am0_env_tag
{
    /// Isochronous information
    struct lli_chan_env         chan;

    /// RX Isochronous Data path
    const struct data_path_itf* rx_dp;
    /// TX Isochronous Data path
    const struct data_path_itf* tx_dp;

    /// Connection Link identifier
    uint8_t                     link_id;
    /// Configure transmitter size in bytes
    uint8_t                     tx_size;
    /// Configure receiver size in bytes
    uint8_t                     rx_size;
    /// State, @see enum lli_am0_stream_state
    uint8_t                     state;
};


/*
 * ENUMERATIONS DEFINITION
 ****************************************************************************************
 */



/*
 * GLOBAL VARIABLES DEFINITION
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

int hci_vs_mic_less_set_cmd_handler(struct hci_vs_mic_less_set_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_basic_conhdl_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, param->conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);

    // link identifier
    uint8_t link_id  = BLE_CONHDL_TO_LINKID(param->conhdl);

    // Mark that mic less encryption is now required, directly checking link state
    uint8_t status = lld_con_am0_use_mic_less(link_id);

    // update the status
    event->status = status;
    event->conhdl = param->conhdl;
    // send the message
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

int hci_vs_setup_am0_data_path_cmd_handler(struct hci_vs_setup_am0_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_vs_basic_am0_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_vs_basic_am0_cmd_cmp_evt);

    // Initialize returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Channel handle
    uint8_t chan_hdl = BLE_AM0HDL_TO_CHANHDL(param->am0_hdl);

    do
    {
        // Pointer to the AM0 Channel information structure
        struct lli_am0_env_tag *p_am0_env = NULL;
        // Retrieve AM0 channel environment
        status = LLI_AM0_ENV_GET(chan_hdl, &p_am0_env);

        // If channel exists
        if (status == CO_ERROR_NO_ERROR)
        {
            const struct data_path_itf** pp_dp;
            int32_t trigger_offset = LLI_INVALID_TRIGGER_OFFSET;
            uint8_t direction;

            if (ISO_DP_INPUT == param->data_path_direction)
            {
                pp_dp = &(p_am0_env->tx_dp);
                direction = ISO_SEL_TX;
            }
            else if (ISO_DP_OUTPUT == param->data_path_direction)
            {
                pp_dp = &(p_am0_env->rx_dp);
                direction = ISO_SEL_RX;
            }
            else
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Check if the data path has already been set up
            if(!data_path_is_disabled(*pp_dp))
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Load Data Path
            status = lli_data_path_load(chan_hdl, direction, param->data_path_id, pp_dp);

            if (status == CO_ERROR_NO_ERROR)
            {
                // Update the data path to use
                status = lld_isoal_datapath_set(chan_hdl, direction, *pp_dp, trigger_offset != LLI_INVALID_TRIGGER_OFFSET, trigger_offset);
            }
        }

    } while (0);

    // Set status and send the complete event
    event->status  = status;
    event->am0_hdl = param->am0_hdl;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

int hci_vs_remove_am0_data_path_cmd_handler(struct hci_vs_remove_am0_data_path_cmd const *param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_vs_basic_am0_cmd_cmp_evt *event = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_vs_basic_am0_cmd_cmp_evt);
    // Channel handle
    uint8_t chan_hdl = BLE_AM0HDL_TO_CHANHDL(param->am0_hdl);

    // Pointer to the AM0 Channel information structure
    struct lli_am0_env_tag *p_am0_env = NULL;
    // Retrieve AM0 channel environment
    uint8_t status = LLI_AM0_ENV_GET(chan_hdl, &p_am0_env);

    // If channel exists
    if (status == CO_ERROR_NO_ERROR)
    {
        // Check if datapaths is enabled
        if (((param->direction & CO_BIT(ISO_SEL_RX)) && data_path_is_disabled(p_am0_env->rx_dp)) ||
            ((param->direction & CO_BIT(ISO_SEL_TX)) && data_path_is_disabled(p_am0_env->tx_dp)))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }

        if((status == CO_ERROR_NO_ERROR) && (param->direction & CO_BIT(ISO_SEL_RX)))
        {
            // Remove the Data path driver
            status = lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_RX, &(p_am0_env->rx_dp));
            // Update the data path to use
            lld_isoal_datapath_remove(p_am0_env->chan.hdl, ISO_SEL_RX);
        }

        if((status == CO_ERROR_NO_ERROR) && (param->direction & CO_BIT(ISO_SEL_TX)))
        {
            // Remove the Data path driver
            status = lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_TX, &(p_am0_env->tx_dp));
            // Update the data path to use
            lld_isoal_datapath_remove(p_am0_env->chan.hdl, ISO_SEL_TX);
        }
    }

    // Set status and send the complete event
    event->status  = status;
    event->am0_hdl = param->am0_hdl;
    hci_send_2_host(event);

    return (KE_MSG_CONSUMED);
}

int hci_vs_setup_am0_stream_cmd_handler(struct hci_vs_setup_am0_stream_cmd const *p_param, uint16_t opcode)
{
    struct hci_vs_setup_am0_stream_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, p_param->conhdl, opcode,
                                                                     hci_vs_setup_am0_stream_cmd_cmp_evt);
    // Initialize returned status
    uint8_t status = CO_ERROR_COMMAND_DISALLOWED;
    // Channel handle
    uint8_t chan_hdl = BLE_CONHDL_TO_LINKID(p_param->conhdl);
    // link identifier
    uint8_t link_id  = BLE_CONHDL_TO_LINKID(p_param->conhdl);

    // Pointer to the AM0 Channel information structure
    struct lli_am0_env_tag *p_am0_env = NULL;

    // Check that the connection exists
    if(!llm_link_active(link_id))
    {
        status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    }
    // If channel does not exists
    else if (CO_ERROR_NO_ERROR != LLI_AM0_ENV_GET(chan_hdl, &p_am0_env))
    {
        // Try to allocate a new AM0 environment
        status = lli_chan_create(chan_hdl, LLI_ISO_CHAN_AM0,
                                 sizeof(struct lli_am0_env_tag), (struct lli_chan_env**) &p_am0_env);

        // Stop operation is not possible to allocate memory
        if (status == CO_ERROR_NO_ERROR)
        {
            struct lld_con_am0_params audio_par;

            p_am0_env->tx_size                              = p_param->tx_size;
            p_am0_env->rx_size                              = p_param->rx_size;

            p_am0_env->link_id                              = link_id;
            p_am0_env->state                                = LLI_AM0_STATE_DISABLE;

            p_am0_env->chan.framing = ISO_UNFRAMED_MODE;

            // Initialize data path to disable
            lli_data_path_init(p_am0_env->chan.hdl, ISO_SEL_RX, &p_am0_env->rx_dp);
            lli_data_path_init(p_am0_env->chan.hdl, ISO_SEL_TX, &p_am0_env->tx_dp);

            // Load Audio parameters required to start the audio link
            audio_par.rx_size        = p_am0_env->rx_size;
            audio_par.tx_size        = p_am0_env->tx_size;

            status = lld_con_am0_start(p_am0_env->link_id, p_am0_env->chan.hdl, &audio_par);

            // an error occurs, go back to disable state
            if (status == CO_ERROR_NO_ERROR)
            {
                // Mark Audio Mode 0 Channel enabled
                p_am0_env->state           = LLI_AM0_STATE_ENABLE;
            }
        }
    }

    // Set channel handle in the command complete event message
    p_evt->am0_hdl  = BLE_CHANHDL_TO_AM0HDL(chan_hdl);
    // Set channel handle in the command complete event message
    p_evt->conhdl   = p_param->conhdl;
    // Send the command complete event
    p_evt->status   = status;
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}


int hci_vs_remove_am0_stream_cmd_handler(struct hci_vs_remove_am0_stream_cmd const *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_vs_basic_am0_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_vs_basic_am0_cmd_cmp_evt);
    // Initialize returned status
    uint8_t status;

    do
    {
        uint8_t chan_hdl = BLE_AM0HDL_TO_CHANHDL(p_param->am0_hdl);

        // Pointer to the AM0 Channel information structure
        struct lli_am0_env_tag *p_am0_env = NULL;

        // Retrieve AM0 channel environment
        status = LLI_AM0_ENV_GET(chan_hdl, &p_am0_env);

        if (status != CO_ERROR_NO_ERROR)
        {
            // Indicated channel does not exists, reject the command
            break;
        }

        // If ISO link is enabled
        if(p_am0_env->state == LLI_AM0_STATE_ENABLE)
        {
            // Remove the Rx Data path driver
            if (!data_path_is_disabled(p_am0_env->rx_dp))
            {
                lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_RX, &(p_am0_env->rx_dp));
            }

            // Remove the Tx Data path driver
            if (!data_path_is_disabled(p_am0_env->tx_dp))
            {
                lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_TX, &(p_am0_env->tx_dp));
            }

            // Force the audio to be stopped
            lld_con_am0_stop(p_am0_env->chan.hdl);

            // Remove Memory information of the channel
            lli_chan_cleanup(p_am0_env->chan.hdl);
        }

    } while(0);

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->am0_hdl = p_param->am0_hdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

/*
 * EXPORTED FUNCTIONS DEFINITION
 ****************************************************************************************
 */

void lli_am0_link_stop_ind(uint8_t link_id, struct lli_chan_env* p_iso_env, uint8_t reason)
{
    struct lli_am0_env_tag *p_am0_env = (struct lli_am0_env_tag *)p_iso_env;

    if (   (p_am0_env->link_id == link_id)
        && (p_am0_env->state  == LLI_AM0_STATE_ENABLE))
    {
        // Remove the Rx Data path driver
        if (!data_path_is_disabled(p_am0_env->rx_dp))
        {
            lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_RX, &(p_am0_env->rx_dp));
        }

        // Remove the Tx Data path driver
        if (!data_path_is_disabled(p_am0_env->tx_dp))
        {
            lli_data_path_remove(p_am0_env->chan.hdl, ISO_SEL_TX, &(p_am0_env->tx_dp));
        }

        // Force the audio to be stopped
        lld_con_am0_stop(p_am0_env->chan.hdl);

        // Remove Memory information of the channel
        lli_chan_cleanup(p_am0_env->chan.hdl);
    }
}

#endif //(BLE_ISO_MODE_0)

/// @} AUDIO
