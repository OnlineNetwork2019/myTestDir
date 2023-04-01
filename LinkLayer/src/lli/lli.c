/**
 ****************************************************************************************
 *
 * @file lli.c
 *
 * @brief Definition of the functions used by the Link Layer ISO
 *
 * Copyright (C) RivieraWaves 2009-2017
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
#include "rwip_config.h"
#if (BLE_ISO_PRESENT)
#include "rwip.h"         // IP definitions
#include <string.h>       // For memset definition
#include "lli_int.h"      // Internal LLI API
#include "lli.h"          // LLI API
#include "ke_task.h"      // For LLI task creation
#include "ke_mem.h"       // For memory allocation
#include "llm.h"
#include "hci.h"          // For HCI definitions

#include "data_path.h"    // Isochronous data path manager
#include "co_utils.h"     // Utils functions

/*
 * DEFINES
 ****************************************************************************************
 */
/// Margin added by default to the distance between sub-events (in us)
#define LLI_DFLT_SUB_EVT_MARGIN 0

/*
 * TYPES DEFINITION
 *****************************************************************************************
 */

/// Isochronous Environment structure
struct lli_env_tag
{
    /// List of setup ISO channels
    /// Channel Handle and connection handle use same namespace
    struct lli_chan_env *      chan[BLE_ACTIVITY_MAX];
    #if (BLE_BIS || BLE_CIS)
    /// List of setup ISO groups
    struct lli_group_env *     group[BLE_ISO_GROUP_MAX];
    /// Preferred sub event margin between two sub events (IFS + margin) (in us)
    uint32_t                   sub_evt_margin;
    #endif // (BLE_BIS || BLE_CIS)
    /// Number of channels allocated
    uint8_t                    nb_chan;
};

/*
 * GLOBAL VARIABLE DEFINITION
 *****************************************************************************************
 */

/// Task definition
extern const struct ke_task_desc TASK_DESC_LLI;

/// Isochronous environment
struct lli_env_tag lli_env;

/*
 * MODULE INTERNAL FUNCTION DEFINITIONS
 ****************************************************************************************
 */


/*
 * EXPORTED FUNCTION DEFINITIONS
 ****************************************************************************************
 */

void lli_init(uint8_t init_type)
{
    // Channel handle
    uint16_t hdl;

    #if (BLE_BIS)
    // initialize BI module
    lli_bi_init(init_type);
    #endif // (BLE_BIS)

    #if (BLE_CIS)
    // initialize CI module
    lli_ci_init(init_type);
    #endif // (BLE_CIS)

    switch (init_type)
    {
        case RWIP_INIT:
        {
            // Create LLI task
            ke_task_create(TASK_LLI, &TASK_DESC_LLI);

            // Set all environment pointers to NULL
            memset(&lli_env.chan[0], 0, sizeof(lli_env.chan));
            #if (BLE_BIS || BLE_CIS)
            // Set all environment pointers to NULL
            memset(&lli_env.group[0], 0, sizeof(lli_env.group));
            #endif // (BLE_BIS || BLE_CIS)
        }
        break;

        case RWIP_RST:
        {
            // Clean all allocated ISO environments
            for (hdl = 0; hdl < BLE_ACTIVITY_MAX; hdl++)
            {
                // Free the channel
                lli_chan_cleanup(hdl);
            }

            #if (BLE_BIS || BLE_CIS)
            // Clean all allocated ISO environments
            for (hdl = 0; hdl < BLE_ISO_GROUP_MAX ; hdl++)
            {
                // Free the Group
                lli_group_cleanup(hdl);
            }
            #endif // (BLE_BIS || BLE_CIS)

        }
        // No break

        case RWIP_1ST_RST:
        {
            // default value to used that can be updated over a vendor specific command
            #if (BLE_BIS || BLE_CIS)
            lli_env.sub_evt_margin = LLI_DFLT_SUB_EVT_MARGIN;
            #endif // (BLE_BIS || BLE_CIS)
            lli_env.nb_chan        = 0;
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }

    // Initialize data path ISO driver
    data_path_init(init_type);
}

uint8_t lli_chan_create(uint8_t act_id, uint8_t iso_type, uint16_t size, struct lli_chan_env **pp_env)
{
    uint8_t status = CO_ERROR_NO_ERROR;

    // Allocated environment
    struct lli_chan_env* p_iso_env = (struct lli_chan_env *)ke_malloc(size, KE_MEM_ENV);


    // check if environment available
    if(p_iso_env != NULL)
    {
        ASSERT_ERR(size > sizeof(struct lli_chan_env));
        // Clean the content of the structure
        memset(p_iso_env, 0, sizeof(struct lli_chan_env));

        // Initialize ISO parameters
        p_iso_env->type      = iso_type;
        p_iso_env->hdl       = act_id;

        // Set pointer to the allocated environment
        lli_env.chan[act_id] = p_iso_env;
        lli_env.nb_chan++;
        *pp_env = p_iso_env;
    }
    else
    {
        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
    }

    // Return Status of channel allocation
    return (status);
}

void lli_chan_cleanup(uint16_t chan_hdl)
{
    ASSERT_INFO((chan_hdl < BLE_ACTIVITY_MAX), chan_hdl, 0);

    // Get indicated environment for indicated channel
    if (chan_hdl < BLE_ACTIVITY_MAX)
    {
        if(lli_env.chan[chan_hdl] != NULL)
        {
            // Free environment
            ke_free(lli_env.chan[chan_hdl]);

            // Clean environment pointer
            lli_env.chan[chan_hdl] = NULL;

            lli_env.nb_chan--;
        }
    }
}

uint8_t lli_chan_env_get(uint8_t chan_hdl, uint8_t iso_type, struct lli_chan_env **pp_env)
{
    // Status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;

    do
    {
        struct lli_chan_env *p_env;

        // Check provided channel handle
        if (chan_hdl >= BLE_ACTIVITY_MAX)
        {
            break;
        }

        // Get environment for indicated CIS Channel
        p_env = (struct lli_chan_env *)lli_env.chan[chan_hdl];

        // Check if ISO Channel well exists and is an expected chanel type
        if ((!p_env) || ((iso_type != LLI_ISO_CHAN_UNDEF) && (p_env->type != iso_type)))
        {
            break;
        }

        // Can return the environment
        *pp_env = p_env;

        status = CO_ERROR_NO_ERROR;
    } while (0);

    // Return status
    return (status);
}


#if (BLE_BIS || BLE_CIS)
uint8_t lli_group_create(uint8_t iso_type, uint16_t size, struct lli_group_env **pp_env)
{
    uint8_t  grp_hdl = BLE_INVALID_GROUP_HDL;
    uint8_t  status = CO_ERROR_NO_ERROR;
    uint16_t cur;

    // search for available stream handle
    for (cur = 0;  cur < BLE_ISO_GROUP_MAX;  cur++)
    {
        // Check if channel handle is available and if activity is also available
        if (!lli_env.group[cur])
        {
            grp_hdl = cur;
            break;
        }
    }

    // Check if channel allocation succeed
    if(grp_hdl == BLE_INVALID_GROUP_HDL)
    {
        status = CO_ERROR_CONN_REJ_LIMITED_RESOURCES;
    }
    else
    {
        // Allocated environment
        struct lli_group_env* p_env = (struct lli_group_env *)ke_malloc(size, KE_MEM_ENV);

        // Clean the content of the structure
        ASSERT_ERR(size > sizeof(struct lli_group_env));
        memset(p_env, 0, sizeof(struct lli_group_env));

        // Initialize ISO parameters
        p_env->type      = iso_type;
        p_env->hdl       = grp_hdl;

        // Set pointer to the allocated environment
        lli_env.group[grp_hdl] = p_env;
        *pp_env = p_env;
    }

    // Return Status of channel allocation
    return (status);
}

void lli_group_cleanup(uint16_t grp_hdl)
{
    ASSERT_INFO((grp_hdl < BLE_ISO_GROUP_MAX), grp_hdl, 0);

    // Get indicated environment for indicated channel
    if (grp_hdl < BLE_ISO_GROUP_MAX)
    {
        if(lli_env.group[grp_hdl] != NULL)
        {
            // Free environment
            ke_free(lli_env.group[grp_hdl]);

            // Clean environment pointer
            lli_env.group[grp_hdl] = NULL;
        }
    }
}

uint8_t lli_group_env_get(uint16_t grp_hdl, uint8_t iso_type, struct lli_group_env **pp_env)
{
    // Status
    uint8_t status;

    do
    {
        struct lli_group_env *p_env;

        // Check provided channel handle
        if (grp_hdl >= BLE_ISO_GROUP_MAX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Get environment for indicated CIS Channel
        p_env = lli_env.group[grp_hdl];

        // Check if ISO Channel well exists and is an expected chanel type
        if ((!p_env) || ((iso_type != LLI_ISO_GROUP_UNDEF) && (p_env->type != iso_type)))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Can return the environment
        *pp_env = p_env;

        status = CO_ERROR_NO_ERROR;
    } while (0);

    // Return status
    return (status);
}

uint32_t lli_sub_evt_margin_get(void)
{
    return lli_env.sub_evt_margin;
}

#endif // (BLE_BIS || BLE_CIS)

uint8_t lli_nb_chan_ava_get(void)
{
    return BLE_ISO_CHANNEL_MAX - lli_env.nb_chan;
}

void lli_link_stop_ind(uint8_t link_id, uint8_t reason)
{
    uint16_t chan_hdl;

    for (chan_hdl = 0; chan_hdl < BLE_ACTIVITY_MAX; chan_hdl++)
    {
        struct lli_chan_env *p_iso_env = lli_env.chan[chan_hdl];

        if (p_iso_env)
        {
            switch(p_iso_env->type)
            {
                #if (BLE_CIS)
                case LLI_ISO_CHAN_CIS:
                {
                    // inform that connection has been stopped
                    lli_cis_link_stop_ind(link_id, p_iso_env, reason);
                } break;
                #endif // (BLE_CIS)
                #if (BLE_ISO_MODE_0)
                case LLI_ISO_CHAN_AM0:
                {
                    // inform that connection has been stopped
                    lli_am0_link_stop_ind(link_id, p_iso_env, reason);
                } break;
                #endif // (BLE_ISO_MODE_0)

                default: /* Nothing to do */ break;
            }
        }
    }
}

#if (BLE_ISO_MODE_0)
bool lli_am0_check(uint8_t link_id)
{
    // Get environment for indicated link
    struct lli_chan_env *p_env = (struct lli_chan_env *) lli_env.chan[link_id];

    return ((p_env != NULL) && (p_env->type == LLI_ISO_CHAN_AM0));
}
#endif // (BLE_ISO_MODE_0)

#if ((RW_DEBUG) && (BLE_BIS || BLE_CIS))
int hci_dbg_iso_set_param_cmd_handler(struct hci_dbg_iso_set_param_cmd const *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_basic_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_basic_cmd_cmp_evt);

    // parameters can be changed only when no channel are configured
    if(lli_env.nb_chan != 0)
    {
        p_evt->status   = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        lli_env.sub_evt_margin = p_param->sub_evt_margin;

        p_evt->status = CO_ERROR_NO_ERROR;
    }

    // Send the command complete event
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif //((RW_DEBUG) && (BLE_BIS || BLE_CIS))

uint16_t lli_chanhdl_to_isohdl(uint8_t chan_hdl)
{
    uint16_t iso_hdl = BLE_INVALID_ISOHDL;
    struct lli_chan_env *p_iso_env = lli_env.chan[chan_hdl];

    if (p_iso_env)
    {
        switch(p_iso_env->type)
        {
            #if (BLE_CIS)
            case LLI_ISO_CHAN_CIS:
            {
                iso_hdl = BLE_CHANHDL_TO_CISHDL(chan_hdl);
            } break;
            #endif // (BLE_CIS)
            #if (BLE_BIS)
            case LLI_ISO_CHAN_BIS:
            {
                iso_hdl = BLE_CHANHDL_TO_BISHDL(chan_hdl);
            } break;
            #endif // (BLE_BIS)
            #if (BLE_ISO_MODE_0)
            case LLI_ISO_CHAN_AM0:
            {
                iso_hdl = BLE_CHANHDL_TO_AM0HDL(chan_hdl);
            } break;
            #endif // (BLE_ISO_MODE_0)

            default: /* Nothing to do */ break;
        }
    }

    return (iso_hdl);
}

#if (BLE_BIS || BLE_CIS)
int hci_le_rd_iso_link_quality_cmd_handler(struct hci_le_rd_iso_link_quality_cmd const *param, uint16_t opcode)
{
    // Allocate HCI message
    struct hci_le_rd_iso_link_quality_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_rd_iso_link_quality_cmd_cmp_evt);

    // Command status
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
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
                // Read CIS statistics
                status = lli_cis_stats_get(p_env, &p_evt->tx_unacked_packets, &p_evt->tx_flushed_packets, &p_evt->tx_last_subevent_packets,
                                                  &p_evt->retransmitted_packets, &p_evt->crc_error_packets, &p_evt->rx_unreceived_packets,
                                                  &p_evt->duplicate_packets);
            } break;
            #endif // (BLE_CIS)
            #if (BLE_BIS)
            case LLI_ISO_CHAN_BIS:
            {
                // Read BIS statistics
                status = lli_bis_stats_get(p_env, &p_evt->crc_error_packets, &p_evt->rx_unreceived_packets);

                if (status != CO_ERROR_NO_ERROR)
                {
                    p_evt->tx_unacked_packets       = 0;
                    p_evt->tx_flushed_packets       = 0;
                    p_evt->tx_last_subevent_packets = 0;
                    p_evt->retransmitted_packets    = 0;
                    p_evt->duplicate_packets        = 0;
                }
            } break;
            #endif // (BLE_BIS)
            default:
            {
                status = CO_ERROR_UNSUPPORTED;
            } break;
        }
    }

    p_evt->status = (status == CO_ERROR_NO_ERROR) ? CO_ERROR_NO_ERROR : CO_ERROR_UNKNOWN_CONNECTION_ID;
    p_evt->conhdl = param->conhdl;

    // Send the message
    hci_send_2_host(p_evt);

    return (KE_MSG_CONSUMED);
}
#endif // (BLE_BIS || BLE_CIS)

#endif // (BLE_ISO_PRESENT)
/// @} LLI
