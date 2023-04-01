/**
 ****************************************************************************************
 *
 * @file lli_cis.c
 *
 * @brief Definition of the functions used by the link layer Connected Isochronous Stream module
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

#include "rwip_config.h"    // Stack configuration

#if (BLE_CIS)

#include <string.h>
#include "rwip.h"           // IP definitions
#include "lli_int.h"        // LLI internal definitions
#include "lli.h"            // LLI definitions
#include "llm.h"            // LLM definitions
#include "llc.h"            // For definitions of LLC kernel messages
#include "lld.h"            // LLD definitions
#include "hci.h"            // For HCI definitions
#include "co_utils.h"       // For GETF macro
#include "ble_util.h"       // BLE utility functions
#include "data_path.h"      // Isochronous channel Data path
#include "sch_plan.h"       // Schedule planning
#include "co_math.h"        // Mathematics API
#include "ke_mem.h"         // kernel memory

/*
 * DEFINES
 ****************************************************************************************
 */
/// Retrieve CIS environment
#define LLI_CIS_ENV_GET(chan_hdl, env_ptr)\
    lli_chan_env_get(chan_hdl, LLI_ISO_CHAN_CIS, (struct lli_chan_env**) (env_ptr))

/// Retrieve CIG environment
#define LLI_CIG_ENV_GET(grp_hdl, env_ptr)\
    lli_group_env_get(grp_hdl, LLI_ISO_GROUP_CIG, (struct lli_group_env**) (env_ptr))



/*
 * TYPE DEFINITION
 ****************************************************************************************
 */

/// CIS environment structure
struct lli_cis_env
{
    /// ISO Channel information
    struct lli_chan_env          chan;
    /// CIS parameters used during negotiation
    struct lli_cis_param         cis_param;

    /// RX and TX Isochronous Data path
    const struct data_path_itf*  p_dp[ISO_SEL_MAX];
    /// Trigger offset for the data path
    int32_t                      trigger_offset[ISO_SEL_MAX];

    /// Connection Link identifier
    uint8_t                      link_id;
    /// Handle of Group (unique identifier in controller)
    uint8_t                      grp_hdl;
    /// Reject or disable reason
    uint8_t                      reason;
    /// Role
    uint8_t                      role;
};

/// CIG environment structure
struct lli_cig_env
{
    /// ISO Stream information
    struct lli_group_env         group;
    /// list of CIS present in the group
    struct co_list               channels;

    /// CIG parameters used during negotiation
    struct lli_cig_param         cig_param;

    /// Sub-event Interval, valid only for interleave scheduling (in us)
    uint32_t                     sub_interval;
    /// Bit field used to mark which CIS identifier is present
    uint32_t                     bf_cis_present;
    /// Bit field that mark which CIS identifier is under negotiation
    uint32_t                     bf_cis_busy;
    /// Bit field that mark which CIS identifier is enabled
    uint32_t                     bf_cis_en;
    /// Planning activity offset (in half-slots)
    uint16_t                     act_offset;
    /// Slave connection link identifier (invalid if master role)
    uint8_t                      link_id;
    /// Number of CIS
    uint8_t                      nb_cis;
    /// CIG identifier
    uint8_t                      cig_id;
    /// Role of the CIG
    uint8_t                      role;
    /// Group packing method (@see enum iso_packing)
    uint8_t                      packing;
};

/// CI Environment structure to manage the CIS creation operation
struct lli_ci_env_tag
{
    /// CIS Creation command
    struct hci_le_create_cis_cmd const* p_create_cmd;

    /// Callback to execute when a CIS is started
    lli_cis_start_evt_cb                cb_start_evt;

    /// use to know which channel is under creation
    uint8_t cursor;
};

/*
 * LOCAL VARIABLE DEFINITION
 ****************************************************************************************
 */
/// CI Environment variable
struct lli_ci_env_tag lli_ci_env;

/*
 * LOCAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */

/**
 ****************************************************************************************
 * Compute Sub-Event duration in us
 *
 * @param[in] tx_size     transmission size
 * @param[in] tx_rate     transmission rate
 * @param[in] rx_size     reception size
 * @param[in] rx_rate     reception rate
 * @param[in] mic_present True if mic is transmitted over the air, False else
 *
 * @return Sub-Event duration in us
 ****************************************************************************************
 */
__STATIC uint32_t lli_ci_se_dur_compute(uint8_t tx_size, uint8_t tx_rate, uint8_t rx_size, uint8_t rx_rate, bool mic_present)
{
    uint32_t sub_evt_dur;
    // Compute packet duration over the air to have the minimum sub_evt duration
    sub_evt_dur  = (tx_size > 0) // check if mic len must be added if TX packet len != 0
                 ? ble_util_pkt_dur_in_us(tx_size + (mic_present ? MIC_LEN : 0), tx_rate)
                 : ble_util_pkt_dur_in_us(0, tx_rate);
    sub_evt_dur += (rx_size > 0) // check if mic len must be added if RX packet len != 0
                 ? ble_util_pkt_dur_in_us(rx_size + (mic_present ? MIC_LEN : 0), rx_rate)
                 : ble_util_pkt_dur_in_us(0, rx_rate);
    sub_evt_dur += BLE_IFS_DUR + BLE_MSS_DUR;

    return (sub_evt_dur);
}

/**
 ****************************************************************************************
 * @brief Sends HCI LE CIS Established Event to the host
 *
 * @param[in] p_cig_env       Pointer to the environment structure for the CIG
 * @param[in] p_cis_env       Pointer to the environment structure for the CIS enabled
 * @param[in] status          Status to report
 * @param[in] rate_m2s        Master to slave rate
 * @param[in] rate_s2m        Slave to master rate
 ****************************************************************************************
 */
__STATIC void lli_cis_established_evt_send(struct lli_cig_env *p_cig_env, struct lli_cis_env *p_cis_env,
                                           uint8_t status, uint8_t rate_m2s, uint8_t rate_s2m)
{
    // Allocate HCI message
    struct hci_le_cis_established_evt *p_evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_cis_established_evt);

    // Fill the message
    p_evt->subcode        = HCI_LE_CIS_ESTABLISHED_EVT_SUBCODE;
    p_evt->conhdl         = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);
    p_evt->status         = status;
    if(status == CO_ERROR_NO_ERROR)
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);
        struct lli_cig_param* p_cig_param = &(p_cig_env->cig_param);

        p_evt->cig_sync_delay = p_cig_param->grp_air_time;
        p_evt->cis_sync_delay = p_cig_param->grp_air_time - p_cis_param->grp_offset;
        p_evt->trans_latency_m2s = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->tx_trans_latency : p_cig_param->rx_trans_latency;
        p_evt->trans_latency_s2m = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->rx_trans_latency : p_cig_param->tx_trans_latency;
        p_evt->cis_sync_delay = p_cig_param->grp_air_time - p_cis_param->grp_offset;
        p_evt->phy_m2s        = co_rate_to_phy[rate_m2s];
        p_evt->phy_s2m        = co_rate_to_phy[rate_s2m];
        p_evt->nse            = p_cis_param->nse;
        p_evt->bn_m2s         = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->tx_bn : p_cis_param->rx_bn;
        p_evt->bn_s2m         = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->rx_bn : p_cis_param->tx_bn;
        p_evt->ft_m2s         = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->tx_ft : p_cig_param->rx_ft;
        p_evt->ft_s2m         = (p_cig_env->role == MASTER_ROLE) ? p_cig_param->rx_ft : p_cig_param->tx_ft;
        p_evt->max_pdu_m2s    = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->tx_max_pdu : p_cis_param->rx_max_pdu;
        p_evt->max_pdu_s2m    = (p_cig_env->role == MASTER_ROLE) ? p_cis_param->rx_max_pdu : p_cis_param->tx_max_pdu;
        p_evt->iso_interval   = p_cig_param->iso_interval;
    }
    else
    {
        p_evt->cig_sync_delay = 0;
        p_evt->cis_sync_delay = 0;
        p_evt->trans_latency_m2s = 0;
        p_evt->trans_latency_s2m = 0;
        p_evt->cis_sync_delay = 0;
        p_evt->phy_m2s        = 0;
        p_evt->phy_s2m        = 0;
        p_evt->nse            = 0;
        p_evt->bn_m2s         = 0;
        p_evt->bn_s2m         = 0;
        p_evt->ft_m2s         = 0;
        p_evt->ft_s2m         = 0;
        p_evt->max_pdu_m2s    = 0;
        p_evt->max_pdu_s2m    = 0;
        p_evt->iso_interval   = 0;
    }

    // Send the message
    hci_send_2_host(p_evt);
}

/**
 ****************************************************************************************
 * @brief Sends HCI CIS Disconnect Complete Event to the host
 *
 * @param[in] p_cis_env    Pointer to the environment structure for the disabled channel
 * @param[in] reason       disconnection reason
 ****************************************************************************************
 */
__STATIC void lli_cis_disc_cmp_evt_send(struct lli_cis_env *p_cis_env, uint8_t reason)
{
    uint16_t conhdl  = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);
    struct hci_disc_cmp_evt *disc_cc = KE_MSG_ALLOC(HCI_EVENT, conhdl, HCI_DISC_CMP_EVT_CODE, hci_disc_cmp_evt);

    disc_cc->status  = CO_ERROR_NO_ERROR;
    disc_cc->conhdl  = conhdl;
    disc_cc->reason  = reason;

    // send the message
    hci_send_2_host(disc_cc);
}

#if (BLE_PERIPHERAL)
/**
 ****************************************************************************************
 * @brief Sends HCI LE CIS Req Event to the host
 *
 * @param[in] p_cis_env    Pointer to the environment structure for the channel
 ****************************************************************************************
 */
__STATIC void lli_cis_request_evt_send(struct lli_cis_env *p_cis_env)
{
    struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);
    // Allocate HCI message
    struct hci_le_cis_request_evt *p_evt = KE_MSG_ALLOC(HCI_LE_EVENT, 0, HCI_LE_META_EVT_CODE, hci_le_cis_request_evt);

    // Fill the message
    p_evt->subcode              = HCI_LE_CIS_REQUEST_EVT_SUBCODE;
    p_evt->acl_conhdl           = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);
    p_evt->cis_conhdl           = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);
    p_evt->cig_id               = p_cis_param->cig_id;
    p_evt->cis_id               = p_cis_param->cis_id;

    // Send the message
    hci_send_2_host(p_evt);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Allocate environment structure for a new CIS.
 *
 * @param[out] pp_cis_env   Address of pointer containing address of allocated environment
 *
 * @return Status of the allocation (@see enum co_error)
 ****************************************************************************************
 */
__STATIC uint8_t lli_cis_create(struct lli_cis_env **pp_cis_env)
{
    struct lli_chan_env *p_cis_env;
    uint8_t  act_id = BLE_ACTIVITY_MAX;
    // get available activity
    uint8_t status;

    do
    {
        status= llm_activity_free_get(&act_id);

        // Verify that an activity identifier and channel handle are available
        if(status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Check which channel and activity identifier are available.
        status = lli_chan_create(act_id, LLI_ISO_CHAN_CIS, sizeof(struct lli_cis_env), &p_cis_env);

        // Verify that an activity identifier and channel handle are available
        if(status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // And return address of environment
        *pp_cis_env = (struct lli_cis_env*) p_cis_env;

        // Reserve an activity
        llm_activity_cis_reserve(p_cis_env->hdl);

        // Set channel handle
        (*pp_cis_env)->link_id = BLE_INVALID_LINK_ID;

        // Initialize data path to disable
        lli_data_path_init(p_cis_env->hdl, ISO_SEL_RX, &((*pp_cis_env)->p_dp[ISO_SEL_RX]));
        (*pp_cis_env)->trigger_offset[ISO_SEL_RX] = LLI_INVALID_TRIGGER_OFFSET;
        lli_data_path_init(p_cis_env->hdl, ISO_SEL_TX, &((*pp_cis_env)->p_dp[ISO_SEL_TX]));
        (*pp_cis_env)->trigger_offset[ISO_SEL_TX] = LLI_INVALID_TRIGGER_OFFSET;

        // Clean the content of the structure
        memset(&((*pp_cis_env)->cis_param), 0, sizeof(struct lli_cis_param));
    } while (0);

    return (status);
}

/**
 * Get Group parameter information
 *
 * @param[in] cig_id     CIG identifier.
 * @param[in] role       Master of slave role
 * @param[in] link_id    Link identifier (meaningful only on slave side)
 *
 * @return Stream parameters information
 */
__STATIC struct lli_cig_env* lli_cig_get(uint8_t cig_id, uint8_t role, uint8_t link_id)
{
    struct lli_cig_env* p_env = NULL;

    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ISO_GROUP_MAX; cnt++)
    {
        // check if stream exist
        if (   (LLI_CIG_ENV_GET(cnt, &p_env) == CO_ERROR_NO_ERROR)
            && (p_env->cig_id  == cig_id)
            && (p_env->role    == role)
            && (p_env->link_id == link_id))
        {
            break;
        }
    }

    if(cnt == BLE_ISO_GROUP_MAX)
    {
        p_env = NULL;
    }

    // Return the stream parameters
    return (p_env);
}

#if (BLE_CENTRAL)
/**
 ****************************************************************************************
 * Handle creation of CIS one by one
 ****************************************************************************************
 */
__STATIC void lli_cis_create_cont(void)
{
    // Initialize returned status
    uint8_t cursor;

    for(cursor = lli_ci_env.cursor ; cursor < lli_ci_env.p_create_cmd->cis_count ; cursor++)
    {
        uint8_t status;
        // Pointer to the CIS Channel information structure
        struct lli_cis_env *p_cis_env;
        struct lli_cig_env *p_cig_env = NULL;

        uint8_t  chan_hdl = BLE_CISHDL_TO_CHANHDL(lli_ci_env.p_create_cmd->params[cursor].cis_conhdl);

        // Retrieve CIS environment
        status = LLI_CIS_ENV_GET(chan_hdl, &p_cis_env);

        if (status == CO_ERROR_NO_ERROR)
        {
            uint16_t act_offset;
            uint16_t act_margin;

            // Retrieve CIG environment
            LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

            // Check that negotiation is performed for first channel
            if(p_cig_env->bf_cis_en == 0)
            {
                struct lli_cig_param* p_cig_param = &(p_cig_env->cig_param);

                // Environment of other CIS of the CIG
                struct lli_cis_env *p_oth_cis_env;

                uint32_t channel_offset = 0;
                // CIG Sync Delay us
                uint32_t cig_sync_delay = 0;

                if(p_cig_env->packing == ISO_PACKING_INTERLEAVED)
                {
                    // **************************************
                    // Step 1, compute the channel offset and CIG interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->channels));

                    // browse all available channels
                    while(p_oth_cis_env != NULL)
                    {
                        struct lli_cis_param* p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        p_oth_cis_param->grp_offset = channel_offset;
                        // update the channel offset
                        channel_offset += p_oth_cis_param->sub_evt_dur + lli_sub_evt_margin_get();

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->chan.hdr.next;
                    }

                    // Group sub-event interval correspond to maximum channel offset
                    p_cig_env->sub_interval = channel_offset;

                    // **************************************
                    // Step 2, Compute CIG/CIS Sync delay, update channel sub-event interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->channels));

                    // browse all available channels
                    while(p_oth_cis_env != NULL)
                    {
                        uint32_t cis_sync_delay;
                        struct lli_cis_param* p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        // Set the channel offset, stream interval
                        p_oth_cis_param->sub_interval = p_cig_env->sub_interval;

                        // Compute CIS Sync delay
                        cis_sync_delay = p_oth_cis_param->grp_offset
                                       + (p_cig_env->sub_interval * (p_oth_cis_param->nse -1))
                                       + p_oth_cis_param->sub_evt_dur;

                        // update CIG sync delay if needed
                        if(cis_sync_delay > cig_sync_delay)
                        {
                            cig_sync_delay = cis_sync_delay;
                        }

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->chan.hdr.next;
                    }
                }
                else // Sequential
                {
                    // **************************************
                    // Compute channel offsets, stream delay, use sub event duration as sub interval
                    // **************************************
                    p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->channels));

                    // browse all available channels
                    while(p_oth_cis_env != NULL)
                    {
                        uint32_t cis_sync_delay;
                        struct lli_cis_param* p_oth_cis_param = &(p_oth_cis_env->cis_param);

                        // Set the channel offset, stream interval
                        p_oth_cis_param->grp_offset   = channel_offset;
                        p_oth_cis_param->sub_interval = p_oth_cis_param->sub_evt_dur + lli_sub_evt_margin_get();

                        // Compute CIS Sync delay
                        cis_sync_delay = p_oth_cis_param->sub_interval * p_oth_cis_param->nse;

                        // update CIG Sync delay (remove margin to be at end of sub event)
                        cig_sync_delay = channel_offset + cis_sync_delay - lli_sub_evt_margin_get();

                        // load next element
                        p_oth_cis_env = (struct lli_cis_env *) p_oth_cis_env->chan.hdr.next;
                        channel_offset += cis_sync_delay;
                    }
                }

                // If computed stream bandwidth is too large to fit in a channel interval, reject the CIS creation
                if(cig_sync_delay > ((uint32_t)p_cig_param->iso_interval * SLOT_SIZE*2))
                {
                    status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                }
                else
                {
                    // Request an offset range to the planner
                    struct sch_plan_req_param req_param;
                    req_param.interval_max    = p_cig_param->iso_interval << 2;
                    req_param.interval_min    = p_cig_param->iso_interval << 2;
                    req_param.duration_min    = (2*cig_sync_delay + (HALF_SLOT_SIZE-1))/HALF_SLOT_SIZE;
                    req_param.duration_max    = (2*cig_sync_delay + (HALF_SLOT_SIZE-1))/HALF_SLOT_SIZE;
                    req_param.offset_min      = 0;
                    req_param.offset_max      = req_param.interval_max-1;
                    req_param.margin          = 0;
                    req_param.pref_period     = 0;
                    req_param.conhdl          = lli_ci_env.p_create_cmd->params[cursor].cis_conhdl;
                    req_param.conhdl_ref      = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);

                    if(sch_plan_req(&req_param) == SCH_PLAN_ERROR_OK)
                    {
                        act_offset = req_param.offset_min;
                        act_margin = req_param.offset_max - req_param.offset_min;
                    }
                    else
                    {
                        // No available offset found, let slave provide preferred parameters
                        act_offset = 0;
                        act_margin = req_param.interval_max-1;
                    }

                    p_cig_param->grp_air_time = cig_sync_delay;
                }
            }
            else
            {
                // Timings are fixed for subsequent CISes
                act_offset = p_cig_env->act_offset;
                act_margin = 0;
            }

            if(status == CO_ERROR_NO_ERROR)
            {
                // Start negotiation at Connection level (handled by LLC CIS module)
                status = llc_cis_create_req(p_cis_env->link_id,  p_cis_env->chan.hdl, act_offset, act_margin);
                break;
            }
        }

        // Send message to inform host that CIS establishment fails
        lli_cis_established_evt_send(p_cig_env, p_cis_env, status, 0, 0);
    }

    lli_ci_env.cursor = cursor;

    // operation is over perform command clean-up
    if(lli_ci_env.cursor >= lli_ci_env.p_create_cmd->cis_count)
    {
        ke_free(ke_param2msg(lli_ci_env.p_create_cmd));
        lli_ci_env.p_create_cmd = NULL;
    }
}
#endif // (BLE_CENTRAL)

/**
 ****************************************************************************************
 * @brief Compute CIG/CIS parameters
 *
 * @param[in]   p_cmd        Set CIG parameters command
 * @param[out]  p_cig_param  CIG parameters
 * @param[in]   p_evt        Set CIS parameters command complete event (used to obtain the pointers to each CIS environment)
 ****************************************************************************************
 */
__STATIC uint8_t lli_cig_compute_params(const struct hci_le_set_cig_params_cmd *p_cmd, struct lli_cig_param* p_cig_param, const struct hci_le_set_cig_params_cmd_cmp_evt *p_evt)
{
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t cig_framing = p_cmd->framing;
    uint16_t cig_iso_intv = 0xFFFF;
    uint8_t count = 0;
    bool cig_repeat;

    do
    {
        cig_repeat = false;

        // Parse all the CISes in the CIG
        for (uint8_t i = 0; i < p_cmd->cis_count; i++)
        {
            uint8_t framing = cig_framing;

            uint8_t bn, nse, max_pdu;
            bool repeat;

            struct lli_cis_env   *p_cis_env;
            struct lli_cis_param* p_cis_param;

            LLI_CIS_ENV_GET(BLE_CISHDL_TO_CHANHDL(p_evt->conhdl[i]), &p_cis_env);
            ASSERT_ERR(p_cis_env != NULL);

            p_cis_param = &(p_cis_env->cis_param);

            p_cis_param->nse = 0;

            for (uint8_t j = 0; j < 2; j++)
            {
                uint32_t sdu_interval;
                uint16_t max_sdu;
                uint16_t iso_intv;
                uint16_t trans_latency;
                uint8_t rtn;

                uint8_t eff_pld_size = BLE_ISO_MAX_PAYLOAD_SIZE;

                if (j == 0) // Tx
                {
                    sdu_interval = p_cmd->sdu_interval_m2s;
                    max_sdu = p_cis_param->tx_max_sdu;
                    iso_intv = p_cmd->sdu_interval_m2s / (SLOT_SIZE * 2);
                    rtn = p_cmd->params[i].rtn_m2s;
                    trans_latency = p_cmd->max_trans_latency_m2s;
                }
                else // Rx
                {
                    sdu_interval = p_cmd->sdu_interval_s2m;
                    max_sdu = p_cis_param->rx_max_sdu;
                    iso_intv = p_cmd->sdu_interval_s2m / (SLOT_SIZE * 2);
                    rtn = p_cmd->params[i].rtn_s2m;
                    trans_latency = p_cmd->max_trans_latency_s2m;
                }

                if ((sdu_interval == 0) || (max_sdu == 0))
                {
                    max_sdu = 0;
                    iso_intv = 0;
                    rtn = 0;

                    continue;
                }

                if (CO_MOD(sdu_interval, (SLOT_SIZE * 2)))
                {
                    framing = 1;
                }

                // Assume that the CIS is encrypted
                eff_pld_size -= MIC_LEN;

                // This loop is needed because the framing parameter may change if initially 0
                do
                {
                    repeat = false;

                    // Re-initialize iso_intv as it may have changed
                    if (j == 0) // Tx
                    {
                        iso_intv = p_cmd->sdu_interval_m2s / (SLOT_SIZE * 2);
                    }
                    else // Rx
                    {
                        iso_intv = p_cmd->sdu_interval_s2m / (SLOT_SIZE * 2);
                    }

                    // Make sure that the ISO interval respects the transport latency requirements
                    {
                        uint16_t max_iso_intv;

                        if (framing)
                        {
                            if (sdu_interval > (trans_latency*1000))
                            {
                                status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                                break;
                            }
                            // trans_latency*1000 = 2*max_iso_intv*SLOT_SIZE*2 + sdu_interval
                            max_iso_intv = (trans_latency*1000 - sdu_interval) / (4*SLOT_SIZE);
                        }
                        else
                        {
                            // trans_latency*1000 = 2*max_iso_intv*SLOT_SIZE*2 - sdu_interval
                            max_iso_intv = (trans_latency*1000 + sdu_interval) / (4*SLOT_SIZE);
                        }

                        if (max_iso_intv < BLE_ISO_MIN_INTERVAL)
                        {
                            status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                            break;
                        }

                        if (iso_intv > max_iso_intv)
                        {
                            uint8_t factor = CO_DIVIDE_CEIL(iso_intv, max_iso_intv);
                            uint8_t rem = CO_MOD(iso_intv, factor);

                            iso_intv = iso_intv / factor;

                            if (rem && !framing)
                            {
                                framing = 1;

                                if ((i != 0) || (j != 0))
                                {
                                    repeat = true;
                                }
                            }
                        }
                    }

                    if ((count > 0) && (cig_iso_intv != 0xFFFF))
                    {
                        uint8_t factor = CO_DIVIDE_CEIL((sdu_interval / (SLOT_SIZE * 2)), cig_iso_intv);
                        uint8_t rem = CO_MOD(iso_intv, factor);

                        if (rem && !framing)
                        {
                            framing = 1;

                            if ((i != 0) || (j != 0))
                            {
                                repeat = true;
                            }
                        }

                        iso_intv = cig_iso_intv;
                    }

                    if (framing)
                    {
                        eff_pld_size -= (BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE);
                    }

                    if (max_sdu > eff_pld_size)
                    {
                        max_pdu = BLE_ISO_MAX_PAYLOAD_SIZE;
                        bn = CO_DIVIDE_CEIL(max_sdu, eff_pld_size);
                        nse = bn * (rtn + 1);
                        // Adjust max_pdu accordingly
                        {
                            uint16_t total_eff_pld_size = bn * eff_pld_size;
                            uint16_t total_diff = total_eff_pld_size - max_sdu;
                            uint8_t diff = total_diff / bn;
                            max_pdu -= diff;
                        }

                        // If iso_intv does not equal sdu_interval
                        if (iso_intv < (sdu_interval / (SLOT_SIZE * 2)))
                        {
                            uint8_t factor = CO_DIVIDE_CEIL((sdu_interval / (SLOT_SIZE * 2)), iso_intv);
                            bn = CO_DIVIDE_CEIL(bn, factor);
                            nse = bn * (rtn + 1);
                        }

                        while (bn > BLE_CIS_MAX_BN)
                        {
                            uint8_t factor = CO_DIVIDE_CEIL(bn, BLE_CIS_MAX_BN);
                            uint8_t rem = CO_MOD(iso_intv, factor);

                            bn = CO_DIVIDE_CEIL(bn, factor);
                            nse = bn * (rtn + 1);
                            iso_intv = iso_intv / factor;

                            if (rem && !framing)
                            {
                                framing = 1;
                                repeat = true;
                            }
                        }

                        while (nse > BLE_ISO_MAX_NSE)
                        {
                            uint8_t factor = CO_DIVIDE_CEIL(nse, BLE_ISO_MAX_NSE);
                            uint8_t rem = CO_MOD(iso_intv, factor);

                            bn = CO_DIVIDE_CEIL(bn, factor);
                            nse = bn * (rtn + 1);
                            iso_intv = iso_intv / factor;

                            if (rem && !framing)
                            {
                                framing = 1;
                                repeat = true;
                            }
                        }

                        if (iso_intv < BLE_ISO_MIN_INTERVAL)
                        {
                            status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                            break;
                        }
                    }
                    else // p_param->max_sdu <= eff_pld_size
                    {
                        max_pdu = max_sdu;

                        // Assume that the CIS is encrypted
                        max_pdu += MIC_LEN;

                        if (framing)
                        {
                            max_pdu += (BLE_ISOAL_SEG_HEADER_SIZE + BLE_ISOAL_TIME_OFFSET_SIZE);
                        }

                        ASSERT_ERR(max_pdu <= BLE_ISO_MAX_PAYLOAD_SIZE);

                        bn = 1;
                        nse = rtn + 1;

                        // If iso_intv does not equal sdu_interval
                        if (iso_intv < (sdu_interval / (SLOT_SIZE * 2)))
                        {
                            uint8_t factor = CO_DIVIDE_CEIL((sdu_interval / (SLOT_SIZE * 2)), iso_intv);
                            nse = CO_DIVIDE_CEIL(nse, factor);
                        }
                    }
                } while (repeat && (status == CO_ERROR_NO_ERROR));

                if (status != CO_ERROR_NO_ERROR)
                {
                    break;
                }

                if (j == 0) // Tx
                {
                    p_cis_param->tx_max_pdu = max_pdu;
                    p_cis_param->tx_bn = bn;
                }
                else // Rx
                {
                    p_cis_param->rx_max_pdu = max_pdu;
                    p_cis_param->rx_bn = bn;
                }

                p_cis_param->nse = co_max(nse, p_cis_param->nse);

                // If any direction in any CIS requires framed PDUs
                if ((cig_framing == 0) && framing)
                {
                    cig_framing = 1;

                    if ((i != 0) || (j != 0))
                    {
                        cig_repeat = true;
                        count++;
                    }
                }

                // Save the minimum ISO interval
                if ((iso_intv != 0) && (iso_intv < cig_iso_intv))
                {
                    if (cig_iso_intv != 0xFFFF)
                    {
                        cig_repeat = true;
                        count++;
                    }

                    cig_iso_intv = iso_intv;
                }

                ASSERT_ERR(count < 3);
            }

            if (status != CO_ERROR_NO_ERROR)
            {
                break;
            }

            // Compute packet duration over the air to have the minimum sub_evt duration
            p_cis_param->sub_evt_dur = lli_ci_se_dur_compute(p_cis_param->tx_max_pdu, p_cis_param->tx_rate, p_cis_param->rx_max_pdu, p_cis_param->rx_rate, true);

            // Check if the number of subevents can fit into the ISO interval
            if ((p_cis_param->sub_evt_dur * p_cis_param->nse) > (cig_iso_intv*SLOT_SIZE*2))
            {
                uint8_t rtn, bn;
                bool found = false;

                if ((p_cis_param->tx_bn * (p_cmd->params[i].rtn_m2s + 1)) >= (p_cis_param->rx_bn * (p_cmd->params[i].rtn_s2m + 1)))
                {
                    rtn = p_cmd->params[i].rtn_m2s;
                    bn = p_cis_param->tx_bn;
                }
                else
                {
                    rtn = p_cmd->params[i].rtn_s2m;
                    bn = p_cis_param->rx_bn;
                }


                while (!found && (p_cis_param->nse > bn) && (rtn > 0))
                {
                    p_cis_param->nse -= bn;
                    rtn--;

                    if ((p_cis_param->sub_evt_dur * p_cis_param->nse) <= (cig_iso_intv*SLOT_SIZE*2))
                    {
                        found = true;
                    }
                }

                if (!found)
                {
                    status = CO_ERROR_UNACCEPTABLE_CONN_PARAM;
                    break;
                }
            }
        }
    } while (cig_repeat && (status == CO_ERROR_NO_ERROR));

    if (status == CO_ERROR_NO_ERROR)
    {
        // Save computed parameters
        p_cig_param->framing = cig_framing;
        p_cig_param->iso_interval = cig_iso_intv;
    }

    return(status);
}

/*
 * HCI COMMAND HANDLERS
 ****************************************************************************************
 */

#if (BLE_CENTRAL)
int hci_le_set_cig_params_cmd_handler(const struct hci_le_set_cig_params_cmd *p_cmd, uint16_t opcode)
{
    // Allocate the command complete event to be sent to the host
    struct hci_le_set_cig_params_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode,
                                                                   hci_le_set_cig_params_cmd_cmp_evt);
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env *p_cig_env = NULL;

    do
    {
        uint8_t cursor;
        // Pointer to the CIS Channel information structure
        struct lli_cis_env   *p_cis_env;
        struct lli_cig_param* p_cig_param;

        // Check if feature is locally supported
        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // Check provided parameters
        if(   (p_cmd->cig_id > BLE_CIG_MAX_ID)                       || (p_cmd->packing >= ISO_PACKING_MAX)
           || (p_cmd->sdu_interval_m2s  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_m2s  > BLE_ISO_MAX_SDU_INTERVAL)
           || (p_cmd->sdu_interval_s2m  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_s2m  > BLE_ISO_MAX_SDU_INTERVAL)
           || (p_cmd->max_trans_latency_m2s < BLE_ISO_MIN_TRANS_LATENCY) || (p_cmd->max_trans_latency_m2s > BLE_ISO_MAX_TRANS_LATENCY)
           || (p_cmd->max_trans_latency_s2m < BLE_ISO_MIN_TRANS_LATENCY) || (p_cmd->max_trans_latency_s2m > BLE_ISO_MAX_TRANS_LATENCY)
           || (p_cmd->framing >= ISO_FRAME_MODE_MAX))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Mark that parameter are not supported - only UnFramed and Un-segmented SDU is supported
//        if(   (p_cmd->framing != ISO_UNFRAMED_MODE)
//           || (p_cmd->sdu_interval_m2s != p_cmd->sdu_interval_s2m)
//           || ((p_cmd->sdu_interval_m2s % (SLOT_SIZE*2)) != 0))
//        {
//            status = CO_ERROR_UNSUPPORTED;
//            break;
//        }

        // Get environment of the CIG
        p_cig_env = lli_cig_get(p_cmd->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

        // If Group does not exist
        if(p_cig_env == NULL)
        {
            // Allocate CIG environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env),
                                      (struct lli_group_env**) &(p_cig_env));

            // Check that CIG environment properly loaded, else exit
            if(status != CO_ERROR_NO_ERROR)
            {
                break;
            }
            else
            {
                co_list_init(&(p_cig_env->channels));
                p_cig_env->role           = ROLE_MASTER;
                p_cig_env->cig_id         = p_cmd->cig_id;
                p_cig_env->sub_interval   = 0;
                p_cig_env->bf_cis_present = 0;
                p_cig_env->bf_cis_busy    = 0;
                p_cig_env->bf_cis_en      = 0;
                p_cig_env->link_id        = BLE_INVALID_LINK_ID;
                p_cig_env->nb_cis         = 0;
            }
        }

        // check if it's possible to insert channel (stream busy or CIS_ID already present)
        if((p_cig_env->bf_cis_busy != 0) || (p_cig_env->bf_cis_en != 0))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        p_cig_param = &(p_cig_env->cig_param);


        // Browse all channels
        for(cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t tx_rate, rx_rate;
            // parameters needed to check group validity
            uint16_t sub_evt_dur;
            const struct hci_le_cis_param* p_params = &(p_cmd->params[cursor]);
            struct lli_cis_param* p_cis_param;

            // Check provided parameters
            if (   (p_params->cis_id > BLE_CIS_MAX_ID)
                || (p_params->max_sdu_m2s > BLE_ISO_MAX_SDU_SIZE) || (p_params->max_sdu_s2m > BLE_ISO_MAX_SDU_SIZE)
                || (p_params->rtn_m2s > BLE_CIS_MAX_RTN)          || (p_params->rtn_s2m > BLE_CIS_MAX_RTN))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Mark that parameter are not supported - only UnFramed and Un-segmented SDU is supported
//            if((p_params->max_sdu_m2s > BLE_ISO_MAX_PAYLOAD_SIZE) || (p_params->max_sdu_s2m > BLE_ISO_MAX_PAYLOAD_SIZE))
//            {
//                status = CO_ERROR_UNSUPPORTED;
//                break;
//            }

            // Check if PHY proposed are supported
            if(   ((p_params->phy_m2s & supp_phy_loc_msk) == 0) || ((p_params->phy_s2m & supp_phy_loc_msk) == 0)
                  // For the moment only one PHY is supported
               || (NB_ONE_BITS(p_params->phy_m2s) != 1)         || (NB_ONE_BITS(p_params->phy_s2m) != 1))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            tx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_m2s]];
            rx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_s2m]];

            // Compute packet duration over the air to have the minimum sub_evt duration
            sub_evt_dur = lli_ci_se_dur_compute(p_params->max_sdu_m2s, tx_rate, p_params->max_sdu_s2m, rx_rate, true);

            // check if cis_id already present in group
            if((p_cig_env->bf_cis_present & CO_BIT(p_params->cis_id)) == 0)
            {
                // check if a channels can be allocated
                if(lli_nb_chan_ava_get() == 0)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // Try to allocate a new CIS environment
                status = lli_cis_create(&p_cis_env);

                // check if a channels can be allocated
                if(status != CO_ERROR_NO_ERROR)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // put CIS into the queue
                co_list_push_back(&(p_cig_env->channels), &(p_cis_env->chan.hdr));
            }
            else
            {
                // search existing channel
                uint8_t cnt;
                for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
                {
                    if (   (LLI_CIS_ENV_GET(cnt, &p_cis_env) == CO_ERROR_NO_ERROR)
                        && (p_cis_env->grp_hdl == p_cig_env->group.hdl)
                        && (p_cis_env->cis_param.cis_id == p_params->cis_id))
                    {
                        break;
                    }
                }

                // channel not found
                if(cnt == BLE_ACTIVITY_MAX)
                {
                    status = CO_ERROR_UNSPECIFIED_ERROR;
                    break;
                }
            }

            p_cis_param = &(p_cis_env->cis_param);

            // Keep provided parameters
            p_cis_env->role              = ROLE_MASTER;
            p_cis_env->grp_hdl           = p_cig_env->group.hdl;
            p_cis_env->chan.framing      = p_cmd->framing;

            // These parameters do not change
            p_cis_param->cis_id          = p_params->cis_id;
            p_cis_param->cig_id          = p_cmd->cig_id;
            p_cis_param->tx_max_sdu      = p_params->max_sdu_m2s;
            p_cis_param->rx_max_sdu      = p_params->max_sdu_s2m;
            p_cis_param->tx_rate         = tx_rate;
            p_cis_param->rx_rate         = rx_rate;
            p_cig_param->tx_ft           = 1;
            p_cig_param->rx_ft           = 1;

            // These parameters are initialized here and will be updated later
            p_cis_param->tx_max_pdu      = p_params->max_sdu_m2s;
            p_cis_param->rx_max_pdu      = p_params->max_sdu_s2m;
            p_cis_param->tx_bn           = (p_params->max_sdu_m2s > 0 ? 1 : 0);
            p_cis_param->rx_bn           = (p_params->max_sdu_s2m > 0 ? 1 : 0);
            p_cis_param->nse             = co_max(p_cis_param->tx_bn * (p_params->rtn_m2s + 1),
                                                  p_cis_param->rx_bn * (p_params->rtn_s2m + 1));
            p_cis_param->sub_evt_dur     = sub_evt_dur;

//            {
//                // put all parameters in 0.1ms granularity
//                uint16_t buf_trans_latency_m2s = (p_cmd->max_trans_latency_m2s*10) / (p_cig_param->iso_interval*8);
//                uint16_t buf_trans_latency_s2m = (p_cmd->max_trans_latency_s2m*10) / (p_cig_param->iso_interval*8);
//
//                if(buf_trans_latency_m2s > 1)
//                {
//                    p_cig_param->tx_ft = co_min(buf_trans_latency_m2s, BLE_CIS_MAX_FT);
//                }
//
//                if(buf_trans_latency_s2m > 1)
//                {
//                    p_cig_param->rx_ft = co_min(buf_trans_latency_s2m, BLE_CIS_MAX_FT);
//                }
//            }

            // Set channel handle in the command complete event message
            p_evt->conhdl[cursor]                = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);

            // Add the new channel in the CIG
            p_cig_env->bf_cis_present   |= CO_BIT(p_params->cis_id);
            p_cig_env->nb_cis++;
        }

        if(status == CO_ERROR_NO_ERROR)
        {
            // Compute parameters for the CIG and all the CISes
            status = lli_cig_compute_params(p_cmd, p_cig_param, p_evt);
        }

        // an error occurs, stop the command
        if(status != CO_ERROR_NO_ERROR)
        {
            uint8_t nb_added_chan = cursor;
            // Browse all channels
            for(cursor = 0 ; cursor < nb_added_chan ; cursor++)
            {
                LLI_CIS_ENV_GET(BLE_CISHDL_TO_CHANHDL(p_evt->conhdl[cursor]), &p_cis_env);
                ASSERT_ERR(p_cis_env != NULL);

                // mark channel removed
                p_cig_env->bf_cis_present &= ~CO_BIT(p_cis_env->cis_param.cis_id);
                p_cig_env->nb_cis--;

                // Free the associated activity
                co_list_extract(&(p_cig_env->channels), &(p_cis_env->chan.hdr));
                llm_activity_free_set(p_cis_env->chan.hdl);
                // Clean the channel
                lli_chan_cleanup(p_cis_env->chan.hdl);
            }
            break;
        }

        // update scheduling parameters
        //p_cig_param->iso_interval          = (p_cmd->sdu_interval_m2s / (SLOT_SIZE * 2));
        p_cig_param->tx_sdu_interval       = p_cmd->sdu_interval_m2s;
        p_cig_param->rx_sdu_interval       = p_cmd->sdu_interval_s2m;
        //p_cig_param->framing               = p_cmd->framing;

        p_cig_env->packing                 = p_cmd->packing;

    } while(0);


    // Send the command complete event
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = (status == CO_ERROR_NO_ERROR) ? p_cmd->cis_count : 0;
    p_evt->status    = status;
    hci_send_2_host(p_evt);

    // if an error occurs, check if CIG environment should be cleaned-up
    if((status != CO_ERROR_NO_ERROR) && (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}




int hci_le_set_cig_params_test_cmd_handler(struct hci_le_set_cig_params_test_cmd *p_cmd, uint16_t opcode)
{
    // Allocate the command complete event to be sent to the host
    struct hci_le_set_cig_params_test_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_set_cig_params_test_cmd_cmp_evt);
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env* p_cig_env = NULL;
    struct lli_cig_param* p_cig_param;

    do
    {
        uint8_t cursor;
        // Pointer to the CIS Channel information structure
        struct lli_cis_env*   p_cis_env;

        uint8_t ft_m2s = 1;
        uint8_t ft_s2m = 1;
        uint32_t iso_interval_us = p_cmd->iso_interval * (SLOT_SIZE*2);

        // Check if feature is locally supported
        uint8_t supp_phy_loc_msk = PHY_1MBPS_BIT;
        SETB(supp_phy_loc_msk, PHY_2MBPS, BLE_PHY_2MBPS_SUPPORT);
        SETB(supp_phy_loc_msk, PHY_CODED, BLE_PHY_CODED_SUPPORT);

        // Check provided parameters
        if(   (p_cmd->cig_id > BLE_CIG_MAX_ID)                       || (p_cmd->iso_interval > BLE_ISO_MAX_INTERVAL)
           || (p_cmd->sdu_interval_m2s  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_m2s  > BLE_ISO_MAX_SDU_INTERVAL)
           || (p_cmd->sdu_interval_s2m  < BLE_ISO_MIN_SDU_INTERVAL)  || (p_cmd->sdu_interval_s2m  > BLE_ISO_MAX_SDU_INTERVAL)
           || (p_cmd->ft_s2m < BLE_CIS_MIN_FT)                       || (p_cmd->ft_s2m > BLE_CIS_MAX_FT)
           || (p_cmd->ft_m2s < BLE_CIS_MIN_FT)                       || (p_cmd->ft_m2s > BLE_CIS_MAX_FT)
           || (p_cmd->sca > SCA_20PPM)
           || (p_cmd->packing >= ISO_PACKING_MAX)                    || (p_cmd->framing >= ISO_FRAME_MODE_MAX))
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        if(p_cmd->framing == ISO_UNFRAMED_MODE)
        {
            // ISO interval and SDU interval must be multiple
            if(   (CO_MOD(iso_interval_us, p_cmd->sdu_interval_m2s) != 0)
               || (CO_MOD(iso_interval_us, p_cmd->sdu_interval_s2m) != 0))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }
        }

        // Get environment of the CIG
        p_cig_env = lli_cig_get(p_cmd->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

        // If Group does not exist
        if(p_cig_env == NULL)
        {
            // Allocate CIG environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env),
                                      (struct lli_group_env**) &(p_cig_env));

            // Check that CIG environment properly loaded, else exist
            if(status != CO_ERROR_NO_ERROR)
            {
                break;
            }
            else
            {
                co_list_init(&(p_cig_env->channels));
                p_cig_env->role           = ROLE_MASTER;
                p_cig_env->cig_id         = p_cmd->cig_id;
                p_cig_env->sub_interval   = 0;
                p_cig_env->bf_cis_present = 0;
                p_cig_env->bf_cis_busy    = 0;
                p_cig_env->bf_cis_en      = 0;
                p_cig_env->link_id        = BLE_INVALID_LINK_ID;
                p_cig_env->nb_cis         = 0;
            }
        }

        // check if it's possible to insert channel (stream busy or CIS_ID already present)
        if((p_cig_env->bf_cis_busy != 0) || (p_cig_env->bf_cis_en != 0))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Browse all channels
        for(cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t tx_rate, rx_rate;
            // parameters needed to check group validity
            uint32_t total_evt_dur = 0;
            uint16_t sub_evt_dur;
            struct hci_le_cis_test_param* p_params = &(p_cmd->params[cursor]);
            struct lli_cis_param* p_cis_param;


            // Check provided parameters
            if (   (p_params->cis_id > BLE_CIS_MAX_ID)
                || (p_params->max_pdu_m2s > BLE_ISO_MAX_PAYLOAD_SIZE)  || (p_params->max_pdu_s2m > BLE_ISO_MAX_PAYLOAD_SIZE)
                || (p_params->max_sdu_m2s > BLE_ISO_MAX_SDU_SIZE)      || (p_params->max_sdu_s2m > BLE_ISO_MAX_SDU_SIZE)
                || (p_params->nse < BLE_ISO_MIN_NSE)                   || (p_params->nse > BLE_ISO_MAX_NSE)
                || (p_params->bn_s2m > BLE_CIS_MAX_BN)                 || (p_params->bn_m2s > BLE_CIS_MAX_BN)
                || (p_params->bn_s2m > p_params->nse)                  || (p_params->bn_m2s > p_params->nse))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // Force SDU and PDU size to 0 if BN - 0
            if(p_params->bn_m2s == 0)
            {
                p_params->max_pdu_m2s = 0;
                p_params->max_sdu_m2s = 0;
            }

            // Force SDU and PDU size to 0 if BN - 0
            if(p_params->bn_s2m == 0)
            {
                p_params->max_pdu_s2m = 0;
                p_params->max_sdu_s2m = 0;
            }

            // check size of parameters
            if(p_cmd->framing == ISO_UNFRAMED_MODE)
            {
                // per SDU interval a rounded number of burst is required TODO [FBE] typo
                if(   (CO_MOD(p_params->bn_m2s, iso_interval_us / p_cmd->sdu_interval_m2s) != 0)
                   || (CO_MOD(p_params->bn_m2s, iso_interval_us / p_cmd->sdu_interval_m2s) != 0))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
                else
                {
                    uint16_t max_sdu_m2s = p_params->max_pdu_m2s * ((p_params->bn_m2s * p_cmd->sdu_interval_m2s) / iso_interval_us);
                    uint16_t max_sdu_s2m = p_params->max_pdu_s2m * ((p_params->bn_s2m * p_cmd->sdu_interval_s2m) / iso_interval_us);

                    // check that maximum SDU size requested can be transmitted
                    if((p_params->max_sdu_m2s > max_sdu_m2s) || (p_params->max_sdu_s2m > max_sdu_s2m))
                    {
                        status = CO_ERROR_INVALID_HCI_PARAM;
                        break;
                    }
                }
            }
            else
            {
                uint16_t max_sdu_m2s = (  (p_params->max_pdu_m2s * p_params->bn_m2s * p_cmd->sdu_interval_m2s)
                                        - (BLE_ISOAL_SEG_HEADER_SIZE * p_params->bn_m2s)) / iso_interval_us
                                     - BLE_ISOAL_TIME_OFFSET_SIZE;

                uint16_t max_sdu_s2m = (  (p_params->max_pdu_s2m * p_params->bn_s2m * p_cmd->sdu_interval_s2m)
                                        - (BLE_ISOAL_SEG_HEADER_SIZE * p_params->bn_s2m)) / iso_interval_us
                                     - BLE_ISOAL_TIME_OFFSET_SIZE;

                // TODO [FBE] this is a small sanity check to ensure that at least a SDU can be transmitted,
                // but it does not work with all segmentation capabilities
                if((p_params->max_sdu_m2s > max_sdu_m2s) || (p_params->max_sdu_s2m > max_sdu_s2m))
                {
                    status = CO_ERROR_INVALID_HCI_PARAM;
                    break;
                }
            }


            // Check if PHY proposed are supported
            if(((p_params->phy_m2s & supp_phy_loc_msk) == 0) || ((p_params->phy_s2m & supp_phy_loc_msk) == 0))
            {
                status = CO_ERROR_UNSUPPORTED;
                break;
            }

            // Only one PHY must be configured
            if((NB_ONE_BITS(p_params->phy_m2s) != 1)         || (NB_ONE_BITS(p_params->phy_s2m) != 1))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            tx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_m2s]];
            rx_rate = co_phy_to_rate[co_phy_mask_to_value[p_params->phy_s2m]];

            // Compute packet duration over the air to have the minimum sub_evt duration
            sub_evt_dur = lli_ci_se_dur_compute(p_params->max_pdu_m2s, tx_rate, p_params->max_pdu_s2m, rx_rate, true);

            // compute the event duration and check if parameters can fit
            total_evt_dur = ((sub_evt_dur) * (p_params->nse-1)) + sub_evt_dur;

            // check if all sub event can enter into the channel interval
            if(total_evt_dur > ((uint32_t)p_cmd->iso_interval * SLOT_SIZE*2))
            {
                status = CO_ERROR_INVALID_HCI_PARAM;
                break;
            }

            // check if cis_id already present in group
            if((p_cig_env->bf_cis_present & CO_BIT(p_params->cis_id)) == 0)
            {
                // check if a channels can be allocated
                if(lli_nb_chan_ava_get() == 0)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // Try to allocate a new CIS environment
                status = lli_cis_create(&p_cis_env);

                // check if a channels can be allocated
                if(status != CO_ERROR_NO_ERROR)
                {
                    status = CO_ERROR_CON_LIMIT_EXCEED;
                    break;
                }

                // put CIS into the queue
                co_list_push_back(&(p_cig_env->channels), &(p_cis_env->chan.hdr));
            }
            else
            {
                // search existing channel
                uint8_t cnt;
                for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
                {
                    if (   (LLI_CIS_ENV_GET(cnt, &p_cis_env) == CO_ERROR_NO_ERROR)
                        && (p_cis_env->grp_hdl == p_cig_env->group.hdl)
                        && (p_cis_env->cis_param.cis_id == p_params->cis_id))
                    {
                        break;
                    }
                }

                // channel not found
                if(cnt == BLE_ACTIVITY_MAX)
                {
                    status = CO_ERROR_UNSPECIFIED_ERROR;
                    break;
                }
            }

            p_cis_param = &(p_cis_env->cis_param);

            // Keep provided parameters
            p_cis_env->role                      = ROLE_MASTER;
            p_cis_env->grp_hdl                   = p_cig_env->group.hdl;
            p_cis_env->chan.framing              = p_cmd->framing;

            // CIS parameters
            p_cis_param->cis_id                  = p_params->cis_id;
            p_cis_param->cig_id                  = p_cmd->cig_id;
            p_cis_param->nse                     = p_params->nse;
            p_cis_param->tx_max_sdu              = p_params->max_sdu_m2s;
            p_cis_param->rx_max_sdu              = p_params->max_sdu_s2m;
            p_cis_param->tx_max_pdu              = p_params->max_pdu_m2s;
            p_cis_param->rx_max_pdu              = p_params->max_pdu_s2m;
            p_cis_param->tx_bn                   = p_params->bn_m2s;
            p_cis_param->rx_bn                   = p_params->bn_s2m;
            p_cis_param->tx_rate                 = tx_rate;
            p_cis_param->rx_rate                 = rx_rate;
            p_cis_param->sub_evt_dur             = sub_evt_dur;

            ft_m2s                               = p_cmd->ft_m2s;
            ft_s2m                               = p_cmd->ft_s2m;

            // Set channel handle in the command complete event message
            p_evt->conhdl[cursor]                = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);

            // Add the new channel in the CIG
            p_cig_env->bf_cis_present   |= CO_BIT(p_params->cis_id);
            p_cig_env->nb_cis++;
        }

        // an error occurs, stop the command
        if(status != CO_ERROR_NO_ERROR)
        {
            uint8_t nb_added_chan = cursor;
            // Browse all channels
            for(cursor = 0 ; cursor < nb_added_chan ; cursor++)
            {
                LLI_CIS_ENV_GET(BLE_CISHDL_TO_CHANHDL(p_evt->conhdl[cursor]), &p_cis_env);
                ASSERT_ERR(p_cis_env != NULL);

                // mark channel removed
                p_cig_env->bf_cis_present &= CO_BIT(p_cis_env->cis_param.cis_id);
                p_cig_env->nb_cis--;

                // Free the associated activity
                co_list_extract(&(p_cig_env->channels), &(p_cis_env->chan.hdr));
                llm_activity_free_set(p_cis_env->chan.hdl);
                // Clean the channel
                lli_chan_cleanup(p_cis_env->chan.hdl);
            }
            break;
        }

        p_cig_param = &(p_cig_env->cig_param);

        // update scheduling parameters
        p_cig_param->iso_interval        = p_cmd->iso_interval;
        p_cig_param->tx_sdu_interval     = p_cmd->sdu_interval_m2s;
        p_cig_param->rx_sdu_interval     = p_cmd->sdu_interval_s2m;
        p_cig_param->framing             = p_cmd->framing;
        p_cig_env->packing               = p_cmd->packing;
        p_cig_param->tx_ft               = ft_m2s;
        p_cig_param->rx_ft               = ft_s2m;

    } while(0);

    // Send the command complete event
    p_evt->cig_id = p_cmd->cig_id;
    p_evt->cis_count = (status == CO_ERROR_NO_ERROR) ? p_cmd->cis_count : 0;
    p_evt->status    = status;
    hci_send_2_host(p_evt);

    // if an error occurs, check if CIG environment should be cleaned-up
    if((status != CO_ERROR_NO_ERROR) && (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_create_cis_cmd_handler(struct hci_le_create_cis_cmd const *p_cmd, uint16_t opcode)
{
    int msg_status = KE_MSG_CONSUMED;
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, 0, opcode, hci_cmd_stat_event);
    // Initialize returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    uint8_t cursor = 0;
    // Pointer to the CIS Channel information structure
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env = NULL;

    struct le_features local_feats;
    uint8_t byte_nb = BLE_FEAT_ISO_CHANNELS_HOST_SUPPORT/8;
    uint8_t bit_nb = CO_MOD(BLE_FEAT_ISO_CHANNELS_HOST_SUPPORT, 8);

    // Read local features
    llm_le_features_get(&local_feats);

    // Check if the Isochronous Channels (Host Support) feature bit is set
    if (!(local_feats.feats[byte_nb] & (1 << bit_nb)))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    // Check if command not already under execution
    else if(lli_ci_env.p_create_cmd != NULL)
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        for(cursor = 0 ; cursor < p_cmd->cis_count ; cursor++)
        {
            uint8_t  chan_hdl = BLE_CISHDL_TO_CHANHDL(p_cmd->params[cursor].cis_conhdl);
            struct lli_cis_param* p_cis_param;
            uint8_t  role;

            // Retrieve CIS environment
            status = LLI_CIS_ENV_GET(chan_hdl, &p_cis_env);

            if (status != CO_ERROR_NO_ERROR)
            {
                // Provided channel does not exists
                break;
            }

            // check if channel is configured for master role
            if (p_cis_env->role != ROLE_MASTER)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Retrieve CIG environment
            status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

            if (status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                ASSERT_ERR(0);
                // Provided channel does not exists
                break;
            }

            p_cis_param = &(p_cis_env->cis_param);

            // Check that no negotiation is on-going
            if ((p_cig_env->bf_cis_busy & CO_BIT(p_cis_param->cis_id)) != 0)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Check that channel is not already enabled
            if ((p_cig_env->bf_cis_en & CO_BIT(p_cis_param->cis_id)) != 0)
            {
                status = CO_ERROR_CON_ALREADY_EXISTS;
                break;
            }

            // Keep the provided connection handle
            p_cis_env->link_id = BLE_CONHDL_TO_LINKID(p_cmd->params[cursor].acl_conhdl);

            // retrieve PHY used by channel
            status = llc_role_get(p_cis_env->link_id, &role);
            if (status != CO_ERROR_NO_ERROR)
            {
                // Provided channel does not exists
                break;
            }

            // check if role is master
            if(role != MASTER_ROLE)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }

            // Update channel state
            p_cig_env->bf_cis_busy |= CO_BIT(p_cis_param->cis_id);
        }
    }

    // Send the command complete event
    p_evt->status   = status;
    hci_send_2_host(p_evt);

    if(status == CO_ERROR_NO_ERROR)
    {
        // keep command to execute it later
        lli_ci_env.p_create_cmd = p_cmd;
        lli_ci_env.cursor = 0;
        msg_status = KE_MSG_NO_FREE;

        // Continue CIS creation
        lli_cis_create_cont();
    }
    // if an error occurs, clean-up busy information
    else
    {
        uint8_t nb_cis = cursor;

        for(cursor = 0 ; cursor < nb_cis ; cursor++)
        {
            uint8_t  chan_hdl = BLE_CISHDL_TO_CHANHDL(p_cmd->params[cursor].cis_conhdl);
            struct lli_cis_param* p_cis_param;

            // Retrieve CIS environment
            if (LLI_CIS_ENV_GET(chan_hdl, &p_cis_env) != CO_ERROR_NO_ERROR)
            {
                continue;
            }

            // Retrieve CIG environment
            LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

            p_cis_param = &(p_cis_env->cis_param);

            // Update channel state
            p_cig_env->bf_cis_busy &= ~CO_BIT(p_cis_param->cis_id);
        }
    }

    // Message can be consumed
    return (msg_status);
}
#endif //(BLE_CENTRAL)

int hci_disconnect_cis_cmd_handler(struct hci_disconnect_cmd const *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, p_param->conhdl, opcode, hci_cmd_stat_event);
    // Command status
    uint8_t status;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env = NULL;
    uint8_t chan_hdl = BLE_CISHDL_TO_CHANHDL(p_param->conhdl);

    // Check if CIS exists
    status = LLI_CIS_ENV_GET(chan_hdl, &p_cis_env);

    if (status == CO_ERROR_NO_ERROR)
    {
        // Retrieve CIG environment
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_UNSPECIFIED_ERROR;
            ASSERT_ERR(0);
        }
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);

        // Check that no negotiation are on-going
        if (   (p_cig_env->bf_cis_busy != 0)
            // Check that channel is enabled
            || ((p_cig_env->bf_cis_en & CO_BIT(p_cis_param->cis_id)) == 0))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
        else
        {
            // Keep the reason
            p_cis_env->reason = p_param->reason;

            // Provide the request to the LLC CIS module
            llc_cis_stop_req(p_cis_env->link_id, p_cis_env->chan.hdl, p_cis_env->reason);

            // Mark that a procedure is on-going on stream
            p_cig_env->bf_cis_busy |= CO_BIT(p_cis_param->cis_id);
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_remove_cig_cmd_handler(struct hci_le_remove_cig_cmd const *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_le_remove_cig_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, 0, opcode, hci_le_remove_cig_cmd_cmp_evt);
    // Command status
    uint8_t status = CO_ERROR_NO_ERROR;

    struct lli_cig_env *p_cig_env;

    p_cig_env = lli_cig_get(p_param->cig_id, ROLE_MASTER, BLE_INVALID_LINK_ID);

    // Check that nb value is not higher than array size
    if ((p_cig_env == NULL) || (p_cig_env->bf_cis_en != 0) || (p_cig_env->bf_cis_busy != 0))
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }
    else
    {
        while(!co_list_is_empty(&(p_cig_env->channels)))
        {
            // Pointer to the indicated CIS environment
            struct lli_cis_env *p_cis_env = (struct lli_cis_env*) co_list_pop_front(&(p_cig_env->channels));

            // Check if datapath must be removed
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX]))
            {
                lli_cis_data_path_remove(&(p_cis_env->chan), ISO_SEL_RX);
            }
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX]))
            {
                lli_cis_data_path_remove(&(p_cis_env->chan), ISO_SEL_TX);
            }

            // Free the associated activity
            llm_activity_free_set(p_cis_env->chan.hdl);

            // Clean the channel
            lli_chan_cleanup(p_cis_env->chan.hdl);
        }

        // Clean-up group environment
        lli_group_cleanup(p_cig_env->group.hdl);
    }

    // Send the command complete event
    p_evt->status = status;
    p_evt->cig_id = p_param->cig_id;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

#if (BLE_PERIPHERAL)
int hci_le_accept_cis_req_cmd_handler(struct hci_le_accept_cis_req_cmd const *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_cmd_stat_event *p_evt = KE_MSG_ALLOC(HCI_CMD_STAT_EVENT, p_param->conhdl, opcode, hci_cmd_stat_event);
    // Command status
    uint8_t status;
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env;
    uint8_t chan_hdl = BLE_CISHDL_TO_CHANHDL(p_param->conhdl);

    // Check if CIS exists
    status = LLI_CIS_ENV_GET(chan_hdl, &p_cis_env);

    if(status == CO_ERROR_NO_ERROR)
    {
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);

        p_cis_param = &(p_cis_env->cis_param);

        // Check if command reception is expected
        if (   ((p_cig_env->bf_cis_busy   & CO_BIT(p_cis_param->cis_id)) != 0)
            && ((p_cig_env->bf_cis_en & CO_BIT(p_cis_param->cis_id)) == 0))
        {
            // Provide host response to the LLC CIS module
            llc_cis_create_rsp(p_cis_env->link_id, CO_ERROR_NO_ERROR);

            // mark that host request has been performed
            p_cig_env->bf_cis_busy &= ~CO_BIT(p_cis_param->cis_id);
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

int hci_le_reject_cis_req_cmd_handler(struct hci_le_reject_cis_req_cmd *p_param, uint16_t opcode)
{
    // Allocate the status event to be sent to the host
    struct hci_basic_conhdl_cmd_cmp_evt *p_evt = KE_MSG_ALLOC(HCI_CMD_CMP_EVENT, p_param->conhdl, opcode, hci_basic_conhdl_cmd_cmp_evt);
    // Command status
    uint8_t status;
    // Pointer to the indicated CIS environment
    struct lli_cis_env   *p_cis_env;
    struct lli_cig_env *p_cig_env;
    uint8_t chan_hdl = BLE_CISHDL_TO_CHANHDL(p_param->conhdl);

    // Check if CIS exists
    status = LLI_CIS_ENV_GET(chan_hdl, &p_cis_env);

    if(status == CO_ERROR_NO_ERROR)
    {
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);
    }

    if (status == CO_ERROR_NO_ERROR)
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);

        // Check if command reception is expected
        if (   ((p_cig_env->bf_cis_busy   & CO_BIT(p_cis_param->cis_id)) != 0)
            && ((p_cig_env->bf_cis_en & CO_BIT(p_cis_param->cis_id)) == 0))
        {
            if(p_param->reason == CO_ERROR_NO_ERROR)
            {
                p_param->reason = CO_ERROR_OPERATION_CANCELED_BY_HOST;
            }

            // Provide host response to the LLC CIS module
            llc_cis_create_rsp(p_cis_env->link_id, p_param->reason);

            // mark that procedure is over for the CIG and CIS
            p_cig_env->bf_cis_present   &= ~CO_BIT(p_cis_param->cis_id);
            p_cig_env->bf_cis_busy &= ~CO_BIT(p_cis_param->cis_id);
            p_cig_env->nb_cis--;

            // check if CIG should be removed
            if((p_cig_env->nb_cis == 0))
            {
                lli_group_cleanup(p_cig_env->group.hdl);
            }

            // Free the associated activity
            llm_activity_free_set(p_cis_env->chan.hdl);

            // Finally, clean the channel
            lli_chan_cleanup(p_cis_env->chan.hdl);
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    }

    // Set status and send the complete event
    p_evt->status = status;
    p_evt->conhdl = p_param->conhdl;
    hci_send_2_host(p_evt);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}
#endif //(BLE_PERIPHERAL)

/**
 ****************************************************************************************
 * @brief Handles the CIS stop indication message.
 ****************************************************************************************
 */
int lld_cis_stop_ind_handler(ke_msg_id_t const msgid,
                             struct lld_cis_stop_ind *p_param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    do
    {
        // Pointer to the CIS Channel information structure
        struct lli_cis_env   *p_cis_env;
        struct lli_cig_env *p_cig_env;
        struct lli_cis_param* p_cis_param;
        // Reason
        uint8_t reason;

        if (   (LLI_CIS_ENV_GET(p_param->chan_hdl, &p_cis_env)  != CO_ERROR_NO_ERROR)
            || (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) != CO_ERROR_NO_ERROR))
        {
            // Provided channel does not exist, drop the message
            ASSERT_INFO(0, p_param->chan_hdl , p_param->reason);
            break;
        }

        // If CIS terminated due to MIC failure, close the associated BLE link (and all other associated CISes)
        if(p_param->reason == CO_ERROR_TERMINATED_MIC_FAILURE)
        {
            // Check other
            llc_disconnect(p_cis_env->link_id, CO_ERROR_TERMINATED_MIC_FAILURE, true);
        }

        p_cis_param = &(p_cis_env->cis_param);

        // Check if reception of message was expected
        if ((p_cig_env->bf_cis_busy & CO_BIT(p_cis_param->cis_id)) != 0)
        {
            reason = p_cis_env->reason;
        }
        else
        {
            reason = p_param->reason;
        }

        // Clean the connection handle
        p_cis_env->link_id = BLE_INVALID_LINK_ID;

        // Send HCI LE CIS Disconnection Complete Event
        lli_cis_disc_cmp_evt_send(p_cis_env, reason);

        p_cig_env->bf_cis_busy &= ~CO_BIT(p_cis_param->cis_id);
        p_cig_env->bf_cis_en   &= ~CO_BIT(p_cis_param->cis_id);

        // On slave side, the channel is considered removed and can no more be reused
        if (p_cis_env->role == ROLE_SLAVE)
        {
            // Check if datapath must be removed
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_RX]))
            {
                lli_cis_data_path_remove(&(p_cis_env->chan), ISO_SEL_RX);
            }
            if (!data_path_is_disabled(p_cis_env->p_dp[ISO_SEL_TX]))
            {
                lli_cis_data_path_remove(&(p_cis_env->chan), ISO_SEL_TX);
            }

            // Remove channel from CIG
            co_list_extract(&(p_cig_env->channels), &(p_cis_env->chan.hdr));

            p_cig_env->bf_cis_present   &= ~CO_BIT(p_cis_param->cis_id);
            p_cig_env->nb_cis--;

            // Free the associated activity
            llm_activity_free_set(p_cis_env->chan.hdl);
            // Clean the channel
            lli_chan_cleanup(p_cis_env->chan.hdl);

            if(p_cig_env->nb_cis == 0)
            {
                lli_group_cleanup(p_cig_env->group.hdl);
            }
        }
    } while (0);

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

/**
 ****************************************************************************************
 * @brief Handles the CIS established indication message.
 ****************************************************************************************
 */
int lld_cis_estab_ind_handler(ke_msg_id_t const msgid,
                             struct lld_cis_estab_ind *p_param,
                             ke_task_id_t const dest_id, ke_task_id_t const src_id)
{
    // Pointer to the CIS Channel information structure
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env;

    if (   (LLI_CIS_ENV_GET(p_param->chan_hdl, &p_cis_env)  == CO_ERROR_NO_ERROR)
        && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        // Conclude negotiation at Connection level (handled by LLC CIS module)
        llc_cis_established(p_cis_env->link_id, p_param->status);
    }
    else
    {
        ASSERT_INFO(0, 0, p_param->chan_hdl);
    }

    // Message can be consumed
    return (KE_MSG_CONSUMED);
}

/*
 * GLOBAL FUNCTIONS DEFINITION
 ****************************************************************************************
 */


void lli_ci_init(uint8_t init_type)
{
    switch (init_type)
    {
        case RWIP_INIT:
        {
            // initialize structure
            lli_ci_env.p_create_cmd = NULL;
        }
        break;

        case RWIP_RST:
        {
            // free CIS creation operation if exists
            if(lli_ci_env.p_create_cmd != NULL)
            {
                ke_msg_free(ke_param2msg(lli_ci_env.p_create_cmd));
                lli_ci_env.p_create_cmd = NULL;
            }
        }
        // No break

        case RWIP_1ST_RST:
        {
            lli_ci_env.cb_start_evt = NULL;
        }
        break;

        default:
        {
            // Do nothing
        }
        break;
    }
}

void lli_ci_param_get(uint8_t chan_hdl, struct lli_cig_param ** pp_cig_param, struct lli_cis_param ** pp_cis_param)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if (   (LLI_CIS_ENV_GET(chan_hdl, &p_cis_env)           == CO_ERROR_NO_ERROR)
        && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        if (pp_cig_param != NULL)
            *pp_cig_param = &(p_cig_env->cig_param);

        if (pp_cis_param != NULL)
            *pp_cis_param = &(p_cis_env->cis_param);
    }
}

uint8_t lli_cis_chan_hdl_get(uint8_t link_id, uint8_t cig_id, uint8_t cis_id)
{
    // Allocated channel handle (match with activity index to be used)
    uint8_t chan_hdl = BLE_INVALID_CHANHDL;
    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
    {
        struct lli_cis_env *p_cis_env = NULL;

        if (   (LLI_CIS_ENV_GET(cnt, &p_cis_env) == CO_ERROR_NO_ERROR)
            && (p_cis_env->link_id          == link_id)
            && (p_cis_env->cis_param.cig_id == cig_id)
            && (p_cis_env->cis_param.cis_id == cis_id))
        {
            // Available channel handle has been found, no need to continue
            chan_hdl = (uint16_t)cnt;
            break;
        }
    }

    // Return the allocated channel handle
    return (chan_hdl);
}

bool lli_cis_is_present(uint8_t link_id)
{
    // Counter
    uint8_t cnt;

    for (cnt = 0; cnt < BLE_ACTIVITY_MAX; cnt++)
    {
        struct lli_cis_env *p_cis_env = NULL;

        if (   (LLI_CIS_ENV_GET(cnt, &p_cis_env) == CO_ERROR_NO_ERROR)
            && (p_cis_env->link_id          == link_id) )
        {
            break;
        }
    }

    // Return the allocated channel handle
    return (cnt < BLE_ACTIVITY_MAX);
}

void lli_cis_link_stop_ind(uint8_t link_id, struct lli_chan_env* p_iso_env, uint8_t reason)
{
    struct lli_cis_env   *p_cis_env   = (struct lli_cis_env *)p_iso_env;
    struct lli_cig_env *p_cig_env = NULL;

    if(LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR)
    {
        // check that current channel is enabled
        if (   (p_cis_env->link_id == link_id)
            && ((p_cig_env->bf_cis_en & CO_BIT(p_cis_env->cis_param.cis_id)) != 0))
        {
            // Disconnection reason
            p_cis_env->reason = reason;

            // Mark that a procedure is on-going on CIG
            p_cig_env->bf_cis_busy |= CO_BIT(p_cis_env->cis_param.cis_id);

            // Stop the CIS driver
            lld_cis_stop(p_cis_env->chan.hdl);
        }
    }
}

#if (BLE_PERIPHERAL)
uint8_t lli_cis_create_req(uint8_t link_id, struct ll_cis_req *p_pdu, uint8_t framing, uint8_t *p_chan_hdl)
{
    // Returned status
    uint8_t status = CO_ERROR_NO_ERROR;
    struct lli_cig_env *p_cig_env = NULL;

    do
    {
        // Channel handle allocated for the new channel
        uint8_t chan_hdl;
        // Pointer to allocated CIS environment
        struct lli_cis_env *p_cis_env;
        struct lli_cig_param* p_cig_param;
        struct lli_cis_param* p_cis_param;

        // Get environement of the stream
        p_cig_env = lli_cig_get(p_pdu->cig_id, ROLE_SLAVE, link_id);
        if(p_cig_env == NULL)
        {
            // Allocate stream environment
            status = lli_group_create(LLI_ISO_GROUP_CIG, sizeof(struct lli_cig_env),
                                      (struct lli_group_env**) &(p_cig_env));

            // Check that stream environment properly loaded, else exist
            if(status != CO_ERROR_NO_ERROR)
            {
                status = CO_ERROR_UNSPECIFIED_ERROR;
                break;
            }
            else
            {
                p_cig_param = &(p_cig_env->cig_param);
                p_cig_env->role                  = ROLE_SLAVE;
                p_cig_env->cig_id                = p_pdu->cig_id;
                p_cig_env->sub_interval          = 0;
                p_cig_env->bf_cis_present        = 0;
                p_cig_env->bf_cis_busy           = 0;
                p_cig_env->bf_cis_en             = 0;
                p_cig_env->link_id               = link_id;
                p_cig_env->nb_cis                = 0;
                co_list_init(&(p_cig_env->channels));

                p_cig_param->iso_interval        = p_pdu->iso_interval;
                p_cig_param->tx_sdu_interval     = p_pdu->sdu_interval_s2m;
                p_cig_param->rx_sdu_interval     = p_pdu->sdu_interval_m2s;
                p_cig_param->framing             = framing;
                p_cig_param->tx_ft               = p_pdu->ft_s2m;
                p_cig_param->rx_ft               = p_pdu->ft_m2s;

            }
        }
        else
        {
            p_cig_param = &(p_cig_env->cig_param);


            // Check that for this connection handle there is not already a CIS
            // with the same cig_id/cis_id
            if ((p_cig_env->bf_cis_present & CO_BIT(p_pdu->cis_id)) != 0)
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }

            // check that all channel interval of the CIG are equals
            if (   (p_cig_param->iso_interval    != p_pdu->iso_interval)
                || (p_cig_param->tx_sdu_interval != p_pdu->sdu_interval_s2m)
                || (p_cig_param->rx_sdu_interval != p_pdu->sdu_interval_m2s)
                || (p_cig_param->tx_ft           != p_pdu->ft_s2m)
                || (p_cig_param->rx_ft           != p_pdu->ft_m2s))
            {
                status = CO_ERROR_INVALID_LMP_PARAM;
                break;
            }
        }

        // Try to allocate a new CIS environment
        status = lli_cis_create(&p_cis_env);

        // Stop operation is not possible to allocate memory
        if (status != CO_ERROR_NO_ERROR)
        {
            status = CO_ERROR_UNSPECIFIED_ERROR;
            break;
        }

        // Set connection handle
        p_cis_env->link_id            = link_id;
        p_cis_env->role               = ROLE_SLAVE;
        p_cis_env->grp_hdl            = p_cig_env->group.hdl;
        p_cis_env->chan.framing       = framing;

        chan_hdl                      = p_cis_env->chan.hdl;

        // Store received CIS Channel parameters
        p_cis_param = &(p_cis_env->cis_param);
        p_cis_param->cig_id           = p_pdu->cig_id;
        p_cis_param->cis_id           = p_pdu->cis_id;
        p_cis_param->nse              = p_pdu->nse;
        p_cis_param->tx_rate          = p_pdu->phy_s2m;
        p_cis_param->tx_max_pdu       = p_pdu->max_pdu_s2m;
        p_cis_param->tx_max_sdu       = p_pdu->max_sdu_s2m;
        p_cis_param->tx_bn            = GETF(p_pdu->bn, LLCP_CIS_BN_S2M);
        p_cis_param->rx_rate          = p_pdu->phy_m2s;
        p_cis_param->rx_max_pdu       = p_pdu->max_pdu_m2s;
        p_cis_param->rx_max_sdu       = p_pdu->max_sdu_m2s;
        p_cis_param->rx_bn            = GETF(p_pdu->bn, LLCP_CIS_BN_M2S);
        p_cis_param->sub_interval     = p_pdu->sub_interval;


        // Send HCI LE CIS Channel Request Event to the Host
        lli_cis_request_evt_send(p_cis_env);

        // mark that procedure is on-going for the stream and channel present
        p_cig_env->bf_cis_present |= CO_BIT(p_pdu->cis_id);
        p_cig_env->bf_cis_busy    |= CO_BIT(p_pdu->cis_id);
        p_cig_env->nb_cis++;

        // Return the channel handle
        *p_chan_hdl = chan_hdl;
    } while (0);

    if((status != CO_ERROR_NO_ERROR)&& (p_cig_env != NULL) && (p_cig_env->nb_cis == 0))
    {
        lli_group_cleanup(p_cig_env->group.hdl);
    }

    return (status);
}
#endif //(BLE_PERIPHERAL)

void lli_cis_create_nego_end(uint8_t chan_hdl, uint8_t status, bool trigger_hci_evt)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if (   (LLI_CIS_ENV_GET(chan_hdl, &p_cis_env)           == CO_ERROR_NO_ERROR)
        && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);

        if(trigger_hci_evt)
        {
            // Send HCI CIS Enable Complete Event to the Host
            lli_cis_established_evt_send(p_cig_env, p_cis_env, status,
                                         ((p_cis_env->role == ROLE_MASTER) ? p_cis_param->tx_rate : p_cis_param->rx_rate),
                                         ((p_cis_env->role == ROLE_MASTER) ? p_cis_param->rx_rate : p_cis_param->tx_rate));
        }

        #if BLE_PWR_CTRL
        {
            uint8_t con_tx_rate;
            // Get connection tx rates
            if (CO_ERROR_NO_ERROR == lld_con_tx_rate_get(p_cis_env->link_id, &con_tx_rate))
            {
                // If the CIS tx rate differs, signal power change indication
                if (p_cis_param->tx_rate != con_tx_rate)
                {
                    struct lld_cis_pwr_change_ind *msg = KE_MSG_ALLOC(LLD_CIS_PWR_CHANGE_IND,
                                                                      KE_BUILD_ID(TASK_LLC, p_cis_env->link_id),
                                                                      TASK_NONE, lld_cis_pwr_change_ind);
                    msg->tx_rate = p_cis_param->tx_rate; // CIS transmit rate
                    msg->en = true; // enabled
                    ke_msg_send(msg);
                }
            }
        }
        #endif // BLE_PWR_CTRL

        // Mark procedure negotiation done
        p_cig_env->bf_cis_busy   &= ~CO_BIT(p_cis_param->cis_id);

        if (status != CO_ERROR_NO_ERROR)
        {
            // Clean the connection handle
            p_cis_env->link_id = BLE_INVALID_LINK_ID;

            p_cig_env->bf_cis_en   &= ~CO_BIT(p_cis_param->cis_id);

            #if (BLE_PERIPHERAL)
            // on slave side, since the datapath can be already configured, channel has to be removed by host
            // but channel handle cannot be reused so we remove information of related connection.
            // this channel becomes goes into a garbage state.
            if(p_cis_env->role == ROLE_SLAVE)
            {
                // remove channel from CIG
                p_cig_env->bf_cis_present &= ~CO_BIT(p_cis_param->cis_id);
                p_cig_env->nb_cis--;

                // check if CIG could be removed
                if((p_cig_env->nb_cis == 0))
                {
                    // clean-up the CIG
                    lli_group_cleanup(p_cig_env->group.hdl);
                }
            }
            #endif //(BLE_PERIPHERAL)
        }
    }
    else
    {
        ASSERT_INFO(0, chan_hdl, status);
    }

    #if (BLE_CENTRAL)
    if(lli_ci_env.p_create_cmd != NULL)
    {
        // check if another channel must be created
        lli_ci_env.cursor++;
        lli_cis_create_cont();
    }
    #endif // (BLE_CENTRAL)
}

void lli_cis_stop(uint8_t chan_hdl, uint8_t reason)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env   *p_cis_env;
    struct lli_cig_env *p_cig_env;

    // Check if CIS exists
    if (   (LLI_CIS_ENV_GET(chan_hdl, &p_cis_env)           == CO_ERROR_NO_ERROR)
        && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);

        // Check status, channel should at least be enabled
        if ((p_cig_env->bf_cis_en & CO_BIT(p_cis_param->cis_id)) != 0)
        {
            // Store reason
            p_cis_env->reason = reason;

            // Mark that a procedure is on-going on CIG
            p_cig_env->bf_cis_busy |= CO_BIT(p_cis_param->cis_id);

            // Stop the CIS driver
            lld_cis_stop(p_cis_env->chan.hdl);
        }
    }
    else
    {
        ASSERT_INFO(0, chan_hdl, reason);
    }
}

uint8_t lli_cis_start(uint8_t chan_hdl, uint32_t access_addr, uint16_t evt_cnt, uint32_t con_offset, uint16_t act_offset, uint16_t act_offset_hus, uint32_t air_time, bool encrypted)
{
    // Pointer to the indicated CIS environment
    struct lli_cis_env *p_cis_env;
    struct lli_cig_env *p_cig_env;
    uint8_t status = CO_ERROR_NO_ERROR;

    // Check if CIS exists
    if (   (LLI_CIS_ENV_GET(chan_hdl, &p_cis_env)           == CO_ERROR_NO_ERROR)
        && (LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR))
    {
        struct lli_cig_param* p_cig_param = &(p_cig_env->cig_param);
        struct lli_cis_param* p_cis_param = &(p_cis_env->cis_param);
        struct lld_cis_params drv_param;
        uint32_t cis_start_hs;
        uint16_t cis_start_hus;


        // Slave must compute the scheduling method as soon as more than one CIS must be established
        #if (BLE_PERIPHERAL)
        if((p_cis_env->role == ROLE_SLAVE) )
        {
            if(p_cig_env->bf_cis_en != 0)
            {
                struct lli_cis_env *p_oth_cis_env = (struct lli_cis_env *) co_list_pick(&(p_cig_env->channels));
                // packing is interleaved if both CIS has same interval and both CIS has group offset within this interval
                if(   (p_cis_env->cis_param.sub_interval == p_oth_cis_env->cis_param.sub_interval)
                   && (p_cis_env->cis_param.grp_offset     < p_cis_env->cis_param.sub_interval)
                   && (p_oth_cis_env->cis_param.grp_offset < p_cis_env->cis_param.sub_interval))
                {
                    p_cig_env->packing = ISO_PACKING_INTERLEAVED;
                }
                else
                {
                    p_cig_env->packing = ISO_PACKING_SEQUENTIAL;
                }
            }
            else
            {
                p_cig_env->packing = ISO_PACKING_SEQUENTIAL;
            }

            // add new channel in the list
            co_list_push_back(&(p_cig_env->channels), &(p_cis_env->chan.hdr));
        }
        #endif // (BLE_PERIPHERAL)


        // Fill Driver parameters
        drv_param.iso_interval     = p_cig_param->iso_interval;
        drv_param.sub_interval     = p_cis_param->sub_interval;
        drv_param.chan_hdl         = p_cis_env->chan.hdl;
        drv_param.grp_hdl          = p_cis_env->grp_hdl;
        drv_param.role             = ((p_cis_env->role == ROLE_MASTER) ? MASTER_ROLE : SLAVE_ROLE);
        drv_param.tx_ft            = p_cig_param->tx_ft;
        drv_param.rx_ft            = p_cig_param->rx_ft;
        drv_param.tx_rate          = p_cis_param->tx_rate;
        drv_param.rx_rate          = p_cis_param->rx_rate;
        drv_param.nse              = p_cis_param->nse;
        drv_param.access_addr      = access_addr;
        drv_param.con_link_id      = p_cis_env->link_id;
        drv_param.con_evt_cnt      = evt_cnt;
        drv_param.con_offset       = con_offset;
        drv_param.framing          = p_cig_param->framing;
        drv_param.tx_max_pdu       = p_cis_param->tx_max_pdu;
        drv_param.rx_max_pdu       = p_cis_param->rx_max_pdu;
        drv_param.tx_bn            = p_cis_param->tx_bn;
        drv_param.rx_bn            = p_cis_param->rx_bn;
        drv_param.tx_sdu_interval  = p_cig_param->tx_sdu_interval;
        drv_param.rx_sdu_interval  = p_cig_param->rx_sdu_interval;
        drv_param.tx_max_sdu       = p_cis_param->tx_max_sdu;
        drv_param.rx_max_sdu       = p_cis_param->rx_max_sdu;

        // Perform transport latency computation - can be done once.
        if(p_cig_env->bf_cis_en == 0)
        {
            // compute iso interval in microseconds
            uint32_t iso_interval_us = ((uint32_t) p_cig_param->iso_interval) * 1250;

            // Unframed mode - compute transport latency
            if(p_cig_param->framing == ISO_UNFRAMED_MODE)
            {
                // Transport_Latency = CIG Sync_Delay + (FT - 1) * ISO_Interval + ((ISO_Interval ?SDU interval)-1) * SDU interval
                // Transport_Latency = CIG Sync_Delay + FT * ISO_Interval - SDU interval
                p_cig_param->tx_trans_latency = p_cig_param->grp_air_time + (p_cig_param->tx_ft * iso_interval_us)
                                              - p_cig_param->tx_sdu_interval;
                p_cig_param->rx_trans_latency = p_cig_param->grp_air_time + (p_cig_param->rx_ft * iso_interval_us)
                                              - p_cig_param->rx_sdu_interval;
            }
            // Framed mode
            else
            {
                // Transport_Latency = CIG Sync_Delay + FT * ISO_Interval + SDU_Interval
                p_cig_param->tx_trans_latency = p_cig_param->grp_air_time + (p_cig_param->tx_ft * iso_interval_us)
                                              + p_cig_param->tx_sdu_interval;
                p_cig_param->rx_trans_latency = p_cig_param->grp_air_time + (p_cig_param->rx_ft * iso_interval_us)
                                              + p_cig_param->rx_sdu_interval;;
            }
        }


        drv_param.tx_trans_latency = p_cig_param->tx_trans_latency;
        drv_param.rx_trans_latency = p_cig_param->rx_trans_latency;

        drv_param.cis_spacing  = p_cis_param->grp_offset;
        drv_param.cis_sync_delay = p_cig_param->grp_air_time - p_cis_param->grp_offset;

        // Compute packet duration over the air
        drv_param.air_exch_dur = lli_ci_se_dur_compute(p_cis_param->tx_max_pdu, p_cis_param->tx_rate,
                                                       p_cis_param->rx_max_pdu, p_cis_param->rx_rate, encrypted);

        drv_param.packing      = p_cig_env->packing;

        status = lld_cis_start(p_cis_env->chan.hdl, &drv_param, &cis_start_hs, &cis_start_hus);

        if(status == CO_ERROR_NO_ERROR)
        {
            struct sch_plan_elt_tag *plan_elt = llm_plan_elt_get(p_cis_env->chan.hdl);

            // Load RX/TX Data Path
            if(drv_param.tx_max_sdu > 0)
            {
                lld_isoal_datapath_set(p_cis_env->chan.hdl, ISO_SEL_TX, p_cis_env->p_dp[ISO_SEL_TX],
                                       p_cis_env->trigger_offset[ISO_SEL_TX] != LLI_INVALID_TRIGGER_OFFSET,
                                       p_cis_env->trigger_offset[ISO_SEL_TX]);
            }
            if(drv_param.rx_max_sdu > 0)
            {
                lld_isoal_datapath_set(p_cis_env->chan.hdl, ISO_SEL_RX, p_cis_env->p_dp[ISO_SEL_RX],
                                       p_cis_env->trigger_offset[ISO_SEL_RX] != LLI_INVALID_TRIGGER_OFFSET,
                                       p_cis_env->trigger_offset[ISO_SEL_RX]);
            }

            #if (BLE_CENTRAL)
            // First CIS enabled, store the activity offset information
            if((p_cis_env->role == ROLE_MASTER) && (p_cig_env->bf_cis_en == 0))
            {
                p_cig_env->act_offset = act_offset;
            }
            #endif // (BLE_CENTRAL)

            // Register CIS in the planner
            plan_elt->offset       = act_offset;
            plan_elt->interval     = p_cig_param->iso_interval << 2;
            plan_elt->duration_min = (act_offset_hus + 2*air_time + HALF_SLOT_SIZE - 1) / HALF_SLOT_SIZE;
            plan_elt->duration_max = plan_elt->duration_min;
            plan_elt->margin       = 1 * (p_cis_env->role == ROLE_SLAVE);
            plan_elt->conhdl       = BLE_CHANHDL_TO_CISHDL(p_cis_env->chan.hdl);
            plan_elt->conhdl_ref   = BLE_LINKID_TO_CONHDL(p_cis_env->link_id);
            plan_elt->cb_move      = NULL;
            plan_elt->mobility     = SCH_PLAN_MB_LVL_0;

            sch_plan_set(plan_elt);

            // Update CIG Enabled state
            p_cig_env->bf_cis_en |=  CO_BIT(p_cis_param->cis_id);

            // inform that new CIS is granted
            if((lli_ci_env.cb_start_evt != NULL))
            {
                lli_ci_env.cb_start_evt(p_cis_env->chan.hdl, drv_param.con_link_id, p_cis_param,
                                        drv_param.iso_interval, drv_param.access_addr, cis_start_hs, cis_start_hus);
            }
        }
    }
    else
    {
        status = CO_ERROR_UNSPECIFIED_ERROR;
    }

    return status;
}

uint8_t lli_cis_data_path_set(struct lli_chan_env *p_env, uint8_t direction, uint8_t data_path_type)
{
    uint8_t status                = CO_ERROR_NO_ERROR;
    struct lli_cis_env* p_cis_env = (struct lli_cis_env*) p_env;
    struct lli_cig_env *p_cig_env = NULL;
    ASSERT_ERR(direction < ISO_SEL_MAX);
    do
    {
        const struct data_path_itf** pp_dp = &(p_cis_env->p_dp[direction]);
        int32_t trigger_offset             = p_cis_env->trigger_offset[direction];

        // load group environment
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);
        // An error occurs, do not continue data-path loading
        if(status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Check BN
        {
            uint8_t bn = (direction == ISO_SEL_TX) ? p_cis_env->cis_param.tx_bn : p_cis_env->cis_param.rx_bn;
            if(bn == 0)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }

        // Check the SDU size in ISO test mode
        #if (BLE_ISOGEN)
        if (data_path_type == ISO_DP_ISOGEN)
        {
            uint16_t max_sdu_size = (direction == ISO_SEL_TX) ? p_cis_env->cis_param.tx_max_sdu : p_cis_env->cis_param.rx_max_sdu;
            if(max_sdu_size < ISO_TEST_PKT_CNT_SIZE)
            {
                status = CO_ERROR_COMMAND_DISALLOWED;
                break;
            }
        }
        #endif //(BLE_ISOGEN)

        // Check if the data path has already been set up
        if(!data_path_is_disabled(*pp_dp))
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
            break;
        }

        // Load RX data path
        status = lli_data_path_load(p_env->hdl, direction, data_path_type, pp_dp);

        // Check that Isochronous stream is enabled
        if ((status == CO_ERROR_NO_ERROR) && (p_cig_env->bf_cis_en & CO_BIT(p_cis_env->cis_param.cis_id)) != 0)
        {
            // Update the data path to use
            status = lld_isoal_datapath_set(p_cis_env->chan.hdl, direction, *pp_dp, (trigger_offset != LLI_INVALID_TRIGGER_OFFSET), trigger_offset);
        }
    } while (0);

    return (status);
}

uint8_t lli_cis_data_path_remove(struct lli_chan_env *p_env, uint8_t direction)
{
    uint8_t status                = CO_ERROR_NO_ERROR;
    struct lli_cis_env* p_cis_env = (struct lli_cis_env*) p_env;
    struct lli_cig_env *p_cig_env = NULL;
    bool cis_active               = false;
    const struct data_path_itf** pp_dp;

    ASSERT_ERR(direction < ISO_SEL_MAX);

    pp_dp = &(p_cis_env->p_dp[direction]);

    if(LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env) == CO_ERROR_NO_ERROR)
    {
        // Check that Isochronous stream is enabled
        if ((p_cig_env->bf_cis_en & CO_BIT(p_cis_env->cis_param.cis_id)) != 0)
        {
            cis_active = true;
        }
    }

    // Check if datapath is enabled
    if(!data_path_is_disabled(*pp_dp))
    {
        if(cis_active)
        {
            // Update the data path to use
            lld_isoal_datapath_remove(p_cis_env->chan.hdl, direction);
        }

        lli_data_path_remove(p_env->hdl, direction, pp_dp);
    }
    else // The data path has not been set up
    {
        status = CO_ERROR_COMMAND_DISALLOWED;
    }

    return (status);
}

uint8_t lli_cis_trigger_set(struct lli_chan_env *p_env, uint8_t direction, bool enable, int32_t offset)
{
    uint8_t status                = CO_ERROR_NO_ERROR;

    do
    {
        struct lli_cis_env* p_cis_env = (struct lli_cis_env*) p_env;
        struct lli_cig_env *p_cig_env = NULL;

        // Load stream information
        status = LLI_CIG_ENV_GET(p_cis_env->grp_hdl, &p_cig_env);

        // Ensure that stream environment is available
        if(status != CO_ERROR_NO_ERROR)
        {
            break;
        }

        // Only RX direction supported
        if(direction != ISO_SEL_RX)
        {
            status = CO_ERROR_INVALID_HCI_PARAM;
            break;
        }

        // Check if stream is enabled
        if ((p_cig_env->bf_cis_en & CO_BIT(p_cis_env->cis_param.cis_id)) != 0)
        {
            // Set the trigger offset
            status = lld_isoal_trigger_set(p_cis_env->chan.hdl, direction, enable, offset);
        }
        else
        {
            status = CO_ERROR_COMMAND_DISALLOWED;
        }
    } while (0);

    return (status);
}

void lli_cis_start_evt_set(lli_cis_start_evt_cb evt_cb)
{
    lli_ci_env.cb_start_evt = evt_cb;
}

uint8_t lli_cis_stats_get(struct lli_chan_env *p_env, uint32_t* tx_unacked_packets, uint32_t* tx_flushed_packets, uint32_t* tx_last_subevent_packets,
                                                  uint32_t* retransmitted_packets, uint32_t* crc_error_packets, uint32_t* rx_unreceived_packets,
                                                  uint32_t* duplicate_packets)
{
    uint8_t status = CO_ERROR_UNKNOWN_CONNECTION_ID;
    struct lli_cis_env* p_cis_env = (struct lli_cis_env*) p_env;

    if (p_cis_env != NULL)
    {
        status = lld_cis_stats_get(p_cis_env->chan.hdl, tx_unacked_packets, tx_flushed_packets, tx_last_subevent_packets,
                                                    retransmitted_packets, crc_error_packets, rx_unreceived_packets,
                                                    duplicate_packets);

        // tx_last_subevent_packets is only valid when in the slave role
        if (p_cis_env->role == ROLE_MASTER)
        {
            *tx_last_subevent_packets = 0;
        }

    }

    return (status);
}

#endif //(BLE_CIS)

/// @} LLI
