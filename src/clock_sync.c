#include "clock_sync.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <mpsl_timeslot.h>
#include <hal/nrf_radio.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_timer.h>
#include <hal/nrf_ppi.h>
#include "device_role.h"

// --- CONFIGURATION ---
#define TIMESLOT_LENGTH_US 55000   
#define CUSTOM_SYNC_FREQ 80

// --- GLOBALS ---
static mpsl_timeslot_session_id_t m_session_id;
static uint8_t sync_packet[4] = {0xAA, 0xBB, 0xCC, 0xDD};

volatile uint32_t captured_hardware_time_us = 0;
volatile bool is_hardware_synced = false;
volatile uint32_t global_sync_baseline_ms = 0;
volatile bool baseline_update_pending = false;

// --- SHARED RADIO CONFIG ---
static void configure_radio_hardware(void) {
    nrf_radio_power_set(NRF_RADIO, true);
    nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_BLE_1MBIT);
    nrf_radio_frequency_set(NRF_RADIO, CUSTOM_SYNC_FREQ);
    
    nrf_radio_txaddress_set(NRF_RADIO, 0);
    nrf_radio_rxaddresses_set(NRF_RADIO, 1);
    
    nrf_radio_prefix0_set(NRF_RADIO, 0xC0);
    nrf_radio_base0_set(NRF_RADIO, 0x01234567);

    // Wipe out BLE settings and force a raw 4-byte packet ---
    NRF_RADIO->PCNF0 = 0; // Disable S0, Length, and S1 fields
    
    // MAXLEN = 4, STATLEN = 4, Base Address Length = 4
    NRF_RADIO->PCNF1 = (4UL << RADIO_PCNF1_MAXLEN_Pos)  | 
                       (4UL << RADIO_PCNF1_STATLEN_Pos) | 
                       (4UL << RADIO_PCNF1_BALEN_Pos);
}

// --- THE PRIMARY CALLBACK ---
static mpsl_timeslot_signal_return_param_t* primary_timeslot_callback(
    mpsl_timeslot_session_id_t session_id, uint32_t signal_type) 
{
    static mpsl_timeslot_signal_return_param_t return_param;
    return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;

    if (signal_type == MPSL_TIMESLOT_SIGNAL_START) {
        
        // Safe Clock Start
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
        uint32_t fail_safe = 10000;
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED) && --fail_safe);
        if (fail_safe == 0) goto cleanup; 

        configure_radio_hardware();
        nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_0DBM);

        global_sync_baseline_ms = k_uptime_get_32();

        // BURST MODE: Fire the packet 50 times safely
        for (int i = 0; i < 50; i++) {
            nrf_radio_packetptr_set(NRF_RADIO, sync_packet);
            
            // Wait for TX to be ready
            nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);
            nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_TXEN);
            fail_safe = 10000;
            while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_READY) && --fail_safe);
            if (fail_safe == 0) break; 
            
            // Send packet
            nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
            nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);
            
            fail_safe = 10000;
            while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_END) && --fail_safe);
            if (fail_safe == 0) break; 
            
            // SAFELY turn off radio before next loop
            nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
            nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
            
            fail_safe = 10000;
            while(!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED) && --fail_safe);
            if (fail_safe == 0) break; 
            
            k_busy_wait(20);
        }
cleanup:
        // Force the radio to turn off and WAIT for it
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
        nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
        uint32_t off_safe = 10000;
        while(!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED) && --off_safe);

        // Wipe every flag back to 0
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

        return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
    }
    return &return_param;
}

// --- THE SECONDARY CALLBACK ---
static mpsl_timeslot_signal_return_param_t* secondary_timeslot_callback(
    mpsl_timeslot_session_id_t session_id, uint32_t signal_type) 
{
    static mpsl_timeslot_signal_return_param_t return_param;
    return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;

    if (signal_type == MPSL_TIMESLOT_SIGNAL_START) {
        
        // Safe Clock Start
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
        uint32_t fail_safe = 10000;
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED) && --fail_safe);
        if (fail_safe == 0) goto cleanup;

        // Setup Timer
        nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_TIMER);
        nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_32);
        nrf_timer_prescaler_set(NRF_TIMER1, NRF_TIMER_FREQ_1MHz);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

        // Setup PPI
        nrf_ppi_channel_endpoint_setup(NRF_PPI, NRF_PPI_CHANNEL19, 
            (uint32_t)nrf_radio_event_address_get(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS),
            (uint32_t)nrf_timer_task_address_get(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE0));
        nrf_ppi_channel_enable(NRF_PPI, NRF_PPI_CHANNEL19);

        // Setup Radio
        configure_radio_hardware();
        nrf_radio_packetptr_set(NRF_RADIO, sync_packet); 

        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);
        nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_RXEN);
        
        fail_safe = 10000;
        while (!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_READY) && --fail_safe);
        if (fail_safe == 0) goto cleanup; 
        
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
        nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);

        // EXTENDED TIMEOUT LOOP (50ms Window)
       bool packet_received = false;

        while (1) {
            nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE1);
            uint32_t current_time_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL1);

            if (current_time_us >= 50000) { 
                // 50ms absolute timeout reached. Bail out safely before the 55ms MPSL limit
                break; 
            }

            if (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_END)) {
                baseline_update_pending = true;
                packet_received = true;
                break;
            }
        }

        // 6. Process Result
        if (packet_received) {
            captured_hardware_time_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0);
            is_hardware_synced = true;
        }

cleanup:
        // Force the radio to turn off and WAIT for it
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);
        nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_DISABLE);
        uint32_t off_safe = 10000;
        while(!nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_DISABLED) && --off_safe);

        // Wipe every radio flag back to 0
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_ADDRESS);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
        nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_DISABLED);

        // Clean up the Timer and the safe PPI channel
        nrf_ppi_channel_disable(NRF_PPI, NRF_PPI_CHANNEL19); // <-- Using safe channel!
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_STOP);
        
        return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_END;
    }
    return &return_param;
}

// --- PUBLIC API ---

void clock_sync_init(void) {
    int32_t err; 
    
    if (device_role_is_primary()) {
        err = mpsl_timeslot_session_open(primary_timeslot_callback, &m_session_id);
        printk("Timeslot Session Opened: PRIMARY\n");
    } else {
        err = mpsl_timeslot_session_open(secondary_timeslot_callback, &m_session_id);
        printk("Timeslot Session Opened: SECONDARY\n");
    }
    
    if (err) {
        printk("FATAL: Failed to open timeslot session: %d\n", err);
    }
}

void request_sync_timeslot(void) {
    static mpsl_timeslot_request_t request;
    
    request.request_type = MPSL_TIMESLOT_REQ_TYPE_EARLIEST;
    request.params.earliest.hfclk = MPSL_TIMESLOT_HFCLK_CFG_NO_GUARANTEE;
    request.params.earliest.priority = MPSL_TIMESLOT_PRIORITY_HIGH;
    request.params.earliest.length_us = TIMESLOT_LENGTH_US;
    request.params.earliest.timeout_us = 1000000; 

    int32_t err = mpsl_timeslot_request(m_session_id, &request);
    
    if (err) {
        printk("Warning: Could not request timeslot: %d\n", err);
    } else {
        printk("Timeslot requested successfully.\n");
    }
}