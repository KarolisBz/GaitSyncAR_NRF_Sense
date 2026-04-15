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
#define TIMESLOT_LENGTH_US 65000   
#define CUSTOM_SYNC_FREQ 80

// At 1 Mbps, 1 bit takes exactly 1 microsecond.
// Preamble (8) + Address (40) + Payload (80) + CRC (16) = 144us Airtime
// 144us Airtime + 6us Disable + 20us Wait + 130us TXEN + ~2us processing = ~302us
#define BURST_LOOP_DELAY_US 303.5 // + 1.5 us of tuning

// Time from capturing the timestamp to the moment the first packet's ADDRESS physically hits the air.
// TX Ramp-up (130us) + Preamble (8us) + Address (40us) = 178us
#define FIRST_PACKET_OFFSET_US 178

// --- GLOBALS ---
static mpsl_timeslot_session_id_t m_session_id;
// Byte 0 = Index, Bytes 1-8 = 64-bit Timestamp, Byte 9 = Pad
static uint8_t sync_packet[10] = {0x00};

volatile uint64_t captured_hardware_time_us = 0;
volatile bool is_hardware_synced = false;
volatile uint64_t global_sync_baseline_us = 0;
volatile bool baseline_update_pending = false;
volatile bool sync_ack_sent = false;
volatile uint64_t local_sync_anchor_us = 0;

// --- SHARED RADIO CONFIG ---
static void configure_radio_hardware(void) {
    nrf_radio_power_set(NRF_RADIO, true);
    nrf_radio_mode_set(NRF_RADIO, NRF_RADIO_MODE_BLE_1MBIT);
    nrf_radio_frequency_set(NRF_RADIO, CUSTOM_SYNC_FREQ);
    
    nrf_radio_txaddress_set(NRF_RADIO, 0);
    nrf_radio_rxaddresses_set(NRF_RADIO, 1);
    
    nrf_radio_prefix0_set(NRF_RADIO, 0xC0);
    nrf_radio_base0_set(NRF_RADIO, 0x01234567);

    // Wipe out BLE settings and force a raw 8-byte packet ---
    NRF_RADIO->PCNF0 = 0; // Disable S0, Length, and S1 fields
    
    // MAXLEN = 10, STATLEN = 10, Base Address Length = 4
    NRF_RADIO->PCNF1 = (10UL << RADIO_PCNF1_MAXLEN_Pos)  | 
                       (10UL << RADIO_PCNF1_STATLEN_Pos) | 
                       (4UL << RADIO_PCNF1_BALEN_Pos);

    nrf_radio_crc_configure(NRF_RADIO, RADIO_CRCCNF_LEN_Two, NRF_RADIO_CRC_ADDR_INCLUDE, 0x11021);
    // Forcing CRC starting seed to be identical on both devices
    nrf_radio_crcinit_set(NRF_RADIO, 0xFFFF);
}

// --- THE PRIMARY CALLBACK ---
static mpsl_timeslot_signal_return_param_t* primary_timeslot_callback(
    mpsl_timeslot_session_id_t session_id, uint32_t signal_type) 
{
    static mpsl_timeslot_signal_return_param_t return_param;
    return_param.callback_action = MPSL_TIMESLOT_SIGNAL_ACTION_NONE;

    if (signal_type == MPSL_TIMESLOT_SIGNAL_START) {
        
        // Safe Clock Start
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
        uint32_t fail_safe = 10000;
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED) && --fail_safe);
        if (fail_safe == 0) goto cleanup; 

        // STARTTING TIMER1 ON PRIMARY SO IMU INTERRUPT CAN USE IT
        nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_TIMER);
        nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_32);
        nrf_timer_prescaler_set(NRF_TIMER1, NRF_TIMER_FREQ_1MHz);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_START);

        configure_radio_hardware();
        nrf_radio_txpower_set(NRF_RADIO, NRF_RADIO_TXPOWER_0DBM);

        // Capture time in microseconds, storing in packet for the secondary to read. This is moment that defines our sync point.
        // Driven by the 64MHz/16MHz system clock (True microsecond resolution)
        uint64_t primary_start_us = k_ticks_to_us_floor64(k_uptime_ticks());
        memcpy(&sync_packet[1], &primary_start_us, sizeof(primary_start_us));

        // BURST MODE: Fire the packet 200 times safely
        for (int i = 0; i < 200; i++) {
            
            // Injecting the current loop index into the very first byte of the payload
            // we can use this to calculate the actual synced moment on the secondary and compensate for any consistent delays in our processing or the radio's state changes
            sync_packet[0] = (uint8_t)i;

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
        // Setting baseline
        global_sync_baseline_us = primary_start_us;
        local_sync_anchor_us = primary_start_us;
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
        printk("global_sync_baseline_us = %lu\n", (unsigned long)global_sync_baseline_us);

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
        nrf_clock_event_clear(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED);
        nrf_clock_task_trigger(NRF_CLOCK, NRF_CLOCK_TASK_HFCLKSTART);
        uint32_t fail_safe = 10000;
        while (!nrf_clock_event_check(NRF_CLOCK, NRF_CLOCK_EVENT_HFCLKSTARTED) && --fail_safe);
        if (fail_safe == 0) goto cleanup;

        // Setup Timer
        nrf_timer_mode_set(NRF_TIMER1, NRF_TIMER_MODE_TIMER);
        nrf_timer_bit_width_set(NRF_TIMER1, NRF_TIMER_BIT_WIDTH_32);
        nrf_timer_prescaler_set(NRF_TIMER1, NRF_TIMER_FREQ_1MHz);
        nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CLEAR);
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

        // TIMEOUT LOOP (50ms Window)
       bool packet_received = false;

        while (1) {
            nrf_timer_task_trigger(NRF_TIMER1, NRF_TIMER_TASK_CAPTURE1);
            uint64_t current_time_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL1);

            if (current_time_us >= 50000) { 
                // 50ms absolute timeout reached. Bail out safely before the 65ms MPSL limit
                break; 
            }

            if (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_END)) {
                if (nrf_radio_event_check(NRF_RADIO, NRF_RADIO_EVENT_CRCOK)) {
                    packet_received = true;
                    break;
                } else {
                    // Noise, false trigger, or partial packet. Clear and go back to listening safely.
                    nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_END);
                    nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCOK);
                    nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_CRCERROR);
                    nrf_radio_event_clear(NRF_RADIO, NRF_RADIO_EVENT_READY);
                    nrf_radio_task_trigger(NRF_RADIO, NRF_RADIO_TASK_START);
                }
            }
        }

        // Process Result
        if (packet_received) {
            captured_hardware_time_us = nrf_timer_cc_get(NRF_TIMER1, NRF_TIMER_CC_CHANNEL0);

            uint8_t packet_index = sync_packet[0];
            uint64_t primary_timestamp_us;
            memcpy(&primary_timestamp_us, &sync_packet[1], sizeof(primary_timestamp_us));

            // Calculating EXACT global transmission moment utilizing new constraints
            uint64_t primary_packet_tx_time = primary_timestamp_us + FIRST_PACKET_OFFSET_US + (packet_index * BURST_LOOP_DELAY_US);

            // Establishing perfect alignment mapping for Timer1 Zero point
            global_sync_baseline_us = primary_packet_tx_time - captured_hardware_time_us;
            
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

        // Clean PPI channel
        nrf_ppi_channel_disable(NRF_PPI, NRF_PPI_CHANNEL19);
        
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
    is_hardware_synced = false; 
    sync_ack_sent = false; 
    baseline_update_pending = false;
    captured_hardware_time_us = 0;
    global_sync_baseline_us = 0;
    
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