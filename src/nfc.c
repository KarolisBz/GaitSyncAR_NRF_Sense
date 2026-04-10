#include "nfc_handler.h"
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/addr.h>
#include <nfc_t2t_lib.h>
#include <nfc/ndef/msg.h>
#include <nfc/ndef/text_rec.h>
#include <stdio.h>
#include <string.h>

#define NDEF_MSG_BUF_SIZE 256

static uint8_t ndef_msg_buf[NDEF_MSG_BUF_SIZE];
static char payload_string[100];

/* Callback for NFC events */
static void nfc_callback(void *context, nfc_t2t_event_t event, const uint8_t *data, size_t data_length)
{
    if (event == NFC_T2T_EVENT_FIELD_ON) {
        printk("-> NFC Tap Detected!\n");
    } else if (event == NFC_T2T_EVENT_DATA_READ) {
        printk("-> SUCCESS: Phone read the MAC address!\n");
    }
}

int nfc_handler_init(void)
{
    int err;
    uint32_t len = sizeof(ndef_msg_buf);
    bt_addr_le_t addrs[1];
    size_t count = 1;
    static const uint8_t lang_code[] = {'e', 'n'};

    /* 1. Get BLE Address for the payload */
    err = bt_enable(NULL);
    if (err) {
        printk("Bluetooth init failed (err %d)\n", err);
        return err;
    }
    
    bt_id_get(addrs, &count);

    snprintf(payload_string, sizeof(payload_string), 
             "Hello World! My BLE MAC is: %02X:%02X:%02X:%02X:%02X:%02X",
             addrs[0].a.val[5], addrs[0].a.val[4], addrs[0].a.val[3],
             addrs[0].a.val[2], addrs[0].a.val[1], addrs[0].a.val[0]);
             
    printk("Generated Payload: %s\n", payload_string);

    /* 2. Setup NFC Tag 2 Type library */
    err = nfc_t2t_setup(nfc_callback, NULL);
    if (err < 0) {
        return err;
    }

    /* 3. Create NDEF Text Record */
    NFC_NDEF_TEXT_RECORD_DESC_DEF(text_rec, 
                                  UTF_8, 
                                  lang_code, sizeof(lang_code), 
                                  payload_string, strlen(payload_string));

    NFC_NDEF_MSG_DEF(text_msg, 1);

    err = nfc_ndef_msg_record_add(&NFC_NDEF_MSG(text_msg), &NFC_NDEF_TEXT_RECORD_DESC(text_rec));
    if (err < 0) {
        return err;
    }

    /* 4. Encode and Set Payload */
    err = nfc_ndef_msg_encode(&NFC_NDEF_MSG(text_msg), ndef_msg_buf, &len);
    if (err < 0) {
        return err;
    }

    err = nfc_t2t_payload_set(ndef_msg_buf, len);
    if (err < 0) {
        return err;
    }

    /* 5. Start Emulation */
    return nfc_t2t_emulation_start();
}