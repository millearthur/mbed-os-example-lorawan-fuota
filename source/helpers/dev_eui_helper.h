/*
 * PackageLicenseDeclared: Apache-2.0
 * Copyright (c) 2018 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef _DEV_EUI_HELPER_H
#define _DEV_EUI_HELPER_H

#include "mbed.h"

/**
 * This file reads DevEUI from ROM/RAM/Flash if the target has one.
 *
 * @param buffer    8 byte buffer to store the DevEUI in
 *                  if the target does not have a built-in DevEUI this buffer is not touched
 * @param size      Size of the buffer
 * @returns         0 if successful, -1 if no DevEUI could be read, -2 if the buffer size was not correct
 */

#if defined(TARGET_FF1705_L151CC) || defined(TARGET_XDOT_L151CC)
#include "xdot_eeprom.h"

int8_t get_built_in_dev_eui(uint8_t *buffer, size_t size) {
    if (size != 8) return -2;

    int v = xdot_eeprom_read_buf(0x401, buffer, size);
    if (v != 0) {
        return -1;
    }

    return 0;
}

#elif defined(TARGET_DISCO_L475VG_IOT01A) 

#include "tiny-aes.h"   // @todo: replace by Mbed TLS / hw crypto?

#include "mbed_trace.h"
#ifndef TRACE_GROUP
#define TRACE_GROUP "EUIH"
#endif

#if defined(ACTIVATE_BLE)
#if ACTIVATE_BLE == 1

#include "ble/BLE.h"

Gap::Address_t address_ble;
static EventQueue event_queue(/* event count */ 10 * EVENTS_EVENT_SIZE);

class BLE_EX : ble::Gap::EventHandler {
public:
    BLE_EX(BLE &ble, events::EventQueue &event_queue) :
        _ble(ble),
        _event_queue(event_queue),
        _adv_data_builder(_adv_buffer) { }

    ~BLE_EX() {
    }

    void start() {
        printf("start, init events\n");
        _ble.gap().setEventHandler(this);
        _ble.init(this, &BLE_EX::on_init_complete); 
        _event_queue.dispatch_forever();
    }

private:
    /** Callback triggered when the ble initialization process has finished */
    void on_init_complete(BLE::InitializationCompleteCallbackContext *params) {
        if (params->error != BLE_ERROR_NONE) {
            printf("Ble initialization failed.");
            return;
        }

        Gap::AddressType_t addr_type;
        BLE::Instance().gap().getAddress(&addr_type, address_ble);
        printf("init complete\n");

        _event_queue.break_dispatch();
    }
private:
    /* Event handler */
    void onDisconnectionComplete(const ble::DisconnectionCompleteEvent&) {
    }

private:
    BLE &_ble;
    events::EventQueue &_event_queue;
    uint8_t _adv_buffer[ble::LEGACY_ADVERTISING_MAX_SIZE];
    ble::AdvertisingDataBuilder _adv_data_builder;
};

/** Schedule processing of events from the BLE middleware in the event queue. */
void schedule_ble_events(BLE::OnEventsToProcessCallbackContext *context) {
    event_queue.call(Callback<void()>(&context->ble, &BLE::processEvents));
    printf("Event processed ...\n");
}
#endif
#endif

int8_t get_built_in_dev_eui(uint8_t *buffer, size_t size) {
    if (size != 8) return -2;

    char* name;
    name = (char*) malloc(10);
    

    #if defined(ACTIVATE_BLE)
    #if ACTIVATE_BLE == 1

    BLE &ble = BLE::Instance();
    ble.onEventsToProcess(schedule_ble_events);
    BLE_EX ex(ble, event_queue);
    ex.start();

    printf("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
            address_ble[5], address_ble[4], address_ble[3], address_ble[2], address_ble[1], address_ble[0]);

    memcpy(name,address_ble,6);
    ble.shutdown();
    #endif


    #else
    mbed_mac_address(name);
    #endif

    tr_debug("NAME BUFFER VALLUE IS : %02x:%02x:%02x:%02x:%02x:%02x" , name[0], name[1], name[2], name[3], name[4], name[5]);
    
    uint8_t key[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
    key[0] = name[0];   //(unique_id & 0x000000ff);
    key[1] = name[1];   //(unique_id & 0x0000ff00) >> 8;
    key[2] = name[2];   //(unique_id & 0x00ff0000) >> 16;
    key[3] = name[3];   //(unique_id & 0xff000000) >> 24;
    key[4] = name[4];
    key[5] = name[5];

    uint8_t in[16] = { 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0 };
    uint8_t out[16] = {};

    AES_ECB_encrypt(in,key,out,16);
    memcpy(buffer,out,size);
    return 0;
}

#else
int8_t get_built_in_dev_eui(uint8_t *, size_t) {
    return -1;
}
#endif

#ifndef TRACE_GROUP
#undef TRACE_GROUP
#endif

#endif // _DEV_EUI_HELPER_H
