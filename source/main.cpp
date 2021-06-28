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

#include "mbed.h"
#include "mbed_trace.h"
#include "LoRaWANInterface.h"
#include "lora_radio_helper.h"
#include "dev_eui_helper.h"
#include "storage_helper.h"
#include "UpdateCerts.h"
#include "LoRaWANUpdateClient.h"

#include "utils_bd.h"

#if defined(TARGET_DISCO_L475VG_IOT01A)
#include "stm32l475e_iot01_tsensor.h"
#include "stm32l475e_iot01_hsensor.h"
#endif

EventQueue evqueue;

// DEV EUI is the authentication of the device itself in the network and to the server
// The default given here should not be used for the wanted targets.
// The value used for targets are either : 
//      - Extracted from the lora chip (xDot)
//      - Extrapolated from mbed_id (disco)
//      - Not changed from default

// Note: if the device has built-in dev eui (see dev_eui_helper.h), the dev eui will be overwritten in main()<
static uint8_t DEV_EUI[] = { 0xbd, 0x25, 0xa6, 0x76, 0x37, 0xb0, 0xe5, 0x96};

// APP EUI is Joiner EUI, this is not used
// the value is kept to zero
static uint8_t APP_EUI[] = { 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}; 

// APP KEY is used for authication to the app 
// and to generate the multicast keys used for communication
// To be able to address all devices at the same time,
// all devices have a common APP_KEY (this key is the default used by the web_app)
static uint8_t APP_KEY[] = { 0x35, 0x0b, 0x7f, 0x62, 0xcf, 0x56, 0xab, 0x3e, 0xfe, 0x9e, 0x2c, 0xc7, 0x24, 0xe8, 0x3f, 0xaa};


static void lora_event_handler(lorawan_event_t event);
static void lora_uc_send(LoRaWANUpdateClientSendParams_t &params);
static void queue_next_send_message();
static void send_message();

static LoRaWANInterface lorawan(radio);
static lorawan_app_callbacks_t callbacks;
static LoRaWANUpdateClient uc(&bd, APP_KEY, lora_uc_send);
static loramac_protocol_params class_a_params;  // @todo: this is 816 bytes, can we use a smaller structure?
static LoRaWANUpdateClientClassCSession_t class_c_details;
static bool in_class_c_mode = false;
static bool clock_is_synced = false;
static LoRaWANUpdateClientSendParams_t queued_message;
static bool queued_message_waiting = false;

static DigitalOut led1(ACTIVITY_LED);

static void turn_led_on() {
    led1 = 1;
}
static void turn_led_off() {
    led1 = 0;
}

// This is already debounced to the eventqueue, so safe to run printf here
static void switch_to_class_a() {
    printf("Switch to Class A\n");
    turn_led_off();
    evqueue.call_in(200, &turn_led_off); //confirm led turn off
    uc.printHeapStats("CLASSA ");

    in_class_c_mode = false;

    // put back the class A session
    lorawan.set_session(&class_a_params);
    lorawan.enable_adaptive_datarate();
    lorawan.set_device_class(CLASS_A);

    // wait for a few seconds to send the message
    evqueue.call_in(5000, &send_message);
}

static void switch_class_c_rx2_params() {
    loramac_protocol_params class_c_params;

    // copy them to the class C params...
    memcpy(&class_c_params, &class_a_params, sizeof(loramac_protocol_params));
    class_c_params.dl_frame_counter = 0;
    class_c_params.ul_frame_counter = 0;
    class_c_params.dev_addr = class_c_details.deviceAddr;
    memcpy(class_c_params.keys.nwk_skey, class_c_details.nwkSKey, 16);
    memcpy(class_c_params.keys.app_skey, class_c_details.appSKey, 16);

    class_c_params.sys_params.rx2_channel.frequency = class_c_details.downlinkFreq;
    class_c_params.sys_params.rx2_channel.datarate = class_c_details.datarate;

    printf("Class C session set with parameters : \n");
    printf("         - frequency : %ld\n",class_c_params.sys_params.rx2_channel.frequency);
    printf("         - datarate  : %d\n",class_c_params.sys_params.rx2_channel.datarate);
    printf("         - dl_frame_counter     : %ld\n",class_c_params.dl_frame_counter);
    printf("         - ul_frame_counter     : %lu\n",class_c_params.ul_frame_counter);
    printf("         - keys.nwk_skey    : "); for (size_t i = 0; i < 16; i++){printf(" 0x%02x",class_c_params.keys.nwk_skey[i]);} printf("\n"); 
    printf("         - keys.app_skey    : "); for (size_t i = 0; i < 16; i++){printf(" 0x%02x",class_c_params.keys.app_skey[i]);} printf("\n"); 

    if(class_c_params.sys_params.rx2_channel.frequency==0){
        printf("     -> As frequency == 0; skipping class C Set Session, directly switching");
        printf("     -> Add to queue to return to class A in 2 mins");
        evqueue.call_in(120000, &switch_to_class_a);
    }else{
    // and set the class C session
        tr_debug("LW SET SESSION : %d ",lorawan.set_session(&class_c_params));
    }

    //start the C session
    tr_debug("LW SET DV CLASS: %d ",lorawan.set_device_class(CLASS_C));
    
}

static void switch_to_class_c() {
    printf("Switch to Class C\n");
    turn_led_on();

    lorawan.cancel_sending();
    lorawan.disable_adaptive_datarate();

    if (queued_message_waiting) {
        queued_message_waiting = false;
        free(queued_message.data);
    }

    in_class_c_mode = true;

    // store the class A parameters
    lorawan.get_session(&class_a_params);

    // in 1.5 second, actually switch to Class C (allow clearing the queue in the LoRaWAN stack)
    evqueue.call_in(1500, &switch_class_c_rx2_params);
}

// This runs in an interrupt routine, so just copy the parameter and dispatch to event queue
static void switch_to_class_c_irq(LoRaWANUpdateClientClassCSession_t* session) {
    core_util_critical_section_enter();
    memcpy(&class_c_details, session, sizeof(LoRaWANUpdateClientClassCSession_t));
    core_util_critical_section_exit();

    evqueue.call(&switch_to_class_c);
}

static void lorawan_uc_fragsession_complete() {
    printf("Frag session is complete\n");
}

#if MBED_CONF_LORAWAN_UPDATE_CLIENT_INTEROP_TESTING
uint32_t interop_crc32 = 0x0;
static void lorawan_uc_firmware_ready(uint32_t crc) {
    uc.printHeapStats("FWREADY ");
    printf("Firmware is ready, CRC32 hash is %08lx\n", crc);
    interop_crc32 = crc;
}
#else
static void lorawan_uc_firmware_ready() {
    uc.printHeapStats("FWREADY ");
    printf("Firmware is ready, hit **RESET** to flash the firmware\n");

    // reboot system
    NVIC_SystemReset();
}
#endif

static void lora_uc_send(LoRaWANUpdateClientSendParams_t &params) {
    queued_message = params;
    // copy the buffer
    queued_message.data = (uint8_t*)malloc(params.length);
    if (!queued_message.data) {
        printf("ERR! Failed to allocate %u bytes for queued_message!\n", params.length);
        return;
    }
    memcpy(queued_message.data, params.data, params.length);
    queued_message_waiting = true;

    // will be sent in the next iteration
}

// Send a message over LoRaWAN - todo, check for duty cycle
static void send_message() {
    if (in_class_c_mode){
        tr_debug("Device is in class C, not sending now");
        return;
    }     
//    tr_debug("Device Not in C continuing to send data");

#if MBED_CONF_LORAWAN_UPDATE_CLIENT_INTEROP_TESTING
    // after calculating the crc32, that's the only thing we'll send
    if (interop_crc32 != 0x0) {
        uint8_t buffer[6] = {
            DATA_BLOCK_AUTH_REQ, 0 /* fragIndex, always 0 */,
            interop_crc32 & 0xff, interop_crc32 >> 8 & 0xff, interop_crc32 >> 16 & 0xff, interop_crc32 >> 24 & 0xff
        };
        int16_t retcode = lorawan.send(201, buffer, sizeof(buffer), MSG_UNCONFIRMED_FLAG);
        if (retcode < 0) {
            printf("send_message for DATA_BLOCK_AUTH_REQ on port %d failed (%d)\n", 201, retcode);
            queue_next_send_message();
        }
        else {
            printf("%d bytes scheduled for transmission on port %d\n", sizeof(buffer), 201);
        }
        return;
    }
#endif

    // @todo: implement retries allowed
    if (queued_message_waiting) {
        // detect if this is class c session start message
        // because if so, we should change the timeToStart to the current moment as we don't send immediately
        if (queued_message.port == MCCONTROL_PORT && queued_message.length == MC_CLASSC_SESSION_ANS_LENGTH
                && queued_message.data[0] == MC_CLASSC_SESSION_ANS) {
            uc.updateClassCSessionAns(&queued_message);
        }

        int16_t retcode = lorawan.send(
            queued_message.port,
            queued_message.data,
            queued_message.length,
            queued_message.confirmed ? MSG_CONFIRMED_FLAG : MSG_UNCONFIRMED_FLAG);

        if (retcode < 0) {
            printf("send_message for queued_message on port %d failed (%d)\n", queued_message.port, retcode);
            queue_next_send_message();
        }
        else {
            free(queued_message.data);
            queued_message_waiting = false;
            printf("%d bytes scheduled for transmission on port %d\n", queued_message.length, queued_message.port);
        }

        return;
    }

    if (!clock_is_synced) {
        // this will trigger a lora_uc_send command
        uc.requestClockSync(true);
        // in the next command we'll actually send it
        queue_next_send_message();
        return;
    }

    uint16_t sizeToSend = 0;
    uint8_t portToSend=0;
    int16_t retcode= 0;

    #if defined(TARGET_DISCO_L475VG_IOT01A)

    //board supports sensors
    #if defined(P_READING) && P_READING==1
    float hum = BSP_HSENSOR_ReadHumidity();
    uint16_t hum_unit = (uint8_t) hum;
    uint16_t hum_dot  = (uint8_t) ((hum-hum_unit)*100); 
    tr_info("Humidity computed is %d,%d ",hum_unit,hum_dot);
    //tr_info("Humidity read : %f",hum);
    #endif

    float temp = BSP_TSENSOR_ReadTemp();
    uint16_t TEMP_unit = (uint8_t) temp;
    uint16_t TEMP_dot  = (uint8_t) ((temp-TEMP_unit)*100); 
    tr_info("Temp computed is %d,%d °C ",TEMP_unit,TEMP_dot);
    //tr_info("Temp read °C : %f",temp);

    
    char messageTemp[30] = {0};
    #if defined(P_READING) && P_READING==1
    sprintf(messageTemp,"TEMP:%2d,%2d||HUM:%2d,%2d",TEMP_unit,TEMP_dot,hum_unit,hum_dot);
    #else
    sprintf(messageTemp,"TEMP:%2d,%2d",TEMP_unit,TEMP_dot);
    #endif

    retcode = lorawan.send(110, (uint8_t*)(&messageTemp), sizeof(messageTemp), MSG_UNCONFIRMED_FLAG);
    sizeToSend = sizeof(float);
    portToSend = 110;
    #else
    // board is not recognised as using sensors
    // just send a random message (this is where you'd put your sensor data)
    int r = rand();
    retcode = lorawan.send(15, (uint8_t*)(&r), sizeof(r), MSG_UNCONFIRMED_FLAG);
    sizeToSend = sizeof(r);
    portToSend = 15;
    #endif

    if (retcode < 0) {
        printf("send_message for normal message on port %d failed (%d)\n", portToSend, retcode);
        queue_next_send_message();
    }
    else {
        printf("%d bytes scheduled for transmission on port %d\n", sizeToSend, portToSend);
    }
}

static void queue_next_send_message() {

    tr_debug("...... trying to queue a new message .......");

    if (in_class_c_mode) return;

    int backoff;
    lorawan.get_backoff_metadata(backoff);

    if (backoff < 0) {
        backoff = 5000;
    }

    evqueue.call_in(backoff, &send_message);
}

static void startup_tests() {
}


int main() {
    printf("\n  Mbed OS 5 Firmware Update over LoRaWAN  \n");
    printf("\n----Version Modified as Nagra Project-----\n");

    printf("\n-- This version reports Temperature In Celsius --\n");
    #if defined(P_READING) && P_READING==1
    printf("\n-- It also reports air Humidity                --\n");
    #endif
    uc.printHeapStats("INFO : ");
    printf("\n-- Version V.2 !!! (JPATCH & DDELTA (w rtos)) --\n");
    printf("\n--       This Will allow tests of patches     --\n");

    // Enable trace output for this demo, so we can see what the LoRaWAN stack does
    mbed_trace_init();
    mbed_trace_exclude_filters_set("QSPIF");

    //Init Sensors if supported at startup
    #if defined(TARGET_DISCO_L475VG_IOT01A)
    #if defined(P_READING) && P_READING==1
    if (BSP_HSENSOR_Init() != HSENSOR_OK){
        printf("Humidity Sensor initialization failed!\n");
        return -1;
    }else{
        printf("Humidity Sensor initialization SUCCESS\n");
    }
    #endif

    if (BSP_TSENSOR_Init() != TSENSOR_OK){
        printf("Temp Sensor initialization failed!\n");
        return -1;
    }else{
        printf("Temp Sensor initialization SUCCESS\n");
    }
    #endif
    if (lorawan.initialize(&evqueue) != LORAWAN_STATUS_OK) {
        printf("LoRa initialization failed!\n");
        return -1;
    }

    // update client callbacks, note that these run in an ISR!
    uc.callbacks.switchToClassA = evqueue.event(switch_to_class_a); // dispatch to eventqueue
    uc.callbacks.switchToClassC = switch_to_class_c_irq;

    // These run in the context that calls the update client
    uc.callbacks.fragSessionComplete = evqueue.event(lorawan_uc_fragsession_complete);
    uc.callbacks.firmwareReady = evqueue.event(lorawan_uc_firmware_ready);

    // prepare application callbacks
    callbacks.events = callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    // Enable adaptive data rating
    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("enable_adaptive_datarate failed!\n");
        return -1;
    }

    lorawan.set_device_class(CLASS_A);

    if (get_built_in_dev_eui(DEV_EUI, sizeof(DEV_EUI)) == 0) {
        printf("read built-in dev eui: %02x %02x %02x %02x %02x %02x %02x %02x\n",
            DEV_EUI[0], DEV_EUI[1], DEV_EUI[2], DEV_EUI[3], DEV_EUI[4], DEV_EUI[5], DEV_EUI[6], DEV_EUI[7]);
    }

    lorawan_connect_t connect_params;
    connect_params.connect_type = LORAWAN_CONNECTION_OTAA;
    connect_params.connection_u.otaa.dev_eui = DEV_EUI;
    connect_params.connection_u.otaa.app_eui = APP_EUI;
    connect_params.connection_u.otaa.app_key = APP_KEY;
    connect_params.connection_u.otaa.nb_trials = 3;

    lorawan_status_t retcode = lorawan.connect(connect_params);

    if (retcode == LORAWAN_STATUS_OK ||
        retcode == LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
    } else {
        printf("Connection error, code = %d\n", retcode);
        return -1;
    }

    printf("Connection - In Progress ...\r\n");

    //Init Sensors if supported at startup
    #if defined(TARGET_DISCO_L475VG_IOT01A)

    printf("Press the BLUE button to execute statups tests\n");

    // hit the blue button to record a message
    static InterruptIn btn(BUTTON1);
    btn.fall(evqueue.event(&startup_tests));

    #endif

    // make your event queue dispatching events forever
    evqueue.dispatch_forever();
}

// This is called from RX_DONE, so whenever a message came in
// so it can be analysed by the application layer
static void receive_message()
{
    uint8_t rx_buffer[255] = { 0 };
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("receive() - Error code %d\n", retcode);
        return;
    }

    printf("Received %d bytes on port %u\n", retcode, port);

    LW_UC_STATUS status = LW_UC_OK;


    if (port == 200) {
        status = uc.handleMulticastControlCommand(rx_buffer, retcode);
    }
    else if (port == 201) {
        // retrieve current session and set dev addr
        loramac_protocol_params params;
        lorawan.get_session(&params);
        status = uc.handleFragmentationCommand(params.dev_addr, rx_buffer, retcode);

        // blink LED when receiving a packet in Class C mode
        if (in_class_c_mode) {
            turn_led_on();
            evqueue.call_in(100, &turn_led_off);
            evqueue.call_in(150, &turn_led_on);
        }
    }
    else if (port == 202) {
        status = uc.handleClockSyncCommand(rx_buffer, retcode);
        if (status == LW_UC_OK) {
            printf("Clock is Synced\n");
            clock_is_synced = true;
        }
    }
    else if (port == STATUS_VERSION_PORT){
        status = uc.handleStatusVersionCommand(rx_buffer, retcode);
        printf(" > msg on port %d; outcome %u\n",port,status);
    }else{
        printf("Message on port not handled");
    }

    if (status != LW_UC_OK) {
        printf("Failed to handle UC command on port %d, status %d\n", port, status);
    }

    printf("\n\n\n");
}

// Event handler from deeper layers
static void lora_event_handler(lorawan_event_t event) {
    switch (event) {
        case CONNECTED:
            printf("Connection - Successful\n");

            uc.printHeapStats("CONNECTED ");

            queue_next_send_message();
            break;
        case DISCONNECTED:
            printf("Disconnected Successfully\n");
            break;
        case TX_DONE:
        {
            printf("Message Sent to Network Server\n");
            queue_next_send_message();
            break;
        }
        case TX_TIMEOUT:
        case TX_ERROR:
        case TX_CRYPTO_ERROR:
        case TX_SCHEDULING_ERROR:
            printf("Transmission Error - EventCode = %d\n", event);
            queue_next_send_message();
            break;
        case RX_DONE:
            printf("Received message from Network Server\n");
            receive_message();
            break;
        case RX_TIMEOUT:
        case RX_ERROR:
            printf("Error in reception - Code = %d\n", event);
            break;
        case JOIN_FAILURE:
            printf("OTAA Failed - Check Keys\n");
            break;
        default:
            MBED_ASSERT("Unknown Event");
            break;
    }
}
