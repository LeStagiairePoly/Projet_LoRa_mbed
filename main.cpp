#include <stdio.h>
/* #include <DHT.h> */

#include "lorawan/LoRaWANInterface.h"
#include "lorawan/system/lorawan_data_structures.h"
#include "events/EventQueue.h"

// Application helpers
#include "DummySensor.h"
#include "trace_helper.h"
#include "lora_radio_helper.h"
#include "CayenneLPP.h"

using namespace events;

uint8_t tx_buffer[30];
uint8_t rx_buffer[30];

#define TX_TIMER                        10000
#define MAX_NUMBER_OF_EVENTS            10
#define CONFIRMED_MSG_RETRY_COUNTER     3
#define PC_9                            0   // dummy sensor pin

DS1820 ds1820(PC_9);
CayenneLPP cayenne(100);

// 🔹 Lecture analogique sur PA_0
AnalogIn analog_in(PA_0);
DigitalOut led(LED1);

static EventQueue ev_queue(MAX_NUMBER_OF_EVENTS * EVENTS_EVENT_SIZE);
static void lora_event_handler(lorawan_event_t event);
static LoRaWANInterface lorawan(radio);
static lorawan_app_callbacks_t callbacks;

int main(void)
{
    setup_trace();

    lorawan_status_t retcode;

    if (lorawan.initialize(&ev_queue) != LORAWAN_STATUS_OK) {
        printf("\r\n LoRa initialization failed! \r\n");
        return -1;
    }

    printf("\r\n Mbed LoRaWANStack initialized \r\n");

    callbacks.events = mbed::callback(lora_event_handler);
    lorawan.add_app_callbacks(&callbacks);

    if (lorawan.set_confirmed_msg_retries(CONFIRMED_MSG_RETRY_COUNTER) != LORAWAN_STATUS_OK) {
        printf("\r\n set_confirmed_msg_retries failed! \r\n\r\n");
        return -1;
    }

    printf("\r\n CONFIRMED message retries : %d \r\n", CONFIRMED_MSG_RETRY_COUNTER);

    if (lorawan.enable_adaptive_datarate() != LORAWAN_STATUS_OK) {
        printf("\r\n enable_adaptive_datarate failed! \r\n");
        return -1;
    }

    printf("\r\n Adaptive data rate (ADR) - Enabled \r\n");

    retcode = lorawan.connect();

    if (retcode != LORAWAN_STATUS_OK && retcode != LORAWAN_STATUS_CONNECT_IN_PROGRESS) {
        printf("\r\n Connection error, code = %d \r\n", retcode);
        return -1;
    }

    printf("\r\n Connection - In Progress ...\r\n");

    ev_queue.dispatch_forever();

    return 0;
}

/**
 * Envoi d’un message LoRa
 */
static void send_message()
{
    uint16_t packet_len;
    int16_t retcode;

    // 🔹 Lecture de la tension
    double meas = analog_in.read();      // [0.0 ; 1.0]
    double tension = meas * 3.3;         // en volts

    int entier = (int)tension;
    int decimales = (int)((tension - entier) * 100);
    printf("Tension mesuree = %d.%02d V\n", entier, decimales);


    // Allumage LED si > 0.5 V
    led = (tension > 0.5) ? 1 : 0;

    // Création du payload Cayenne
    cayenne.reset();
    cayenne.addAnalogInput(8, tension);  // 📌 Transmission comme Analog Input sur canal 8
    cayenne.copy(tx_buffer);
    packet_len = cayenne.getSize();

    retcode = lorawan.send(MBED_CONF_LORA_APP_PORT, tx_buffer, packet_len, MSG_CONFIRMED_FLAG);

    // Affichage debug
    printf("\nMessage envoye (hex): ");
    for (int i = 0; i < packet_len; i++) {
        printf("%02X ", tx_buffer[i]);
    }
    printf("\n");

    if (retcode < 0) {
        retcode == LORAWAN_STATUS_WOULD_BLOCK ? printf("send - WOULD BLOCK\r\n")
        : printf("\r\n send() - Error code %d \r\n", retcode);

        if (retcode == LORAWAN_STATUS_WOULD_BLOCK) {
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                ev_queue.call_in(3000, send_message);
            }
        }
        return;
    }

    printf("\r\n %d bytes scheduled for transmission \r\n", retcode);
    memset(tx_buffer, 0, sizeof(tx_buffer));
}

/**
 * Réception LoRa
 */
static void receive_message()
{
    uint8_t port;
    int flags;
    int16_t retcode = lorawan.receive(rx_buffer, sizeof(rx_buffer), port, flags);

    if (retcode < 0) {
        printf("\r\n receive() - Error code %d \r\n", retcode);
        return;
    }

    printf(" RX Data on port %u (%d bytes): ", port, retcode);
    for (uint8_t i = 0; i < retcode; i++) {
        printf("%02x ", rx_buffer[i]);
    }
    printf("\r\n");

    memset(rx_buffer, 0, sizeof(rx_buffer));
}

/**
 * Handler des événements LoRaWAN
 */
static void lora_event_handler(lorawan_event_t event)
{
    switch (event) {
        case CONNECTED:
            printf("\r\n Connection - Successful \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            } else {
                ev_queue.call_every(TX_TIMER, send_message);
            }
            break;

        case DISCONNECTED:
            ev_queue.break_dispatch();
            printf("\r\n Disconnected Successfully \r\n");
            break;

        case TX_DONE:
            printf("\r\n Message Sent to Network Server \r\n");
            if (MBED_CONF_LORA_DUTY_CYCLE_ON) {
                send_message();
            }
            break;

        case RX_DONE:
            printf("\r\n Received message from Network Server \r\n");
            receive_message();
            break;

        case JOIN_FAILURE:
            printf("\r\n OTAA Failed - Check Keys \r\n");
            break;

        default:
            printf("\r\n Event = %d \r\n", event);
    }
}
