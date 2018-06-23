/**
 * Author: Xavier Vilajosana (xvilajosana@eecs.berkeley.edu)
 *         Pere Tuset (peretuset@openmote.com)
 * Date:   July 2013
 * Description: CC2538-specific definition of the "radio" bsp module.
 */

#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <stdbool.h>

#include "sdk/components/boards/pca10056.h"
#include "sdk/components/boards/boards.h"
#include "sdk/components/drivers_nrf/radio_config/radio_config.h"
#include "sdk/modules/nrfx/mdk/nrf52840.h"
#include "sdk/components/libraries/timer/app_timer.h"

#include "app_config.h"
#include "leds.h"
#include "radio.h"
#include "board.h"
#include "board_info.h"
#include "debugpins.h"
#include "sctimer.h"

//=========================== defines =========================================

/* Bit Masks for the last byte in the RX FIFO */
//#define CRC_BIT_MASK 0x80
//#define LQI_BIT_MASK 0x7F

/* RSSI Offset */
//#define RSSI_OFFSET 73
//#define CHECKSUM_LEN 2

/* For calculating frequency */
#define FREQUENCY_OFFSET  10
#define FREQUENCY_STEP    5

//=========================== variables =======================================

typedef struct {
  radio_capture_cbt         startFrame_cb;
  radio_capture_cbt         endFrame_cb;
  radio_state_t             state; 
} radio_vars_t;

typedef struct {
  uint8_t                   length;
  uint8_t*                  packet;
} nrf_tx_payload;

typedef struct {
  uint8_t* pBufRead;
  uint8_t* pLenRead;
  uint8_t  maxBufLen;
  int8_t* pRssi;
  uint8_t* pLqi;
  bool* pCrc;
} nrf_rx_payload;

radio_vars_t radio_vars;
nrf_tx_payload tx_payload;
nrf_rx_payload rx_payload;
PORT_TIMER_WIDTH capturedTime;

//=========================== prototypes ======================================


//=========================== public ==========================================


void radio_init(void) {

  radio_configure();
  NRF_RADIO->MODE      = (RADIO_MODE_MODE_Ieee802154_250Kbit << RADIO_MODE_MODE_Pos);

}

void radio_setStartFrameCb(radio_capture_cbt cb) {
   radio_vars.startFrame_cb  = cb;
}

void radio_setEndFrameCb(radio_capture_cbt cb) {
   radio_vars.endFrame_cb    = cb;
}

//===== reset

void radio_reset(void) {

}

//===== RF admin

void radio_setFrequency(uint8_t frequency) {

   radio_vars.state = RADIOSTATE_SETTING_FREQUENCY;

   NRF_RADIO->FREQUENCY = (frequency - FREQUENCY_OFFSET) * FREQUENCY_STEP;

   radio_vars.state = RADIOSTATE_FREQUENCY_SET;
}

void radio_rfOn(void) {
   //radio_on();
}

void radio_rfOff(void) {
   // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;
}

//===== TX

void radio_loadPacket(uint8_t* packet, uint16_t len) {
   
  radio_vars.state = RADIOSTATE_LOADING_PACKET;

  if (len < 128)
  {
    tx_payload.length = (uint8_t)len;
    tx_payload.packet = packet;

    NRF_RADIO->PACKETPTR = (uint32_t)&tx_payload;

    radio_vars.state = RADIOSTATE_PACKET_LOADED;
  }
}

void radio_txEnable(void) {

   radio_vars.state = RADIOSTATE_ENABLING_TX;

   NRF_RADIO->EVENTS_READY = 0U;
   NRF_RADIO->TASKS_TXEN = 1;

   radio_vars.state = RADIOSTATE_TX_ENABLED;
}


void radio_txNow(void) {

   radio_vars.state = RADIOSTATE_TRANSMITTING;
   
   while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END  = 0U;
    NRF_RADIO->TASKS_START = 1U;

    while (NRF_RADIO->STATE != RADIO_STATE_STATE_Tx)
    {
      // wait untill radio enters TX state so we can call callback
    }
    capturedTime = sctimer_readCounter();
    radio_vars.startFrame_cb(capturedTime);

    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }

    capturedTime = sctimer_readCounter();
    radio_vars.endFrame_cb(capturedTime);


    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }
    //leds_radio_toggle();

    radio_vars.state = RADIOSTATE_TXRX_DONE;
}

//===== RX

void radio_rxEnable(void) {
   radio_vars.state = RADIOSTATE_ENABLING_RX;
 
   NRF_RADIO->EVENTS_READY = 0U;
   // Enable radio and wait for ready
   NRF_RADIO->TASKS_RXEN = 1;

   radio_vars.state = RADIOSTATE_LISTENING;
}

void radio_rxNow(void) {

  
  
}

void radio_getReceivedFrame(uint8_t* pBufRead,
                            uint8_t* pLenRead,
                            uint8_t  maxBufLen,
                             int8_t* pRssi,
                            uint8_t* pLqi,
                               bool* pCrc) {
   
  radio_vars.state = RADIOSTATE_RECEIVING;

  leds_radio_toggle();
   while (NRF_RADIO->EVENTS_READY == 0U)
    {
        // wait
    }
    NRF_RADIO->EVENTS_END = 0U;
    // Start listening and wait for address received event
    NRF_RADIO->TASKS_START = 1U;

    // Wait for end of packet or buttons state changed
    while (NRF_RADIO->EVENTS_END == 0U)
    {
        // wait
    }


    if (NRF_RADIO->CRCSTATUS == 1U)
    {
        //result = packet;
    }
    NRF_RADIO->EVENTS_DISABLED = 0U;
    // Disable radio
    NRF_RADIO->TASKS_DISABLE = 1U;

    while (NRF_RADIO->EVENTS_DISABLED == 0U)
    {
        // wait
    }


   radio_vars.state = RADIOSTATE_TXRX_DONE;
   
}

//=========================== private =========================================

//=========================== callbacks =======================================

//=========================== interrupt handlers ==============================

kick_scheduler_t radio_isr(void) {
  return DO_NOT_KICK_SCHEDULER;
}
