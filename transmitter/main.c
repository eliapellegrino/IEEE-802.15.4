/**
 * Copyright (c) 2014 - 2020, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/** @file
*
* @defgroup nrf_dev_button_radio_tx_example_main main.c
* @{
* @ingroup nrf_dev_button_radio_tx_example
*
* @brief Radio Transceiver Example Application main file.
*
* This file contains the source code for a sample application using the NRF_RADIO peripheral.
*
*/

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include "boards.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#include "nrf_802154.h"
#include "802154_config_mine.h"

#include "sdk_config.h"

#include "nrf_drv_timer.h"
#include "nrf_drv_gpiote.h"
#include "nrf_drv_uart.h"
/*-------------------------- VERSION WITHOUT CCA-------------------------*/
#define ENABLE_NRF_LOG_INFO            0
#define ENABLE_TIMESTAMP               0
#define ENABLE_PA_LNA_PIN              0
#define TX_BUFF_SIZE_UART              20
#define TX_POWER                       0 /* Tx Power in dBm*/

#define WINDOW_LENGTH                  7
#define TIMER_SIMULATION_LENGTH        1 /*Simulazione onda quadra*/

#define CONFIG_802154_COMPLETE_LED     BSP_BOARD_LED_0 /*I LED 1 e 2 non possono venire usati con 802.15.4*/
#define NOTIF_ON_LED                   BSP_BOARD_LED_1
#define TIMER_SIM_LED                  BSP_BOARD_LED_2

#define PIN_IN                         NRF_GPIO_PIN_MAP(1,1)
#define PA_PIN                         NRF_GPIO_PIN_MAP(1,3)
#define LNA_PIN                        NRF_GPIO_PIN_MAP(1,4)
// PIN PER DELAY
#define PIN_OUT_DEBUG_PRE              NRF_GPIO_PIN_MAP(1,5)
#define PIN_OUT_DEBUG_POST             NRF_GPIO_PIN_MAP(1,6)


static volatile bool tx_in_progress = false;
static volatile bool tx_done = true;
static volatile bool begin_tx = false;

bool enable_pin_out = false;
bool enable_uart = false;

bool notif_on = false;
bool last_packet = false;
uint8_t int_event[MAX_PAYLOAD_SIZE];
uint16_t mask_fc = 0x0001; /*Mask for controlling individual bit of frame control field (used to set b5 ACK/NOT ACK)*/
uint16_t frame_control = 0b1000100001000001; /*Vedere i bit a cosa corrispondono*/
uint8_t current_index = 0; /*Index of position */
uint8_t count = 0;
uint16_t total_event = 0;
uint8_t packet[MAX_PACKET_SIZE];
uint8_t tx_uart_buff[TX_BUFF_SIZE_UART];
// Payload da 1 solo BYTE
uint8_t notif_packet[PHR_SIZE + MAC_HEADER_SIZE + FCS_SIZE + NOTIF_PACKET_PAYLOAD_SIZE];

/*Non usare l'istanza 0 e 1 che buggano (la 2 non è stata provata, ma da documentazione dovrebbero comunque solamente usare due istanze del timer)*/
// Vedere se si possono disattivare in qualche moodo le altre istanze usate dai driver internamente
nrf_drv_timer_t TIMER_SIMULATION = NRF_DRV_TIMER_INSTANCE(2);
nrf_drv_timer_t TIMER_TIMESTAMP = NRF_DRV_TIMER_INSTANCE(3);
nrf_drv_timer_t TIMER_ACQUISITION = NRF_DRV_TIMER_INSTANCE(4); /*NON USARE LA 1 CHE VA IN BUG (probabilmente usato dai radio driver)*/

const nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);

/*Function declaration*/
static void set_frame_control(uint8_t* packet_to_set, bool aux_sec_header, bool frame_pending, bool is_ack, bool pan_id_compress);
// Uart initialization
/*
  Handler for UART event
*/
void uart_event_handler(nrf_drv_uart_event_t * p_event, void * p_context)
{
  switch (p_event->type)
  {
  case NRF_DRV_UART_EVT_TX_DONE:
  {
    memset(tx_uart_buff, 0xff, sizeof(tx_uart_buff));
    #if ENABLE_NRF_LOG_INFO
    NRF_LOG_INFO("TX DONE");
    #endif
    if (last_packet)
    {
      last_packet = false;
      ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, tx_uart_buff, sizeof(tx_uart_buff));
         APP_ERROR_CHECK(err_code);
    }
  } break;
  }
}

static void uart_init()
{
  ret_code_t err_code;
  // UART CONFIGURATION
  static nrf_drv_uart_config_t uart_configuration = {
    .pseltxd = TX_PIN_NUMBER,
    .pselrxd = RX_PIN_NUMBER,
    .pselcts = CTS_PIN_NUMBER,
    .pselrts = RTS_PIN_NUMBER,
    .hwfc = NRF_UART_HWFC_DISABLED,
    .parity = NRF_UART_PARITY_EXCLUDED,
    .baudrate = NRF_UART_BAUDRATE_1000000,
    .interrupt_priority = _PRIO_APP_LOW,
  };
  err_code = nrf_drv_uart_init( & uart_inst, & uart_configuration, uart_event_handler);
  APP_ERROR_CHECK(err_code);
}
static void log_init()
{
  ret_code_t err_code;
  err_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(err_code);

  NRF_LOG_DEFAULT_BACKENDS_INIT();
}
static void led_init()
{
  bsp_board_init(BSP_INIT_LEDS);
}

static void button_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action)
{
  // ATTENZIONE! Se ENABLE_DEBUG_GPIO è settato BUGGANO i bottoni
  switch (pin)
  {
  case BUTTON_1:
  {
    TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
    #if ENABLE_NRF_LOG_INFO
    NRF_LOG_INFO("Timer started at : %d", TIMER_TIMESTAMP.p_reg->CC[0]);
    #endif
    // Send notification packet (guardare bene come funziona il l'ack timeout)
    set_frame_control(notif_packet, false, true, true, true);
    notif_packet[MAC_PAYLOAD_POS] = count;
    nrf_802154_transmit_raw(notif_packet, false);
    notif_on = true;
  } break;
  case BUTTON_2:
  {
    /*
    TIMER_ACQUISITION.p_reg->TASKS_STOP = 1;
    TIMER_ACQUISITION.p_reg->TASKS_CLEAR = 1;
    //TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[1] = 1;
    //NRF_LOG_INFO("Timer stopped at : %d", TIMER_TIMESTAMP.p_reg->CC[1]);
    // Disable PIN_IN sensing
    nrf_drv_gpiote_in_event_disable(PIN_IN);
    // NOTIF_ON_LED off
    bsp_board_led_off(NOTIF_ON_LED);
    */
    //begin_tx = false;
    TIMER_ACQUISITION.p_reg->TASKS_STOP = 1;
    TIMER_ACQUISITION.p_reg->TASKS_CLEAR = 1;
    notif_on = false;
    // Send a packet with last counter and pending bit not set (il ricevitore capisce così di non aspettarsi più nessun dato)
    set_frame_control(notif_packet, false, false, true, true);
    notif_packet[MAC_PAYLOAD_POS] = count;
    volatile bool tmp = false;
    do
    {
      tmp = nrf_802154_transmit_raw(notif_packet, false);
    } while(tmp == false);
  } break;
  case BUTTON_3:
  {
    TIMER_SIMULATION.p_reg->TASKS_START = 1;
    bsp_board_led_on(TIMER_SIM_LED);
  } break;
  case BUTTON_4:
  {
    TIMER_SIMULATION.p_reg->TASKS_STOP = 1;
    TIMER_SIMULATION.p_reg->TASKS_CLEAR = 1;
    bsp_board_led_off(TIMER_SIM_LED);
  }

  }
}

static void buttons_init()
{
  //bsp_board_init(BSP_INIT_BUTTONS);
  ret_code_t err_code = nrf_drv_gpiote_init();
  APP_ERROR_CHECK(err_code);
  //Init button
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_HITOLO(true);
  in_config.pull = NRF_GPIO_PIN_PULLUP;
  //BUTTON 1
  err_code = nrf_drv_gpiote_in_init(BUTTON_1, &in_config, button_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(BUTTON_1, true);
  //BUTTON 2
  err_code = nrf_drv_gpiote_in_init(BUTTON_2, & in_config, button_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(BUTTON_2, true);
  //BUTTON 3
  err_code = nrf_drv_gpiote_in_init(BUTTON_3, & in_config, button_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(BUTTON_3, true);
  //BUTTON 4
  err_code = nrf_drv_gpiote_in_init(BUTTON_4, & in_config, button_handler);
  APP_ERROR_CHECK(err_code);
  nrf_drv_gpiote_in_event_enable(BUTTON_4, true);
}
static void addr_set()
{
  uint8_t saddr[2];
  uint8_t laddr[8];
  //Set short address
  saddr[0] = SHORT_ADDR_SOURCE & 0x00ff;
  saddr[1] = (SHORT_ADDR_SOURCE & 0xff00) >> 8;
  nrf_802154_short_address_set(saddr);
  //Set long address
  laddr[0] = saddr[0];
  laddr[1] = saddr[1];
  laddr[2] = (EXT_ADDR & 0x0000000000ff0000) >> 16;
  laddr[3] = (EXT_ADDR & 0x00000000ff000000) >> 24;
  laddr[4] = (EXT_ADDR & 0x000000ff00000000) >> 32;
  laddr[5] = (EXT_ADDR & 0x0000ff0000000000) >> 40;
  laddr[6] = (EXT_ADDR & 0x00ff000000000000) >> 48;
  laddr[7] = (EXT_ADDR & 0xff00000000000000) >> 56;
  nrf_802154_extended_address_set(laddr);
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("Short and Extended address correctly set: %x %x", laddr[6], laddr[7]);
  #endif
}
static void set_pa_lna()
{
  //nrf_802154_fem_control_cfg_t fem_conf = NRF_802154_FEM_DEFAULT_SETTINGS;
  nrf_fem_interface_config_t fem_conf;
  // LNA PIN
  fem_conf.lna_pin_config.active_high = 1;
  fem_conf.lna_pin_config.enable = 1;
  fem_conf.lna_pin_config.gpio_pin = LNA_PIN;
  fem_conf.lna_pin_config.gpiote_ch_id = NRF_FEM_CONTROL_DEFAULT_LNA_GPIOTE_CHANNEL;
  // PA PIN
  fem_conf.pa_pin_config.active_high = 1;
  fem_conf.pa_pin_config.enable = 1;
  fem_conf.pa_pin_config.gpio_pin = PA_PIN;
  fem_conf.pa_pin_config.gpiote_ch_id = NRF_FEM_CONTROL_DEFAULT_PA_GPIOTE_CHANNEL;
  // OTHER FEM CONFIGURATION
  fem_conf.fem_config.lna_time_gap_us = NRF_FEM_LNA_TIME_IN_ADVANCE_US;
  fem_conf.fem_config.pa_time_gap_us = NRF_FEM_PA_TIME_IN_ADVANCE_US;
  fem_conf.ppi_ch_id_clr = NRF_FEM_CONTROL_DEFAULT_CLR_PPI_CHANNEL;
  fem_conf.ppi_ch_id_set = NRF_FEM_CONTROL_DEFAULT_SET_PPI_CHANNEL;
  ret_code_t err_code = nrf_fem_interface_configuration_set(&fem_conf);
  if (err_code == NRF_SUCCESS)
  {
    #if ENABLE_NRF_LOG_INFO
    NRF_LOG_INFO("PA E LNA PIN SET");
    #endif
  }

}

static void set_frame_control(uint8_t* packet_to_set, bool aux_sec_header, bool frame_pending, bool is_ack, bool pan_id_compress)
{
  // Setto b3 se richiesto
  if (aux_sec_header)
  {
    frame_control = frame_control | (mask_fc << AUX_SEC_HEADER_BIT_POS);
  }
  else
  {
    frame_control = frame_control & (~(mask_fc << AUX_SEC_HEADER_BIT_POS));
  }
  // Setto b4 se richiesto
  if (frame_pending)
  {
    frame_control = frame_control | (mask_fc << FRAME_PENDING_BIT_POS);
  }
  else
  {
    frame_control = frame_control & (~(mask_fc << FRAME_PENDING_BIT_POS));
  }
  // Setto il bit 5 a 1 se ACK, a 0 se non è ACK
  if (is_ack)
  {
    frame_control = frame_control | (mask_fc << ACK_BIT_POS);
  }
  else
  {
    frame_control = frame_control & (~(mask_fc << ACK_BIT_POS));
  }
  // Setto b6 se richiesto
  if (pan_id_compress)
  {
    frame_control = frame_control | (mask_fc << PAN_ID_COMPRESS_BIT_POS);
  }
  else
  {
    frame_control = frame_control & (~(mask_fc << PAN_ID_COMPRESS_BIT_POS));
  }
  // Cambiare in modo da fare tutto con Shift (?)
  packet_to_set[FC_POS] = frame_control & 0x00ff;
  packet_to_set[FC_POS + 1] = (frame_control & 0xff00) >> 8;
}
static void configure_packets()
{
  /*Data packet*/
  //Set PHR field
  packet[PHR_POS] = MAX_PACKET_SIZE;
  /*Set Frame Control*/
  set_frame_control(packet, false, true, false, true);
  // Set Sequence Number (ogni volta viene creato un numero random)
  packet[SN_POS] = 0x00;
  // Set Destination PAN_ID (and only that one for the FRAME CONTROL)
  packet[PAN_DEST_POS] = PAN_ID & 0x00ff;
  packet[PAN_DEST_POS + 1] = (PAN_ID & 0xff00) >> 8;
  // Set Destinatinon Address (Short)
  packet[SA_DEST_POS] = SHORT_ADDR_DEST & 0x00ff;
  packet[SA_DEST_POS + 1] = (SHORT_ADDR_DEST & 0xff00) >> 8;
  // Set Source Address (Short)
  packet[SA_SOURCE_POS] = SHORT_ADDR_SOURCE & 0x00ff;
  packet[SA_SOURCE_POS + 1] = (SHORT_ADDR_SOURCE & 0xff00) >> 8;
  // Set Notification On Packet (se il pending bit è settto, stiamo dicendo al ricevitore di aspettarsi altri dati)
  memcpy(notif_packet, packet, PHR_SIZE + MAC_HEADER_SIZE);
  notif_packet[PHR_POS] = MAC_HEADER_SIZE + FCS_SIZE + NOTIF_PACKET_PAYLOAD_SIZE + PHR_SIZE;
  set_frame_control(notif_packet, false, true, true, true);
}
static void config_802154()
{
  //Set addresses
  addr_set();
  // Set PAN ID
  uint8_t ppan_id[2];
  ppan_id[0] = PAN_ID & 0x00ff;
  ppan_id[1] = (PAN_ID & 0xff00) >> 8;
  nrf_802154_pan_id_set(ppan_id);
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("Pan ID correctly set");
  #endif
  //Set  radio output power (dBm) (@ref nrf52840_bitfields.h)
  nrf_802154_tx_power_set((int8_t)TX_POWER);
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("Power set to : %d dBm", power);
  #endif
  // Set PA e LNA PIN
  #if ENABLE_PA_LNA
  set_pa_lna();
  #endif
  // Configure Packet
  configure_packets();
}
void timer_timestamp_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  //
}
void timer_simulation_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  TIMER_ACQUISITION.p_reg->TASKS_CAPTURE[0] = 1;
  uint8_t now = TIMER_ACQUISITION.p_reg->CC[0];
  memset(&packet[current_index], now, 1);
  current_index++;
}
void timer_acquisition_event_handler(nrf_timer_event_t event_type, void* p_context)
{
  // Aspetto per l'ACK prima di inviare effettivamente il pacchetto (vale solo per il primo pacchetto)
  while (!begin_tx)
  {
    //WAIT
  }
  count++;
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("STATE: %x", nrf_802154_state_get());
  #endif
  // Set SN to counter
  packet[SN_POS] = count;
  // Primo byte per il numero di event in quella finestra
  packet[MAC_PAYLOAD_POS] = current_index;
  memcpy(&packet[MAC_PAYLOAD_POS+1], int_event, MAX_PAYLOAD_SIZE-1);
  memset(int_event, 0, MAX_PAYLOAD_SIZE);
  current_index = 0;
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("%x %x %x %x %x", packet[MAC_PAYLOAD_POS],packet[MAC_PAYLOAD_POS+1],packet[MAC_PAYLOAD_POS+2],packet[MAC_PAYLOAD_POS+3],packet[MAC_PAYLOAD_POS+4]);
  #endif
  do
  {
    tx_in_progress = nrf_802154_transmit_raw(packet, false);
    #if ENABLE_NRF_LOG_INFO
    NRF_LOG_INFO("Trasmission result: %x, Count = %d", tx_in_progress, count);
    #endif
    if (tx_in_progress == 1)
    {
      tx_done = false;
      //NRF_LOG_INFO("EVENT_TRASMIT: %x %x %x %x %x", packet[MAC_PAYLOAD_POS], packet[MAC_PAYLOAD_POS + 1], packet[MAC_PAYLOAD_POS + 2], packet[MAC_PAYLOAD_POS + 3], packet[MAC_PAYLOAD_POS + 4]);
      if (enable_pin_out)
      {
        nrf_gpio_pin_toggle(PIN_OUT_DEBUG_PRE);
      }
      if (enable_uart)
      {
        NRF_LOG_INFO("PACKET IN TRASMISSION");
        tx_uart_buff[0] = packet[SN_POS];
        tx_uart_buff[1] = 1; //channel
        tx_uart_buff[2] = packet[MAC_PAYLOAD_POS];
        memcpy(&tx_uart_buff[3], &packet[MAC_PAYLOAD_POS+1], 20-3);
        //memcpy(tx_uart_buff, packet, MAX_PACKET_SIZE);
        ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, tx_uart_buff, 20);
        APP_ERROR_CHECK(err_code);
      }
    }
  } while (tx_done);
}
static void timers_init()
{
  nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
  ret_code_t err_code;
  // TIMER TIMESTAMP
  #if ENABLE_TIMESTAMP
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
  timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
  err_code = nrf_drv_timer_init(&TIMER_TIMESTAMP, &timer_cfg, timer_timestamp_event_handler);
  APP_ERROR_CHECK(err_code);
  #endif
  // TIMER ACQUISITION
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
  timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
  err_code = nrf_drv_timer_init(&TIMER_ACQUISITION, &timer_cfg, timer_acquisition_event_handler);
  APP_ERROR_CHECK(err_code);
  uint32_t time_ticks = (WINDOW_LENGTH * 0.001) * 31250;
  nrf_drv_timer_extended_compare(&TIMER_ACQUISITION, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
  // Timer simulation
  timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
  timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
  err_code = nrf_drv_timer_init(&TIMER_SIMULATION, &timer_cfg, timer_simulation_event_handler);
  APP_ERROR_CHECK(err_code);
  time_ticks = (TIMER_SIMULATION_LENGTH * 0.001) * 31250;
  nrf_drv_timer_extended_compare(&TIMER_SIMULATION, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
}

void in_pin_handler(nrf_drv_gpiote_pin_t pin, nrf_gpiote_polarity_t action) {
  int8_t i;
  ret_code_t err_code;
  TIMER_ACQUISITION.p_reg -> TASKS_CAPTURE[1] = 1;
  uint8_t now = TIMER_ACQUISITION.p_reg -> CC[1];
  //Current_index riportato a zero ogni volta che scade il timer di invio del pacchetto
  memset(&int_event[current_index], now, 1);
  current_index++;
  total_event++;
}

static void pins_init()
{
  // PIN IN ATC SIGNAL
  nrf_drv_gpiote_in_config_t in_config = GPIOTE_CONFIG_IN_SENSE_LOTOHI(true);
  ret_code_t err_code = nrf_drv_gpiote_in_init(PIN_IN, &in_config, in_pin_handler);
  APP_ERROR_CHECK(err_code);
  //PIN OUT DEBUG
  nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
  nrf_drv_gpiote_out_init(PIN_OUT_DEBUG_PRE, &out_config);
  nrf_drv_gpiote_out_init(PIN_OUT_DEBUG_POST, &out_config);
}


int main(void)
{
  memset(&packet[PHR_SIZE + MAC_HEADER_SIZE], 0xaa, MAX_PACKET_SIZE - PHR_SIZE - MAC_HEADER_SIZE);
  log_init();
  // Peripheral initialization
  led_init();
  buttons_init();
  timers_init();
  pins_init();
  uart_init();
  nrf_drv_gpiote_in_event_enable(PIN_IN, true);
  // 802.15.4 Configuration
  nrf_802154_init();
  config_802154();
  // End of configuration
  bsp_board_led_on(CONFIG_802154_COMPLETE_LED);
  TIMER_TIMESTAMP.p_reg->TASKS_START = 1;
  while (true)
  {
    NRF_LOG_FLUSH();
    __WFI();
  }
}


void nrf_802154_transmitted_raw(const uint8_t* p_frame, uint8_t* p_ack, int8_t power, uint8_t lqi)
{
  tx_done = true;
  if (enable_pin_out)
  {
    nrf_gpio_pin_toggle(PIN_OUT_DEBUG_POST);
  }
  if (p_ack != NULL)
  {
    if (notif_on)
    {
    #if ENABLE_NRF_LOG_INFO
      NRF_LOG_INFO("ACK RECEIVED: ON");
      #endif
      TIMER_ACQUISITION.p_reg->TASKS_START = 1;
      nrf_drv_gpiote_in_event_enable(PIN_IN, true);
      bsp_board_led_on(NOTIF_ON_LED);
      begin_tx = true;
    } else
    {
      #if ENABLE_NRF_LOG_INFO
      NRF_LOG_INFO("ACK RECEIVED: OFF");
      #endif
      //TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[1] = 1;
      //NRF_LOG_INFO("Timer stopped at : %d", TIMER_TIMESTAMP.p_reg->CC[1]);
      // Disable PIN_IN sensing
      nrf_drv_gpiote_in_event_disable(PIN_IN);
      // NOTIF_ON_LED off
      bsp_board_led_off(NOTIF_ON_LED);
      begin_tx = false;
      tx_uart_buff[0] = (total_event & 0x00ff) >> 0;
      tx_uart_buff[1] = (total_event & 0xff00) >> 8;
      ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, tx_uart_buff, 20);
      APP_ERROR_CHECK(err_code);
      last_packet = true;
    }
  }
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("Trasmission done");
  #endif
  nrf_802154_sleep();
}

/*Non chiamata perche #define ACK_TIMEOUT settato a 0*/
void nrf_802154_tx_started (const uint8_t* p_frame)
{
  #if ENABLE_NRF_LOG_INFO
  NRF_LOG_INFO("Trasmission started");
  #endif
}



/**
 *@}
 **/
