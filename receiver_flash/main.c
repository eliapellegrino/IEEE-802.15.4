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
#include "nrf_drv_rtc.h"
#include "nrf_fstorage.h"
#include "nrf_fstorage_nvmc.h"
#include "nrf_delay.h"




#define FLASH_PAGE_START          0x20000
#define FLASH_PAGE_END            0xC0000
#define SIZE_TO_READ              20


#define WINDOW_LENGTH                  10

#define TX_BUFF_SIZE_UART              20

#define CONFIG_802154_COMPLETE_LED     BSP_BOARD_LED_0 /*I LED 1 e 2 non possono venire usati con 802.15.4*/
#define NOTIF_ON_LED                   BSP_BOARD_LED_1 /*On quando le notifiche sono attive*/
#define PACKET_LOST_LED                BSP_BOARD_LED_2 /*Toggle quando vien perso un pacchetto*/

#define PA_PIN                         NRF_GPIO_PIN_MAP(1,3)
#define LNA_PIN                        NRF_GPIO_PIN_MAP(1,4)
// PIN Define
#define PIN_OUT                        NRF_GPIO_PIN_MAP(1,1) /*PIN for recostruction of ATC*/
#define PIN_OUT_DEBUG                  NRF_GPIO_PIN_MAP(1,5) /*PIN for estimation of delay*/



bool enable_pin_out = true;
bool enable_uart = false;
bool enable_flash_write = true;
bool clean_flash = 0;


static uint32_t current_flash_address = FLASH_PAGE_START;
static volatile bool flash_write_busy = false;
static volatile bool uart_busy = false;

bool enable_reconstruction = false;

uint16_t mask_fc = 0x0001; /*Mask used to isolate bit of frame control field*/
uint32_t mask_int = 0x1UL;
int8_t index = 0;
uint8_t count = 0;
uint8_t packet_lost = 0;
uint8_t packet[MAX_PACKET_SIZE];
uint8_t tx_uart_buff[TX_BUFF_SIZE_UART] = {0x00};
bool stop_timer = false;
/*Non usare l'istanza 0 e 1 che buggano (la 2 non è stata provata, ma da documentazione dovrebbero comunque solamente usare due istanze del timer)*/
// Vedere se si possono disattivare in qualche moodo le altre istanze usate dai driver internamente
nrf_drv_timer_t TIMER_TIMESTAMP = NRF_DRV_TIMER_INSTANCE(2);
nrf_drv_timer_t TIMER_RECONSTRUCTION = NRF_DRV_TIMER_INSTANCE(3);
nrf_drv_timer_t TIMER_ACQUISITION = NRF_DRV_TIMER_INSTANCE(4);

//UART
const nrf_drv_uart_t uart_inst = NRF_DRV_UART_INSTANCE(0);

// RTC
const nrf_drv_rtc_t rtc = NRF_DRV_RTC_INSTANCE(0);




/*FSTORAGE MODULE*/
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt);
// Definisco l'stanza da usare per poter scrivere sulla flash
NRF_FSTORAGE_DEF(nrf_fstorage_t fstorage) =
{
    /* Set a handler for fstorage events. */
    .evt_handler = fstorage_evt_handler,

    /* These below are the boundaries of the flash space assigned to this instance of fstorage.
     * You must set these manually, even at runtime, before nrf_fstorage_init() is called.
     * The function nrf5_flash_end_addr_get() can be used to retrieve the last address on the
     * last page of flash available to write data. */
    .start_addr = FLASH_PAGE_START,
    .end_addr   = FLASH_PAGE_END,
};
static void fstorage_evt_handler(nrf_fstorage_evt_t * p_evt)
{
    if (p_evt->result != NRF_SUCCESS)
    {
        NRF_LOG_INFO("--> Event received: ERROR while executing an fstorage operation.");
        return;
    }

    switch (p_evt->id)
    {
        case NRF_FSTORAGE_EVT_WRITE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: wrote %d bytes at address 0x%x.",
                         p_evt->len, p_evt->addr);
                         flash_write_busy = false;
        } break;

        case NRF_FSTORAGE_EVT_ERASE_RESULT:
        {
            NRF_LOG_INFO("--> Event received: erased %d page from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
        case NRF_FSTORAGE_EVT_READ_RESULT:
        {
            NRF_LOG_INFO("--> Event received: read %d bytes from address 0x%x.",
                         p_evt->len, p_evt->addr);
        } break;
        default:
            break;
    }
}
static void clean_flash_memory()
{
  uint32_t tmp_addr = FLASH_PAGE_START;
  uint8_t tmp_buff[SIZE_TO_READ];
  uint8_t flash_buff[SIZE_TO_READ];
  uint8_t cmp_buff[20];
  memset(cmp_buff, 0xff, SIZE_TO_READ);
  //nrf_drv_uart_tx(&uart_inst, cmp_buff, SIZE_TO_READ);
  ret_code_t err_code;
  while(true)
    {
      NRF_LOG_INFO("Reading from: %x", tmp_addr);
      err_code = nrf_fstorage_read(&fstorage, tmp_addr, flash_buff, SIZE_TO_READ);
      APP_ERROR_CHECK(err_code);
      nrf_delay_ms(5);
      tmp_addr = tmp_addr+SIZE_TO_READ;
      memcpy(tmp_buff, flash_buff, SIZE_TO_READ);
      /* BUG */
      err_code = nrf_drv_uart_tx(&uart_inst, tmp_buff, SIZE_TO_READ);
      APP_ERROR_CHECK(err_code);
      if (memcmp(tmp_buff, cmp_buff, SIZE_TO_READ) == 0)
      {
        /*End of data*/
        break;
      }
      uart_busy = true;
      while (uart_busy)
      {
      }
      NRF_LOG_FLUSH();
    }
    NRF_LOG_INFO("Starting erasing from ");
    err_code = nrf_fstorage_erase(&fstorage, FLASH_PAGE_START, (tmp_addr-FLASH_PAGE_START)/4000+1, NULL);
    current_flash_address = FLASH_PAGE_START;
}
static void read_write_flash_init()
{
  nrf_fstorage_api_t* p_fs_api = &nrf_fstorage_nvmc;
  ret_code_t err_code = nrf_fstorage_init(&fstorage, p_fs_api, NULL);
  APP_ERROR_CHECK(err_code);
}




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
      uart_busy = false;
   } break;
   }
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
      TIMER_RECONSTRUCTION.p_reg->CC[0] = 100;
      TIMER_RECONSTRUCTION.p_reg->INTENSET = (uint32_t)mask_int << 16UL + 0;
      TIMER_RECONSTRUCTION.p_reg->TASKS_START = 1;

   } break;
   case BUTTON_2:
   {
      TIMER_RECONSTRUCTION.p_reg->TASKS_STOP = 1;
   } break;
   case BUTTON_3:
   {
    clean_flash = 1;
      bsp_board_led_on(BSP_BOARD_LED_2);
   } break;
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
   NRF_LOG_INFO("Short and Extended address correctly set: %x %x", laddr[6], laddr[7]);
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
      NRF_LOG_INFO("PA E LNA PIN SET");
   }

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
   NRF_LOG_INFO("Pan ID correctly set");
   //Set  radio output power (dBm) (@ref nrf52840_bitfields.h)
   int8_t power = 0;
   nrf_802154_tx_power_set(power);
   NRF_LOG_INFO("Power set to : %d dBm", power);
   // Set PA e LNA PIN
   set_pa_lna();
}
void timer_timestamp_event_handler(nrf_timer_event_t event_type, void* p_context)
{
//
}

void timer_acquisition_event_handler(nrf_timer_event_t event_type, void* p_context)
{
//
}
void timer_reconstruction_event_handler(nrf_timer_event_t event_type, void* p_context)
{
   ret_code_t err_code;
   nrf_gpio_pin_set(PIN_OUT);
   rtc.p_reg->TASKS_START = 1;
   //NRF_LOG_INFO("PIN OUT settatoo\n");
   TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
   NRF_LOG_INFO("event: %d ON at: %d", event_type, TIMER_TIMESTAMP.p_reg->CC[0]);
   if (packet[index] != 0)
   {
      TIMER_RECONSTRUCTION.p_reg->CC[(event_type - 320) / 4] = packet[index];
      TIMER_RECONSTRUCTION.p_reg->INTENSET = mask_int << 16UL + (event_type - 320) / 4;
      index++;
   } else
   {
      TIMER_RECONSTRUCTION.p_reg->INTENCLR = mask_int << 16UL + (event_type - 320) / 4;
      stop_timer = true;
   }
   if (stop_timer && TIMER_RECONSTRUCTION.p_reg->INTENCLR == 0)
   {
      TIMER_RECONSTRUCTION.p_reg->TASKS_STOP = 1;
      TIMER_RECONSTRUCTION.p_reg->TASKS_CLEAR = 1;
      NRF_LOG_INFO("STOPPED");
   }
   //NRF_LOG_INFO("INSIDE HANDLER");
}

static void timers_init()
{
   nrf_drv_timer_config_t timer_cfg = NRF_DRV_TIMER_DEFAULT_CONFIG;
   // TIMER TIMESTAMP
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_32;
   timer_cfg.frequency = NRF_TIMER_FREQ_1MHz;
   ret_code_t err_code = nrf_drv_timer_init(&TIMER_TIMESTAMP, &timer_cfg, timer_timestamp_event_handler);
   APP_ERROR_CHECK(err_code);
   // TIMER ACQUISITION
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_16;
   timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
   err_code = nrf_drv_timer_init(&TIMER_ACQUISITION, &timer_cfg, timer_acquisition_event_handler);
   APP_ERROR_CHECK(err_code);
   uint32_t time_ticks = (WINDOW_LENGTH * 0.001) * 31250;
   NRF_LOG_INFO("TIMEE TICKS: %d", time_ticks);
   nrf_drv_timer_extended_compare(&TIMER_ACQUISITION, NRF_TIMER_CC_CHANNEL0, time_ticks, NRF_TIMER_SHORT_COMPARE0_CLEAR_MASK, true);
   // TIMER RECONSTRUCTION
   timer_cfg.frequency = NRF_TIMER_FREQ_31250Hz;
   timer_cfg.bit_width = NRF_TIMER_BIT_WIDTH_8;
   err_code = nrf_drv_timer_init(&TIMER_RECONSTRUCTION, &timer_cfg, timer_reconstruction_event_handler);
   APP_ERROR_CHECK(err_code);
}

static void pins_init()
{
   //PIN OUT DEBUG
   nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);
   nrf_drv_gpiote_out_init(PIN_OUT_DEBUG, &out_config);
   nrf_drv_gpiote_out_init(PIN_OUT, &out_config);
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
      //.use_easy_dma = true,
   };
   err_code = nrf_drv_uart_init( & uart_inst, & uart_configuration, uart_event_handler);
   APP_ERROR_CHECK(err_code);
}

static void rtc_handler(nrf_drv_rtc_int_type_t int_type)
{

   if (int_type == NRFX_RTC_INT_TICK)
   {
      TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
      NRF_LOG_INFO("OFF: %d", TIMER_TIMESTAMP.p_reg->CC[0]);
      nrf_gpio_pin_clear(PIN_OUT);
      rtc.p_reg->TASKS_STOP = 1;
      //rtc.p_reg->TASKS_CLEAR = 1;

   }
}
/*
static void lfclk_config(void)
{
nrf_drv_clock_lfclk_request(NULL);
}
*/
static void rtc_init(void)
{
   uint32_t err_code;

   //Initialize RTC instance
   nrf_drv_rtc_config_t config = NRF_DRV_RTC_DEFAULT_CONFIG;
   config.prescaler = 8; /*f = 32768/(pr+1)*/
   config.interrupt_priority = 6;  /*Con priorità = 3 bugga, = 4 FATAL ERROR!!*/
   err_code = nrf_drv_rtc_init(&rtc, &config, rtc_handler);
   APP_ERROR_CHECK(err_code);

   //Enable tick event & interrupt (di default è disabilitato)
   nrf_drv_rtc_tick_enable(&rtc, true);
   // Tick event (low power) freq = 32768/(prescaler+1) = 3641Hz
}

int main(void)
{
   log_init();
   // Peripheral initialization
   led_init();
   buttons_init();
   timers_init();
   pins_init();
   uart_init();
   // 802.15.4 Configuration
   nrf_802154_init();
   config_802154();
   rtc_init();
   read_write_flash_init();
   // End of configuration
   bsp_board_led_on(CONFIG_802154_COMPLETE_LED);
   NRF_LOG_INFO("IEEE 802.15.4 RECEIVER");
   TIMER_TIMESTAMP.p_reg->TASKS_START = 1;
   bool result = nrf_802154_receive();
   NRF_LOG_INFO("Enter in RX state with code: %d", result);
   while (true)
   {
      NRF_LOG_FLUSH();
      __WFI();
      if (clean_flash)
      {
        clean_flash_memory();
        clean_flash = 0;
        bsp_board_led_off(BSP_BOARD_LED_2);
      }
   }
}


void nrf_802154_received_raw(uint8_t * p_data, int8_t power, uint8_t lqi)
{
   if (enable_reconstruction)
   {
      TIMER_RECONSTRUCTION.p_reg->TASKS_STOP = 1;
      TIMER_RECONSTRUCTION.p_reg->TASKS_CLEAR = 1;
   }
   /*Controllo la lunghezza del pacchetto ricevuto: se uguale a quello di notifica o di dati*/
   if (p_data[PHR_POS] == MAC_HEADER_SIZE + FCS_SIZE + NOTIF_PACKET_PAYLOAD_SIZE + PHR_SIZE)
   {
      count = p_data[MAC_PAYLOAD_POS];
      //NRF_LOG_INFO("Frame control: %x", p_data[FC_POS + 1] << 8 | p_data[FC_POS]);
      // Controllo il pending bit: se = 0 notifiche disattivate, se = 1 notifiche attive
      if ((mask_fc << FRAME_PENDING_BIT_POS) & (p_data[FC_POS + 1] << 8 | p_data[FC_POS]))
      {
         // Pending bit settato
         count = p_data[MAC_PAYLOAD_POS];
         //NRF_LOG_INFO("Notifiche on: %d", count);
         bsp_board_led_on(NOTIF_ON_LED);
      } else
      {
         // Pending bit non settato
         count = p_data[MAC_PAYLOAD_POS];
         //NRF_LOG_INFO("Notifiche off: %d", count);
         bsp_board_led_off(NOTIF_ON_LED);
         NRF_LOG_INFO("PACKET_LOST: %d", packet_lost);
         //ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, &packet_lost, 20);
         tx_uart_buff[0] = packet_lost;
         ret_code_t err_code = nrf_fstorage_write(&fstorage, current_flash_address, tx_uart_buff, sizeof(tx_uart_buff), NULL);
         APP_ERROR_CHECK(err_code);

         /*
         uint8_t tmp[10] = {0xff};
         ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, tmp, 10);
         */
      }
   }
   memcpy(packet, p_data, MAX_PACKET_SIZE);
   nrf_802154_buffer_free_raw(p_data);
   if (packet [PHR_POS] == MAX_PACKET_SIZE)
   {
      if (enable_pin_out)
      {
         nrf_gpio_pin_toggle(PIN_OUT_DEBUG);
      }
      count++;
      // Se il sequance number del pacchetto ricevuto != da quello che mi aspetto vuol dire cheè stato perso un pacchetto in aria
      if (count != packet[SN_POS])
      {
         TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
         //NRF_LOG_INFO("Problem: received: %d, count: %d, now: %d", packet[SN_POS], count, TIMER_TIMESTAMP.p_reg->CC[0]);
         // Bisognerebbe salvare questa variabile da qualche parte
         packet_lost++;
         count++;
         nrf_gpio_pin_toggle(PACKET_LOST_LED);
      } else
      {
         TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
         //NRF_LOG_INFO("Ok: received: %d, count: %d, now: %d", packet[SN_POS], count, TIMER_TIMESTAMP.p_reg->CC[0]);
         if (enable_uart)
         {
            ret_code_t err_code = nrf_drv_uart_tx(&uart_inst, tx_uart_buff, 20);
            APP_ERROR_CHECK(err_code);
         }
         if (enable_flash_write)
         {
                        tx_uart_buff[0] = packet[SN_POS];
            tx_uart_buff[1] = 1;
            tx_uart_buff[2] = packet[MAC_PAYLOAD_POS],
                              memcpy(&tx_uart_buff[3], &packet[MAC_PAYLOAD_POS + 1], 20 - 3);
            ret_code_t err_code = nrf_fstorage_write(&fstorage, current_flash_address, tx_uart_buff, sizeof(tx_uart_buff), NULL);
            APP_ERROR_CHECK(err_code);
            NRF_LOG_INFO("Write on flash at %x:",current_flash_address);
            current_flash_address = current_flash_address+sizeof(tx_uart_buff);
            flash_write_busy = true;            
         }
         if (enable_reconstruction)
         {
            // Ricostruzione onda su PIN_OUT (setto i valori nei compare e cerco un modo di settare quelli nuovi)
            index = MAC_PAYLOAD_POS + 1; //Contiene l'indice dell'ultima cella da puntare nel pacchetto ricevuto
            stop_timer = false;
            for (int i = MAC_PAYLOAD_POS; i < MAC_PAYLOAD_POS + 6; i++) // i < 6: uso il timer che può contenere fino a 6 copmare (TIMER 3 o 4)
            {
               if (packet[i] != 0)
               {
                  NRF_LOG_INFO("%d", packet[i]);
                  TIMER_RECONSTRUCTION.p_reg->CC[i - MAC_PAYLOAD_POS] = packet[i];
                  TIMER_RECONSTRUCTION.p_reg->INTENSET = (uint32_t)mask_int << 16UL + (i - MAC_PAYLOAD_POS);
                  index = i + 1;
                  NRF_LOG_INFO("INSID RECONSTRUCTION");
               } else
               {
                  // Se trovo uno zero esco dal ciclo (tutti i compare sono stati settati)
                  break;
               }
            }
            if (index != MAC_PAYLOAD_POS)
            {
               NRF_LOG_INFO("TIMER_STARTED");
               NRF_LOG_INFO("%x", TIMER_RECONSTRUCTION.p_reg->INTENSET);
               TIMER_RECONSTRUCTION.p_reg->TASKS_START = 1;// Start the timer
            }
         }


      }
   }
   /*
   else
   {
   count = packet[MAC_PAYLOAD_POS];
   NRF_LOG_INFO("Count received: %d", count);
   }
   */
}

void nrf_802154_receive_failed(nrf_802154_rx_error_t error)
{
   //TIMER_TIMESTAMP.p_reg->TASKS_CAPTURE[0] = 1;
   NRF_LOG_INFO("Error: %d, TIme: %d", error, TIMER_TIMESTAMP.p_reg->CC[0]);
   packet_lost++;
}

void nrf_802154_tx_ack_started(const uint8_t* p_data)
{
//NRF_LOG_INFO("Sending ACK");
}
/**
*@}
**/

