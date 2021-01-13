# RECEIVER
This implements the **RECEIVER** board for our scenario, with a basic implementation of IEEE 802.15.4. The position of relevant bit, and other important configuration can be found in **802154_config_mine.h**.
Once the board is set up, the antenna is in the receiver state, ready for listening for packet. If the received packet is long as the @ref **NOTIF_PACKET_PAYLOAD_SIZE** (plus overhead), it checks for pending bit in Frame Control bytes: if the pending bit is set, the notification will be on, the receiver will ack this packet and the transmitter will start sending packet until another notification packet with pending bit not set is received (and ack).
The *notification on and off packets* are the only packets that are ACKED, the data packets aren't.
If the correspondig flag are sets, the received packet is written on serial port, and a pin is toggled. Moreover, if *enable_reconstruction* is True, the ATC signal will be reconstructed.

## BUTTONs
Used only for debug purpose

## LEDs
1) ON when IEEE 802.15.4 configuration is over
2) ON when notification are on

## PINs
(1,1): PIN_OUT for reconstruction
(1,3): PA_PIN for Power Amplifier 
(1,4): LNA_PIN for Low NOise Amplifier
(1,5): PIN_OUT_DEBUG for delay studies