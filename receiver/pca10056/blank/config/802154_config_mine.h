// Channel from 11 to 26
#ifndef CHANNEL
#define CHANNEL 11
#endif

// PAN ID (2 BYTES LITTLE-ENDIAN)
#ifndef PAN_ID
#define PAN_ID 0xBABA
#endif

// Short Address Source (2B little-endian)
#ifndef SHORT_ADDR_SOURCE
#define SHORT_ADDR_SOURCE 0xDEDB
#endif
// Extended Address (8B little-endian)
#ifndef EXT_ADDR
#define EXT_ADDR 0xBABA10203040DEDB
#endif

// Short Address Destination (2B little-endian)
#ifndef SHORT_ADDR_DEST
#define SHORT_ADDR_DEST 0xDEDA
#endif

/*Payload Definition: used to construct payload. Every single field must be set. See IEEE 802.15.4-2006 section 7.2*/
#define PHR_SIZE                      1   /*Phy header*/
// Mac header
#define FC_SIZE                       2   /*Frame control*/
#define SN_SIZE                       1   /*Sequence Number*/
#define PAN_DEST_SIZE                 2   /*Destination PAN*/
#define SA_DEST_SIZE                  2   /*Short address destination*/
#define PAN_SOURCE_SIZE               0   /*Source PAN (if = 0, fc_b6 = 1, --> PAN_DEST = PAN_SOURCE)*/
#define SA_SOURCE_SIZE                2   /*Short address destination*/
#define AUX_SEC_HEADER_SIZE           0   /*Auxiliary secuirty header (se = 0, fc_b3 = 0)*/
#define MAC_HEADER_SIZE               (FC_SIZE+SN_SIZE+PAN_DEST_SIZE+SA_DEST_SIZE+PAN_SOURCE_SIZE+SA_SOURCE_SIZE+AUX_SEC_HEADER_SIZE)   /*Mac header*/
#define MAX_PAYLOAD_SIZE              100 /*Mac Payload (without header)*/
#define FCS_SIZE                      2   /*Frame Check Sequence(CRC)*/
#define MAX_PACKET_SIZE               (MAX_PAYLOAD_SIZE+PHR_SIZE+FCS_SIZE+MAC_HEADER_SIZE)
#define NOTIF_PACKET_PAYLOAD_SIZE     1 /*Payload size of notif packet*/                  
/*Position of packet's fields: these positions are dependant from Frame Control: in our application we set only 1 PAN ID (the destination and source PAN_ID are equals) e no security header!*/
#define PHR_POS                       0  /*Index of PHR inside packet (as in c: 0 --> first position)*/
#define FC_POS                        1  /*Index of frame control (FC)*/
#define SN_POS                        3  /*Index of Sequence Number*/
#define PAN_DEST_POS                  4  /*Index of Pan Destination*/
#define SA_DEST_POS                   6  /*Index of Short address destination*/
#define SA_SOURCE_POS                 8  /*Index of Short address source*/
#define MAC_PAYLOAD_POS               10  /*Index of Mac Paylod*/

/*Bit position (starting from 0) of various bit */
#define AUX_SEC_HEADER_BIT_POS        3 /*Position of Auxiliary Securty Header bit inside frame control field*/
#define FRAME_PENDING_BIT_POS         4 /*Position of Frame Pending bit inside frame control field*/
#define ACK_BIT_POS                   5 /*Position of ACK bit inside frame control field*/
#define PAN_ID_COMPRESS_BIT_POS       6 /*Position of Pan id compresion bit position in frame control field*/

