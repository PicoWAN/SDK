/*
 * lora_mac - LoRaWAN MAC implementation
 *
 * Some parts have been adapted from Semtech's implementation:
 * https://github.com/Lora-net/LoRaMac-node
 *
 * Copyright (c) 2018, Archos S.A.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in the
 *   documentation and/or other materials provided with the distribution.
 * * Neither the name of Archos nor the names of its contributors may be
 *   used to endorse or promote products derived from this software without
 *   specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS ``AS IS'' AND
 * AND EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL ARCHOS S.A. BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <common.h>
#include <mac_interface.h>
#include <message.h>
#include <osal.h>
#include <stddef.h>
#include "lora_mac.h"
#include "lora_mac_internal.h"

// preamble for lora networks
#define MAC_PREAMBLE			0x34
#define PREAMBLE_LENGTH			8

#define RX_TX_CR			OSAL_CR4_5
#define RX_TX_CRC			OSAL_CRC_ON

#define RADIO_CONFIG_TIME		3000 // us
#define CPU_WAKEUP_TIME			5 // ms

//#define CFG_SUPPORT_CLASS_C

/*!
 * Returns individual channel mask
 *
 * \param[IN] channelIndex Channel index 1 based
 * \retval channelMask
 */
#define LC(channelIndex)		(uint16_t) (1 << (channelIndex-1))

/*!
 * Maximum PHY layer payload size
 */
#define LORAMAC_PHY_MAXPAYLOAD		255

/*!
 * Maximum MAC commands buffer size
 */
#define LORA_MAC_COMMAND_MAX_LENGTH	15

/*!
 * FRMPayload overhead to be used when setting the RADIO.payload_length
 * in rx_window_setup function.
 * Maximum PHYPayload = MaxPayloadOfDatarate/MaxPayloadOfDatarateRepeater + LORA_MAC_FRMPAYLOAD_OVERHEAD
 */
#define LORA_MAC_FRMPAYLOAD_OVERHEAD	13 // MHDR(1) + FHDR(7) + Port(1) + MIC(4)

/*!
 * Class A&B receive delay 1 in ms
 */
#define RECEIVE_DELAY1			1000

/*!
 * Class A&B receive delay 2 in ms
 */
#define RECEIVE_DELAY2			2000

/*!
 * Join accept receive delay 1 in ms
 */
#define JOIN_ACCEPT_DELAY1		5000

/*!
 * Join accept receive delay 2 in ms
 */
#define JOIN_ACCEPT_DELAY2		6000

/*!
 * Time to prevent delay before configuring the radio
 */
#define MARGIN_TIME			1000

/*!
 * Class A&B maximum receive window delay in ms
 */
#define MAX_RX_WINDOW			3000

/*!
 * Maximum allowed gap for the FCNT field
 */
#define MAX_FCNT_GAP			16384

/*!
 * ADR acknowledgement counter limit
 */
#define ADR_ACK_LIMIT			64

/*!
 * Number of ADR acknowledgement requests before returning to default datarate
 */
#define ADR_ACK_DELAY			32

/*!
 * Number of seconds after the start of the second reception window without
 * receiving an acknowledge.
 * AckTimeout = \ref ACK_TIMEOUT + Random(-\ref ACK_TIMEOUT_RND, \ref ACK_TIMEOUT_RND)
 */
#define ACK_TIMEOUT			2000

/*!
 * Random number of seconds after the start of the second reception window without
 * receiving an acknowledge
 * AckTimeout = \ref ACK_TIMEOUT + Random(-\ref ACK_TIMEOUT_RND, \ref ACK_TIMEOUT_RND)
 */
#define ACK_TIMEOUT_RND			1000

/*!
 * Maximum number of times the MAC layer tries to get an acknowledge.
 */
#define MAX_ACK_RETRIES			8

/*!
 * LoRaMAC channels parameters definition
 */
typedef union u_dr_range {
	/*!
	 * Byte-access to the bits
	 */
	int8_t value;
	/*!
	 * Structure to store the minimum and the maximum datarate
	 */
	struct s_fields {
		/*!
		 * Minimum data rate
		 *
		 * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
		 *
		 * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
		 */
		int8_t min : 4;
		/*!
		 * Maximum data rate
		 *
		 * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
		 *
		 * US915 - [DR_0, DR_1, DR_2, DR_3, DR_4]
		 */
		int8_t max : 4;
	} bits;
} dr_range_t;

/*!
 * LoRaMAC band parameters definition
 */
typedef struct s_band {
	/*!
	 * Duty cycle
	 */
	uint16_t d_cycle;
	/*!
	 * Maximum Tx power
	 */
	int8_t tx_max_power;
	/*!
	 * Time stamp of the last Tx frame
	 */
	osal_time_t last_tx_done_time;
	/*!
	 * Holds the time where the device is off
	 */
	osal_time_t time_off;
} band_lora_t;

/*!
 * LoRaMAC channel definition
 */
typedef struct s_channel_params {
	/*!
	 * Uplink frequency in Hz
	 */
	uint32_t uplink_frequency;
	/*!
	* RX1 downlink frequency in Hz
	*/
	uint32_t rx1_downlink_frequency;
	/*!
	 * Data rate definition
	 */
	dr_range_t dr_range;
	/*!
	 * Band index
	 */
	uint8_t band;
} channel_params_t;

/*!
 * LoRaMAC receive window 2 channel parameters
 */
typedef struct s_rx2_channel_params {
	/*!
	 * Frequency in Hz
	 */
	uint32_t frequency;
	/*!
	 * Data rate
	 *
	 * EU868 - [DR_0, DR_1, DR_2, DR_3, DR_4, DR_5, DR_6, DR_7]
	 *
	 * US915 - [DR_8, DR_9, DR_10, DR_11, DR_12, DR_13]
	 */
	uint8_t datarate;
} rx2_channel_params_t;

/*!
 * Global MAC layer parameters
 */
typedef struct s_loramac_params {
	/*!
	 * Channels TX power
	 */
	int8_t channels_tx_power;
	/*!
	 * Channels data rate
	 */
	int8_t channels_datarate;
	/*!
	 * LoRaMac maximum time a reception window stays open
	 */
	uint32_t max_rx_window;
	/*!
	 * Receive delay 1
	 */
	uint32_t receive_delay_1;
	/*!
	 * Receive delay 2
	 */
	uint32_t receive_delay_2;
	/*!
	 * Join accept delay 1
	 */
	uint32_t join_accept_delay_1;
	/*!
	 * Join accept delay 1
	 */
	uint32_t join_accept_delay_2;
	/*!
	 * Number of uplink messages repetitions [1:15] (unconfirmed messages only)
	 */
	uint8_t channels_nb_rep;
	/*!
	 * Datarate offset between uplink and downlink on first window
	 */
	uint8_t rx1_dr_offset;
	/*!
	 * LoRaMAC 2nd reception window settings
	 */
	rx2_channel_params_t rx2_channel;
	/*!
	 * Mask indicating which channels are enabled
	 */
	uint16_t channels_mask[6];
	/*!
	 * Number minimum of symbols to detect frame
	 */
	uint8_t min_rx_symbols;
	/*!
	 * Maximun timing error of the receiver in microseconds
	 */
	uint32_t max_rx_error;
} loramac_params_t;

/*!
 * LoRaMAC multicast channel parameter
 */
typedef struct s_multicast_params {
	/*!
	 * Address
	 */
	uint32_t address;
	/*!
	 * Network session key
	 */
	uint8_t nwkskey[16];
	/*!
	 * Application session key
	 */
	uint8_t appskey[16];
	/*!
	 * Downlink counter
	 */
	uint32_t downlink_counter;
	/*!
	 * Reference pointer to the next multicast channel parameters in the list
	 */
	struct s_multicast_params *next;
} multicast_params_t;

/*!
 * LoRaMAC mote MAC commands
 */
typedef enum e_loramac_mote_cmd {
	/*!
	 * LinkCheckReq
	 */
	MOTE_MAC_LINK_CHECK_REQ		= 0x02,
	/*!
	 * LinkADRAns
	 */
	MOTE_MAC_LINK_ADR_ANS		= 0x03,
	/*!
	 * DutyCycleAns
	 */
	MOTE_MAC_DUTY_CYCLE_ANS		= 0x04,
	/*!
	 * RXParamSetupAns
	 */
	MOTE_MAC_RX_PARAM_SETUP_ANS	= 0x05,
	/*!
	 * DevStatusAns
	 */
	MOTE_MAC_DEV_STATUS_ANS		= 0x06,
	/*!
	 * NewChannelAns
	 */
	MOTE_MAC_NEW_CHANNEL_ANS	= 0x07,
	/*!
	 * RXTimingSetupAns
	 */
	MOTE_MAC_RX_TIMING_SETUP_ANS	= 0x08,
	/*!
	 * TXParamSetupAns
	 */
	MOTE_MAC_TX_PARAM_SETUP_ANS      = 0x09,
	/*!
	 * DlChannelAns
	 */
	MOTE_MAC_DL_CHANNEL_ANS          = 0x0A,
} loramac_mote_cmd_t;

/*!
 * LoRaMAC server MAC commands
 */
typedef enum e_loramac_srv_cmd {
	/*!
	 * LinkCheckAns
	 */
	SRV_MAC_LINK_CHECK_ANS		= 0x02,
	/*!
	 * LinkADRReq
	 */
	SRV_MAC_LINK_ADR_REQ		= 0x03,
	/*!
	 * DutyCycleReq
	 */
	SRV_MAC_DUTY_CYCLE_REQ		= 0x04,
	/*!
	 * RXParamSetupReq
	 */
	SRV_MAC_RX_PARAM_SETUP_REQ	= 0x05,
	/*!
	 * DevStatusReq
	 */
	SRV_MAC_DEV_STATUS_REQ		= 0x06,
	/*!
	 * NewChannelReq
	 */
	SRV_MAC_NEW_CHANNEL_REQ		= 0x07,
	/*!
	 * RXTimingSetupReq
	 */
	SRV_MAC_RX_TIMING_SETUP_REQ	= 0x08,
	/*!
	 * TxParamSetupReq
	 */
	SRV_MAC_TX_PARAM_SETUP_REQ	= 0x09,
	/*!
	 * DlChannelReq
	 */
	SRV_MAC_DL_CHANNEL_REQ		= 0x0A,
} loramac_srv_cmd_t;

/*!
 * LoRaMAC Battery level indicator
 */
typedef enum e_loramac_battery_level {
	/*!
	 * External power source
	 */
	BAT_LEVEL_EXT_SRC		= 0x00,
	/*!
	 * Battery level empty
	 */
	BAT_LEVEL_EMPTY			= 0x01,
	/*!
	 * Battery level full
	 */
	BAT_LEVEL_FULL			= 0xFE,
	/*!
	 * Battery level - no measurement available
	 */
	BAT_LEVEL_NO_MEASURE		= 0xFF,
} loramac_battery_level_t;

/*!
 * LoRaMAC Status
 */
typedef enum e_loramac_status {
	/*!
	 * Service started successfully
	 */
	LORAMAC_STATUS_OK,
	/*!
	 * Service not started - LoRaMAC is busy
	 */
	LORAMAC_STATUS_BUSY,
	/*!
	 * Service unknown
	 */
	LORAMAC_STATUS_SERVICE_UNKNOWN,
	/*!
	 * Service not started - invalid parameter
	 */
	LORAMAC_STATUS_PARAMETER_INVALID,
	/*!
	 * Service not started - invalid frequency
	 */
	LORAMAC_STATUS_FREQUENCY_INVALID,
	/*!
	 * Service not started - invalid datarate
	 */
	LORAMAC_STATUS_DATARATE_INVALID,
	/*!
	 * Service not started - invalid frequency and datarate
	 */
	LORAMAC_STATUS_FREQ_AND_DR_INVALID,
	/*!
	 * Service not started - the device is not in a LoRaWAN
	 */
	LORAMAC_STATUS_NO_NETWORK_JOINED,
	/*!
	 * Service not started - payload lenght error
	 */
	LORAMAC_STATUS_LENGTH_ERROR,
	/*!
	 * Service not started - payload lenght error
	 */
	LORAMAC_STATUS_MAC_CMD_LENGTH_ERROR,
	/*!
	 * Service not started - the device is switched off
	 */
	LORAMAC_STATUS_DEVICE_OFF,
} loramac_status_t;

typedef enum {
	LORAMAC_STATE_IDLE = 0,				///< LORAMAC is in an idle state
	LORAMAC_STATE_CHECKING_DUTY_CYCLE = 1,		///< LORAMAC is checking duty cycle
	LORAMAC_STATE_SENDING_DATA = 2,			///< LORAMAC is sending data
	LORAMAC_STATE_WAITING_OPENING_WINDOW = 3,	///< LORAMAC is waiting to open receive window
	LORAMAC_STATE_WAITING_DATA_RX1 = 4,		///< LORAMAC is waiting data in RX1
	LORAMAC_STATE_WAITING_DATA_RX2 = 5,		///< LORAMAC is waiting data in RX2
	LORAMAC_STATE_DOWNLINK_IDLE = 6,		///< LORAMAC is waiting data in RX2 but ready to send (state of class C)
} loramac_state_t;

/*!
 * LoRaMac maximum number of channels
 */
#define LORA_MAX_NB_CHANNELS		16

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_TX_MIN_DATARATE		DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define LORAMAC_TX_MAX_DATARATE		DR_7

/*!
 * Minimal datarate that can be used by the node
 */
#define LORAMAC_RX_MIN_DATARATE		DR_0

/*!
 * Maximal datarate that can be used by the node
 */
#define LORAMAC_RX_MAX_DATARATE		DR_7

/*!
 * Default datarate used by the node
 */
#define LORAMAC_DEFAULT_DATARATE	DR_0

/*!
 * Minimal Rx1 receive datarate offset
 */
#define LORAMAC_MIN_RX1_DR_OFFSET	0

/*!
 * Maximal Rx1 receive datarate offset
 */
#define LORAMAC_MAX_RX1_DR_OFFSET	5

/*!
 * Minimal Tx output power that can be used by the node
 */
#define LORAMAC_MIN_TX_POWER		TX_POWER_02_DBM

/*!
 * Maximal Tx output power that can be used by the node
 */
#define LORAMAC_MAX_TX_POWER		TX_POWER_16_DBM

/*!
 * Default Tx output power used by the node
 */
#define LORAMAC_DEFAULT_TX_POWER	TX_POWER_16_DBM

/*!
 * LoRaMac TxPower definition
 */
#define TX_POWER_16_DBM			0
#define TX_POWER_14_DBM			1
#define TX_POWER_12_DBM			2
#define TX_POWER_10_DBM			3
#define TX_POWER_08_DBM			4
#define TX_POWER_06_DBM			5
#define TX_POWER_04_DBM			6
#define TX_POWER_02_DBM			7

/*!
 * LoRaMac datarates definition
 */
#define DR_0 				0  // SF12 - BW125
#define DR_1 				1  // SF11 - BW125
#define DR_2 				2  // SF10 - BW125
#define DR_3 				3  // SF9  - BW125
#define DR_4 				4  // SF8  - BW125
#define DR_5 				5  // SF7  - BW125
#define DR_6 				6  // SF7  - BW250
#define DR_7 				7  // FSK

/*!
 * Second reception window channel definition.
 */
// Channel = {Frequency [Hz], Datarate}
#define RX_WND_2_CHANNEL		{869525000, DR_0}

/*!
 * LoRaMac maximum number of bands
 */
#define LORA_MAX_NB_BANDS		5

// Band = {DutyCycle, TxMaxPower, LastTxDoneTime, TimeOff}
#define BAND0				{100 , TX_POWER_16_DBM, 0, 0} //  1.0 %
#define BAND1				{100 , TX_POWER_16_DBM, 0, 0} //  1.0 %
#define BAND2				{1000, TX_POWER_16_DBM, 0, 0} //  0.1 %
#define BAND3				{10  , TX_POWER_16_DBM, 0, 0} // 10.0 %
#define BAND4				{100 , TX_POWER_16_DBM, 0, 0} //  1.0 %

/*!
 * LoRaMac EU868 default bands
 */
typedef enum {
	BAND_G1_0,
	BAND_G1_1,
	BAND_G1_2,
	BAND_G1_3,
	BAND_G1_4,
} band_id_t;

/*!
 * LoRaMac default channels
 */
// Channel = {Frequency [Hz], RX1 Frequency [Hz], {((DrMax << 4) | DrMin)}, Band}
#define LC1				{868100000, 0, {((DR_5 << 4) | DR_0)}, 1}
#define LC2				{868300000, 0, {((DR_5 << 4) | DR_0)}, 1}
#define LC3				{868500000, 0, {((DR_5 << 4) | DR_0)}, 1}

/*!
 * LoRaMac duty cycle for the back-off procedure
 */
#define BACKOFF_DC_1_HOUR		100
#define BACKOFF_DC_10_HOURS		1000
#define BACKOFF_DC_24_HOURS		10000

#define BACKOFF_RND_OFFSET		600000

/*!
 * LoRaMac channels which are allowed for the join procedure
 */
#define JOIN_CHANNELS			(uint16_t) (LC(1) | LC(2) | LC(3))


/*!
 * Device IEEE EUI
 */
static uint8_t dev_deveui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * Application IEEE EUI
 */
static uint8_t dev_appeui[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * AES encryption/decryption cipher application key
 */
static uint8_t dev_appkey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * AES encryption/decryption cipher network session key
 */
static uint8_t dev_nwkskey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

/*!
 * AES encryption/decryption cipher application session key
 */
static uint8_t dev_appskey[] = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};

static uint8_t last_demod_margin = 0;
static uint8_t last_nb_gateways = 0;

static mac_message_callbacks_t *message_callbacks = NULL;
static mac_battery_callback_t *battery_callback = NULL;
static mac_flash_callback_t *flash_callback = NULL;

static mac_tx_info_t mac_tx_info;
static mac_rx_info_t mac_rx_info;
static mac_network_info_t mac_network_info;

osal_job_t rx1_job;
osal_job_t rx2_job;
osal_job_t start_rx_single_job;
osal_job_t start_rx_continuous_job;
osal_job_t tx_delayed_job;
osal_job_t ack_timeout_job;
osal_job_t lock_low_power_job;
osal_job_t tx_done_cb_job;
osal_job_t mac_state_cb_job;

static uint32_t mac_netid = 0;
static uint32_t mac_devaddr = 0;

static message_t message_tx_rx;

/*!
 * If radio finish to receive
 */
uint8_t open_receive_window = 1;

/*!
 * Indicates if the MAC layer has already joined a network.
 */
static uint8_t is_loramac_network_joined = 0;

/*!
 * Indicates if the MAC layer is in ABP mode.
 */
static uint8_t is_loramac_network_ABP = 0;

/*!
 * Request to check the network.
 */
static uint8_t network_check_request = 0;

/*!
 * LoRaMac ADR control status
 */
static uint8_t adr_ctrl_on = 0;

/*!
 * Counts the number of missed ADR acknowledgements
 */
static uint32_t adr_ack_counter = 0;

/*!
 * Indicates if the MAC layer wants to send MAC commands
 */
static uint8_t mac_commands_in_next_tx = 0;

/*!
 * Contains the current mac_commands_buffer index
 */
static uint8_t mac_commands_buffer_index = 0;

/*!
 * Contains the current MacCommandsBuffer index for MAC commands to repeat
 */
static uint8_t mac_commands_buffer_to_repeat_index = 0;

/*!
 * Buffer containing the MAC layer commands
 */
static uint8_t mac_commands_buffer[LORA_MAC_COMMAND_MAX_LENGTH];

/*!
 * Buffer containing the MAC layer commands which must be repeated
 */
static uint8_t mac_commands_buffer_to_repeat[LORA_MAC_COMMAND_MAX_LENGTH];

/*!
 * Device nonce is a random value extracted by issuing a sequence of RSSI
 * measurements
 */
static uint16_t loramac_dev_nonce = 0;

/*!
 * Multicast channels linked list
 */
static multicast_params_t *multicast_channels = NULL;

/*!
 * Actual device class
 */
static device_class_t loramac_device_class;

/*!
 * Indicates if the node supports repeaters
 */
static uint8_t repeater_support;

/*!
 * Indicates current datarate
 */
static int8_t current_datarate;

/*!
 * LoRaMac parameters
 */
loramac_params_t loramac_params;

/*!
 * LoRaMac default parameters
 */
loramac_params_t loramac_params_defaults;

/*!
 * Uplink messages repetitions counter
 */
static uint8_t channels_nb_rep_counter = 0;

/*!
 * LoRaMAC frame counter. Each time a packet is sent the counter is incremented.
 * Only the 16 LSB bits are sent
 */
static uint32_t uplink_counter = 1;

/*!
 * LoRaMAC frame counter. Each time a packet is received the counter is incremented.
 * Only the 16 LSB bits are received
 */
static uint32_t downlink_counter = 0;

/*!
 * Maximum duty cycle
 * \remark Possibility to shutdown the device.
 */
static uint8_t max_duty_cycle = 0;

/*!
 * Aggregated duty cycle management
 */
static uint16_t aggregated_duty_cycle;
static osal_time_t aggregated_last_tx_done_time;
static osal_time_t aggregated_time_off;

/*!
 * Enables/Disables duty cycle management (Test only)
 */
static uint8_t duty_cycle_on;

/*!
 * Current channel index
 */
static uint8_t current_channel;

/*!
 * Channel index of the last transmission
 */
static uint8_t last_tx_channel;

/*!
 * LoRaMAC internal state
 */
loramac_state_t loramac_state = LORAMAC_STATE_IDLE;

/*!
 * MAC state
 */
uint8_t mac_state = MAC_STATE_IDLE;

/*!
 * Allow to add or remove a channel when Tx is running
 */
uint8_t allow_change_channel = 0;

/*!
 * LoRaMac reception windows delay
 * \remark normal frame: RxWindowXDelay = ReceiveDelayX - RADIO_WAKEUP_TIME - CPU_WAKEUP_TIME
 *         join frame  : RxWindowXDelay = JoinAcceptDelayX - RADIO_WAKEUP_TIME - CPU_WAKEUP_TIME
 */
static uint32_t rx_windows1_delay;
static uint32_t rx_windows2_delay;

/*!
 * Number of trials to get a frame acknowledged
 */
static uint8_t ack_timeout_retries = 1;

/*!
 * Number of trials to get a frame acknowledged
 */
static uint8_t ack_timeout_retries_counter = 1;

/*!
 * Variable to see if ack timeout event was reach
 */
static uint8_t ack_timeout_reached = 0;

/*!
 * Last transmission time on air
 */
osal_time_t tx_time_on_air = 0;

/*!
 * Save timestamp at the end of transmission to know when to open receive windows
 */
osal_time_t timestamp_tx_done = 0;

static uint8_t lowpower_sleep_locked = 0;

/*!
 * Data rates table definition
 */
const uint8_t datarate_table[] = {OSAL_SF12, OSAL_SF11, OSAL_SF10, OSAL_SF9, OSAL_SF8, OSAL_SF7, OSAL_SF7, 50};

/*!
 * Maximum payload with respect to the datarate index. Cannot operate with repeater.
 */
const uint8_t max_payload_of_datarate[] = {64, 64, 64, 128, 255, 255, 255, 255};

/*!
 * Maximum payload with respect to the datarate index. Can operate with repeater.
 */
const uint8_t max_payload_of_datarate_repeater[] = {64, 64, 64, 128, 235, 235, 235, 235};

/*!
 * Tx output powers table definition
 */
const int8_t tx_powers[] = {16, 14, 12, 10, 8, 6, 4, 2};

/*!
 * LoRaMac bands
 */
static band_lora_t bands[LORA_MAX_NB_BANDS] = {BAND0, BAND1, BAND2, BAND3, BAND4};

/*!
 * LoRaMAC channels
 */
static channel_params_t channels[LORA_MAX_NB_CHANNELS] = {LC1, LC2, LC3};


typedef struct {
	/*
	 * Frequency of this channel
	 */
	uint32_t freq;

	/*
	 * Spreading factor
	 */
	enum osal_sf_t sf;

	/*
	 * Bandwidth
	 */
	enum osal_bw_t bw;

	/*
	 * Max TX Power in dBm
	 */
	int8_t pmax;

	/*
	 * Channel usage in us
	 */
	uint32_t used_us;

	/*
	 * Subband of the channel (Used for usage accounting)
	 */
	uint8_t subband;
} lora_channel_t;

lora_channel_t tx_rx_channel;

/*!
 * Holds the current rx window slot
 */
static uint8_t rx_slot = 0;

/*!
 * Holds the current radio parameters
 */
static osal_radio_params_t radio_params;

static void prepare_rx_done_abort(void);
static void internal_rxdone_cb(void);
static void internal_txdone_cb(void);

static void send_packet(uint32_t freq, enum osal_sf_t sf, enum osal_bw_t bw, uint8_t *data, int len, uint8_t power);
static void loramac_init_activation_personalization(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey);

/*
 * \brief Sets the duty cycle for retransmissions
 *
 * \retval Duty cycle
 */
static uint16_t retransmission_duty_cycle(void);

/*
 * \brief Calculates the back-off time for the band of a channel.
 *
 * \param [IN] channel     The last Tx channel index
 */
static void calculate_back_off(uint8_t channel);

/*
 * \brief Schedules the frame according to the duty cycle
 *
 * \retval Status of the operation
 */
static loramac_status_t schedule_tx(void);

/*!
 * \brief Verifies, if a value is in a given range.
 *
 * \param value Value to verify, if it is in range
 *
 * \param min Minimum possible value
 *
 * \param max Maximum possible value
 *
 * \retval Returns the maximum value
 */
static inline uint8_t value_in_range(int8_t value, int8_t min, int8_t max);

/*!
 * \brief Adds a new MAC command to be sent.
 *
 * \Remark MAC layer internal function
 *
 * \param [in] cmd MAC command to be added
 *                 [MOTE_MAC_LINK_CHECK_REQ,
 *                  MOTE_MAC_LINK_ADR_ANS,
 *                  MOTE_MAC_DUTY_CYCLE_ANS,
 *                  MOTE_MAC_RX2_PARAM_SET_ANS,
 *                  MOTE_MAC_DEV_STATUS_ANS
 *                  MOTE_MAC_NEW_CHANNEL_ANS]
 * \param [in] p1  1st parameter (optional depends on the command)
 * \param [in] p2  2nd parameter (optional depends on the command)
 *
 * \retval status  Function status [0: OK, 1: Unknown command, 2: Buffer full]
 */
static uint8_t add_mac_command(uint8_t cmd, uint8_t p1, uint8_t p2);

/*!
 * \brief Decodes MAC commands
 */
static void process_mac_command(uint8_t *mac_cmd_buffer, uint8_t mac_cmd_buffer_len, int8_t snr);

/*!
 * \brief Parses the MAC commands which must be repeated.
 *
 * \Remark MAC layer internal function
 *
 * \param [IN] cmdBufIn  Buffer which stores the MAC commands to send
 * \param [IN] length  Length of the input buffer to parse
 * \param [OUT] cmdBufOut  Buffer which stores the MAC commands which must be
 *                         repeated.
 *
 * \retval Size of the MAC commands to repeat.
 */
static uint8_t parse_mac_commands_to_repeat(uint8_t *cmd_buf_in, uint8_t length, uint8_t *cmd_buf_out);

/*!
 * \brief   LoRaMAC channel add service
 *
 * \details Adds a new channel to the channel list and activates the id in
 *          the channel mask. For the US915 band, all channels are enabled
 *          by default. It is not possible to activate less than 6 125 kHz
 *          channels.
 *
 * \param   [IN] id - Id of the channel. Possible values are:
 *
 *          0-15 for EU868
 *          0-72 for US915
 *
 * \param   [IN] params - Channel parameters to set.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
loramac_status_t loramac_channel_add(uint8_t id, channel_params_t params);

/*!
 * \brief   LoRaMAC channel remove service
 *
 * \details Deactivates the id in the channel mask.
 *
 * \param   [IN] id - Id of the channel.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
loramac_status_t loramac_channel_remove(uint8_t id);

/*!
 * \brief   LoRaMAC multicast channel link service
 *
 * \details Links a multicast channel into the linked list.
 *
 * \param   [IN] channelParam - Multicast channel parameters to link.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
loramac_status_t loramac_multicast_channel_link(multicast_params_t *channel_param);

/*!
 * \brief   LoRaMAC multicast channel unlink service
 *
 * \details Unlinks a multicast channel from the linked list.
 *
 * \param   [IN] channelParam - Multicast channel parameters to unlink.
 *
 * \retval  LoRaMacStatus_t Status of the operation. Possible returns are:
 *          \ref LORAMAC_STATUS_OK,
 *          \ref LORAMAC_STATUS_BUSY,
 *          \ref LORAMAC_STATUS_PARAMETER_INVALID.
 */
loramac_status_t loramac_multicast_channel_unlink(multicast_params_t *channel_param);

/*!
 * \brief Disables channel in a specified channel mask
 *
 * \param [IN] id - Id of the channel
 *
 * \param [IN] mask - Pointer to the channel mask to edit
 *
 * \retval [1, if disable was successful, 0 if not]
 */
static uint8_t disable_channel_in_mask(uint8_t id, uint16_t *mask);

/*!
 * \brief Calculates the next datarate to set, when ADR is on or off
 *
 * \param [IN] adrEnabled Specify whether ADR is on or off
 *
 * \param [IN] updateChannelMask Set to true, if the channel masks shall be updated
 *
 * \param [OUT] datarateOut Reports the datarate which will be used next
 *
 * \retval Returns the state of ADR ack request
 */
static uint8_t adr_next_dr(uint8_t adr_enabled, uint8_t update_channel_mask, int8_t *datarate_out);

/*!
 * \brief Validates if the payload fits into the frame, taking the datarate
 *        into account.
 *
 * \details Refer to chapter 4.3.2 of the LoRaWAN specification, v1.0
 *
 * \param [IN] lenN Length of the application payload. The length depends on the
 *             datarate and is region specific
 *
 * \param [IN] Current datarate
 *
 * \param [IN] Length of the fOpts field
 *
 * \retval [0: payload does not fit into the frame, 1: payload fits into
 *          the frame]
 */

/*!
 * \brief Counts the number of bits in a mask.
 *
 * \param [IN] mask A mask from which the function counts the active bits.
 * \param [IN] nb_bits The number of bits to check.
 *
 * \retval Number of enabled bits in the mask.
 */
static uint8_t count_bits(uint16_t mask, uint8_t nb_bits);

/*!
 * \brief Function executed on first Rx window timer event
 */
static void on_rx_window1_timer_event(osal_job_t *j);

/*!
 * \brief Function executed on second Rx window timer event
 */
static void on_rx_window2_timer_event(osal_job_t *j);

/*!
 * \brief Function executed to start the reception of one packet
 */
static void start_rx_single(osal_job_t *j);

/*!
 * \brief Function executed to start the reception
 */
static void start_rx_continuous(osal_job_t *j);

/*!
 * \brief Function executed to delayed transmission
 */
static void on_tx_delayed_timer_event(osal_job_t *j);

/*!
 * \brief Function executed to disable low power mode
 */
static void lock_low_power_event(osal_job_t *j);

/*!
 * \brief Function executed to call the tx done callback
 */
static void tx_done_cb_event(osal_job_t *j);

/*!
 * \brief Function executed to call the mac state callback
 */
static void mac_state_cb_event(osal_job_t *j);

/*!
 * \brief Initializes and opens the reception window
 *
 * \param [IN] freq window channel frequency
 * \param [IN] datarate window channel datarate
 * \param [IN] bandwidth window channel bandwidth
 * \param [IN] 0: reception of one packet, 1: continuous reception
 * \param [IN] timestamp to start the reception
 */
static void rx_window_setup(uint32_t freq, int8_t datarate, enum osal_bw_t bandwidth, uint8_t rx_continuous, osal_time_t start_rx_delay);

/*!
 * \brief Searches and set the next random available channel
 *
 * \param [OUT] Time to wait for the next transmission according to the duty
 *              cycle.
 *
 * \retval status  Function status [1: OK, 0: Unable to find a channel on the
 *                                  current datarate]
 */
static uint8_t set_next_channel(osal_time_t *time);

/*!
 * \brief Function to reset the MAC
 */
static void reset_mac_parameters(void);

static mac_status_t loramac_send_when_possible_confirmed(uint8_t port, uint8_t *payload, uint8_t payload_len, uint8_t nb_retries,
							 downlink_mode_t dl_mode);

/*!
 * Redefinition of rand() and srand() standard C functions.
 * These functions are redefined in order to get the same behavior across
 * different compiler toolchains implementations.
 * It comes from Semtech git and it could be much faster than the
 * rand based on the RSSI.
 */
// Standard random functions redefinition start
#define RAND_LOCAL_MAX 2147483647L

static uint32_t next = 1;

int32_t rand1(void)
{
	return ((next = next * 1103515245L + 12345L) % RAND_LOCAL_MAX);
}

void srand1(uint32_t seed)
{
	next = seed;
}
// Standard random functions redefinition end

int32_t randr(int32_t min, int32_t max)
{
	return (int32_t) rand1() % (max - min + 1) + min;
}

static const uint32_t bw_table[] = {6250, 12500, 25000, 50000};

// Time in us
static uint32_t get_time_symbol(int8_t datarate)
{
	uint32_t t_sym;
	int32_t sf_value = datarate_table[datarate] + 6 - OSAL_SF6;

	t_sym = (1UL << sf_value) * 100000UL / bw_table[radio_params.bw];
	return t_sym;
}

static unsigned int get_time_on_air(int32_t size)
{
	int32_t val;
	int32_t div;
	uint32_t t_sym;
	uint32_t t_on_air;
	int32_t sf_value = datarate_table[loramac_params.channels_datarate] + 6 - OSAL_SF6;
	int32_t cr_value = RX_TX_CR + 1 - OSAL_CR4_5;

	val = 2L * ((int32_t) size) - sf_value + 7L + (RX_TX_CRC == OSAL_CRC_ON ? 4L : 0) - (0 ? 5L : 0);
	div = sf_value - 2L * ((int32_t) (sf_value >= 10));
	val += (div - 1); // ceiling
	val = val / div * (cr_value + 4);

	if (val < 0) {
		val = 0;
	}

	val += 8;

	t_sym = get_time_symbol(loramac_params.channels_datarate);

	t_on_air = t_sym * ((PREAMBLE_LENGTH + val) * 100UL + 425UL) / 100UL;
	// osal_printf("LEN: %d, SF: %d, T: %d\n", size,sf,t_on_air);

	return t_on_air; // in us
}

static inline uint8_t value_in_range(int8_t value, int8_t min, int8_t max)
{
	return (value >= min) && (value <= max);
}

void lora_mac_set_adr(uint8_t enable)
{
	adr_ctrl_on = enable;
}

void lora_mac_set_channels_tx_power(int8_t tx_power)
{
	loramac_params.channels_tx_power = tx_power;
}

void lora_mac_set_channels_datarate(int8_t datarate)
{
	loramac_params.channels_datarate = datarate;
}

void lora_mac_set_repeater_support(uint8_t enable)
{
	repeater_support = enable;
}

uint8_t lora_mac_get_last_demod_margin(void)
{
	return last_demod_margin;
}

uint8_t lora_mac_get_last_nb_gateways(void)
{
	return last_nb_gateways;
}

static void mac_lowpower_sleep_lock()
{
	/* Be sure to lock only once from the MAC point of view */
	if (!lowpower_sleep_locked) {
		osal_lowpower_sleep_lock();
		lowpower_sleep_locked = 1;
	}
}

static void mac_lowpower_sleep_unlock()
{
	/* Be sure to unlock only once from the MAC point of view */
	if (lowpower_sleep_locked) {
		osal_lowpower_sleep_unlock();
		lowpower_sleep_locked = 0;
	}
}

static uint8_t disable_channel_in_mask(uint8_t id, uint16_t *mask)
{
	uint8_t index = 0;
	index = id / 16;

	if ((index > 4) || (id >= LORA_MAX_NB_CHANNELS)) {
		return 0;
	}

	// Desactivate channel
	mask[index] &= ~(1 << (id % 16));

	return 1;
}

static uint8_t adr_next_dr(uint8_t adr_enabled, uint8_t update_channel_mask, int8_t *datarate_out)
{
	uint8_t adr_ack_req = 0;
	int8_t datarate = loramac_params.channels_datarate;

	if (adr_enabled == 1) {
		if (datarate == LORAMAC_TX_MIN_DATARATE) {
			adr_ack_counter = 0;
			adr_ack_req = 0;
		} else {
			if (adr_ack_counter >= ADR_ACK_LIMIT) {
				adr_ack_req = 1;
			} else {
				adr_ack_req = 0;
			}
			if (adr_ack_counter >= (ADR_ACK_LIMIT + ADR_ACK_DELAY)) {
				if ((adr_ack_counter % ADR_ACK_DELAY) == 0) {
					if (datarate > LORAMAC_TX_MIN_DATARATE) {
						datarate--;
					}
					if (datarate == LORAMAC_TX_MIN_DATARATE) {
						if (update_channel_mask == 1) {
							// Re-enable default channels LC1, LC2, LC3
							loramac_params.channels_mask[0] = loramac_params.channels_mask[0] | (LC(1) + LC(2) + LC(3));
						}
					}
				}
			}
		}
	}

	*datarate_out = datarate;

	return adr_ack_req;
}


static uint8_t count_bits(uint16_t mask, uint8_t nb_bits)
{
	uint8_t nb_active_bits = 0;
	uint8_t j = 0;

	for (j = 0; j < nb_bits; j++) {
		if ((mask & (1 << j)) == (1 << j)) {
			nb_active_bits++;
		}
	}
	return nb_active_bits;
}

static uint8_t check_frequency(uint32_t freq, uint8_t *band) {
	// Validate the frequency
	if (freq > 0) {
		if ((freq >= 863000000) && (freq <= 865000000)) {
			*band = BAND_G1_2;
		} else if ((freq > 865000000) && (freq <= 86800000)) {
			*band = BAND_G1_0;
		} else if ((freq > 868000000) && (freq <= 868600000)) {
			*band = BAND_G1_1;
		} else if ((freq >= 868700000) && (freq <= 869200000)) {
			*band = BAND_G1_2;
		} else if ((freq >= 869400000) && (freq <= 869650000)) {
			*band = BAND_G1_3;
		} else if ((freq >= 869700000) && (freq <= 870000000)) {
			*band = BAND_G1_4;
		} else {
			return 0;	// Invalid frequency
		}
		return 1;
	} else {
		return 0;		// Invalid frequency
	}
}

loramac_status_t loramac_channel_add(uint8_t id, channel_params_t params)
{
	uint8_t is_datarate_valid = 1;
	uint8_t is_frequency_valid = 1;
	uint8_t band = 0;

	// The id must not exceed LORA_MAX_NB_CHANNELS
	if (id >= LORA_MAX_NB_CHANNELS) {
		return LORAMAC_STATUS_PARAMETER_INVALID;
	}
	// Validate if the MAC is in a correct state
	if (loramac_state == LORAMAC_STATE_SENDING_DATA) {
		if (allow_change_channel == 0) {
			return LORAMAC_STATUS_BUSY;
		}
	}
	// Validate the datarate
	if ((params.dr_range.bits.min > params.dr_range.bits.max) ||
	    (value_in_range(params.dr_range.bits.min, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE) == 0) ||
	    (value_in_range(params.dr_range.bits.max, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE) == 0)) {
		is_datarate_valid = 0;
	}

	if (id < 3) {
		if (params.uplink_frequency != channels[id].uplink_frequency) {
			is_frequency_valid = 0;
		}
		if (params.dr_range.bits.min > loramac_params_defaults.channels_datarate) {
			is_datarate_valid = 0;
		}
		if (value_in_range(params.dr_range.bits.max, DR_5, LORAMAC_TX_MAX_DATARATE) == 0) {
			is_datarate_valid = 0;
		}
	}

	if (is_frequency_valid == 1) {
		is_frequency_valid = check_frequency(params.uplink_frequency, &band);
	}

	if ((is_datarate_valid == 0) && (is_frequency_valid == 0)) {
		return LORAMAC_STATUS_FREQ_AND_DR_INVALID;
	}
	if (is_datarate_valid == 0) {
		return LORAMAC_STATUS_DATARATE_INVALID;
	}
	if (is_frequency_valid == 0) {
		return LORAMAC_STATUS_FREQUENCY_INVALID;
	}

	// Every parameter is valid, activate the channel
	channels[id] = params;
	channels[id].band = band;
	loramac_params.channels_mask[0] |= (1 << id);

	return LORAMAC_STATUS_OK;
}

loramac_status_t loramac_channel_remove(uint8_t id)
{
	if (loramac_state == LORAMAC_STATE_SENDING_DATA) {
		if (allow_change_channel == 0) {
			return LORAMAC_STATUS_BUSY;
		}
	}

	if ((id < 3) || (id >= LORA_MAX_NB_CHANNELS)) {
		return LORAMAC_STATUS_PARAMETER_INVALID;
	} else {
		// Remove the channel from the list of channels
		channels[id] = (channel_params_t) {0, 0, {0}, 0};

		// Disable the channel as it doesn't exist anymore
		if (disable_channel_in_mask(id, loramac_params.channels_mask) == 0) {
			return LORAMAC_STATUS_PARAMETER_INVALID;
		}
	}
	return LORAMAC_STATUS_OK;
}

loramac_status_t loramac_multicast_channel_link(multicast_params_t *channel_param)
{
	if (channel_param == NULL) {
		return LORAMAC_STATUS_PARAMETER_INVALID;
	}
	if (loramac_state == LORAMAC_STATE_SENDING_DATA) {
		return LORAMAC_STATUS_BUSY;
	}

	// Reset downlink counter
	channel_param->downlink_counter = 0;

	if (multicast_channels == NULL) {
		// New node is the fist element
		multicast_channels = channel_param;
	} else {
		multicast_params_t *cur = multicast_channels;

		// Search the last node in the list
		while (cur->next != NULL) {
			cur = cur->next;
		}
		// This function always finds the last node
		cur->next = channel_param;
	}
	return LORAMAC_STATUS_OK;
}

loramac_status_t loramac_multicast_channel_unlink(multicast_params_t *channel_param)
{
	if (channel_param == NULL) {
		return LORAMAC_STATUS_PARAMETER_INVALID;
	}
	if (loramac_state == LORAMAC_STATE_SENDING_DATA) {
		return LORAMAC_STATUS_BUSY;
	}

	if (multicast_channels != NULL) {
		if (multicast_channels == channel_param) {
			// First element
			multicast_channels = channel_param->next;
		} else {
			multicast_params_t *cur = multicast_channels;

			// Search the node in the list
			while (cur->next && cur->next != channel_param) {
				cur = cur->next;
			}
			// If we found the node, remove it
			if (cur->next) {
				cur->next = channel_param->next;
			}
		}
		channel_param->next = NULL;
	}
	return LORAMAC_STATUS_OK;
}

static uint8_t add_mac_command(uint8_t cmd, uint8_t p1, uint8_t p2)
{
	uint8_t status = 2;
	// The maximum buffer length must take MAC commands to re-send into account.
	uint8_t buf_len = LORA_MAC_COMMAND_MAX_LENGTH - mac_commands_buffer_to_repeat_index;

	switch (cmd) {
		case MOTE_MAC_LINK_CHECK_REQ:
			if (mac_commands_buffer_index < buf_len) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// No payload for this command
				status = 0;
			}
			break;
		case MOTE_MAC_LINK_ADR_ANS:
			if (mac_commands_buffer_index < (buf_len - 1)) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// Margin
				mac_commands_buffer[mac_commands_buffer_index++] = p1;
				status = 0;
			}
			break;
		case MOTE_MAC_DUTY_CYCLE_ANS:
			if (mac_commands_buffer_index < buf_len) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// No payload for this answer
				status = 0;
			}
			break;
		case MOTE_MAC_RX_PARAM_SETUP_ANS:
			if (mac_commands_buffer_index < (buf_len - 1)) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// Status: Datarate ACK, Channel ACK
				mac_commands_buffer[mac_commands_buffer_index++] = p1;
				status = 0;
			}
			break;
		case MOTE_MAC_DEV_STATUS_ANS:
			if (mac_commands_buffer_index < (buf_len - 2)) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// 1st byte Battery
				// 2nd byte Margin
				mac_commands_buffer[mac_commands_buffer_index++] = p1;
				mac_commands_buffer[mac_commands_buffer_index++] = p2;
				status = 0;
			}
			break;
		case MOTE_MAC_NEW_CHANNEL_ANS:
			if (mac_commands_buffer_index < (buf_len - 1)) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// Status: Datarate range OK, Channel frequency OK
				mac_commands_buffer[mac_commands_buffer_index++] = p1;
				status = 0;
			}
			break;
		case MOTE_MAC_RX_TIMING_SETUP_ANS:
			if (mac_commands_buffer_index < buf_len) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// No payload for this answer
				status = 0;
			}
			break;
		case MOTE_MAC_DL_CHANNEL_ANS:
			if (mac_commands_buffer_index < buf_len) {
				mac_commands_buffer[mac_commands_buffer_index++] = cmd;
				// Status: Uplink frequency exists, Channel frequency OK
				mac_commands_buffer[mac_commands_buffer_index++] = p1;
				status = 0;
			}
			break;
		default:
			return 1;
	}
	if (status == 0) {
		mac_commands_in_next_tx = 1;
	}
	return status;
}

static void process_mac_command(uint8_t *mac_cmd_buffer, uint8_t mac_cmd_buffer_len, int8_t snr)
{
	uint8_t mac_index = 0;
	uint8_t i = 0;
	uint16_t ch_mask;
	int8_t tx_power = 0;
	int8_t datarate = 0;
	uint8_t band = 0;
	uint8_t nb_rep = 0;
	uint8_t ch_mask_cntl = 0;
	uint16_t channels_mask[6] = {0, 0, 0, 0, 0, 0};
	int8_t dr_offset = 0;
	uint32_t freq = 0;
	uint8_t battery_level = BAT_LEVEL_NO_MEASURE;
	uint8_t status = 0;
	int8_t channel_index = 0;
	channel_params_t chParam;
	uint8_t delay = 0;
	uint8_t nb_adr_req = 0;

	while (mac_index < mac_cmd_buffer_len) {
		// Decode Frame MAC commands
		switch (mac_cmd_buffer[mac_index++]) {
			case SRV_MAC_LINK_CHECK_ANS:
				network_check_request = 0;
				if (message_callbacks != NULL && message_callbacks->mac_network_state) {
					mac_network_info.status = NETWORK_AVAILABLE;
					last_demod_margin = mac_cmd_buffer[mac_index++];
					last_nb_gateways = mac_cmd_buffer[mac_index++];
					/* Send info to the application layer */
					message_callbacks->mac_network_state(&mac_network_info);
				} else {
					osal_printf("MAC: no callback set for network\n");
				}
				break;
			case SRV_MAC_LINK_ADR_REQ:
				// Initialize local copy of the channels mask array
				for (i = 0; i < 6; i++) {
					channels_mask[i] = loramac_params.channels_mask[i];
				}

				while (mac_index < mac_cmd_buffer_len) {
					status = 0x07;
					nb_adr_req++;

					datarate = mac_cmd_buffer[mac_index++];
					tx_power = datarate & 0x0F;
					datarate = (datarate >> 4) & 0x0F;

					ch_mask = (uint16_t) mac_cmd_buffer[mac_index++];
					ch_mask |= (uint16_t) mac_cmd_buffer[mac_index++] << 8;

					nb_rep = mac_cmd_buffer[mac_index++];
					ch_mask_cntl = (nb_rep >> 4) & 0x07;
					nb_rep &= 0x0F;

					if (nb_rep == 0) {
						nb_rep = 1;
					}

					if ((ch_mask_cntl == 0) && (ch_mask == 0)) {
						status &= 0xFE; // Channel mask KO
					} else if (((ch_mask_cntl >= 1) && (ch_mask_cntl <= 5)) || (ch_mask_cntl >= 7)) {
						// RFU
						status &= 0xFE; // Channel mask KO
					} else {
						for (i = 0; i < LORA_MAX_NB_CHANNELS; i++) {
							if (ch_mask_cntl == 6) {
								if (channels[i].uplink_frequency != 0) {
									ch_mask |= 1 << i;
								}
							} else {
								if (((ch_mask & (1 << i)) != 0) && (channels[i].uplink_frequency == 0)) {
									// Trying to enable an undefined channel
									status &= 0xFE; // Channel mask KO
								}
							}
						}
						channels_mask[0] = ch_mask;
					}

					if (mac_cmd_buffer[mac_index] != SRV_MAC_LINK_ADR_REQ) {
						break;
					} else {
						mac_index++;
					}
				}

				if ((adr_ctrl_on == 0) &&
				    ((loramac_params.channels_datarate != datarate) || (loramac_params.channels_tx_power != tx_power))) {
					// ADR disabled don't handle ADR requests if server tries to change datarate or txpower
					// Answer the server with fail status
					// Power ACK     = 0
					// Data rate ACK = 0
					// Channel mask  = 0
					add_mac_command(MOTE_MAC_LINK_ADR_ANS, 0, 0);
					mac_index += 3; // Skip over the remaining bytes of the request
					break;
				}

				if (value_in_range(datarate, LORAMAC_TX_MIN_DATARATE, LORAMAC_TX_MAX_DATARATE) == 0) {
					status &= 0xFD; // Datarate KO
				}

				if (value_in_range(tx_power, LORAMAC_MAX_TX_POWER, LORAMAC_MIN_TX_POWER) == 0) {
					status &= 0xFB; // TxPower KO
				}

				if ((status & 0x07) == 0x07) {
					loramac_params.channels_datarate = datarate;
					loramac_params.channels_tx_power = tx_power;

					loramac_params.channels_mask[0] = channels_mask[0];
					loramac_params.channels_mask[1] = channels_mask[1];
					loramac_params.channels_mask[2] = channels_mask[2];
					loramac_params.channels_mask[3] = channels_mask[3];
					loramac_params.channels_mask[4] = channels_mask[4];
					loramac_params.channels_mask[5] = channels_mask[5];

					loramac_params.channels_nb_rep = nb_rep;
				}

				for (i = 0; i < nb_adr_req; i++) {
					add_mac_command(MOTE_MAC_LINK_ADR_ANS, status, 0);
				}
				break;
			case SRV_MAC_DUTY_CYCLE_REQ:
				max_duty_cycle = mac_cmd_buffer[mac_index++];
				aggregated_duty_cycle = 1 << max_duty_cycle;
				add_mac_command(MOTE_MAC_DUTY_CYCLE_ANS, 0, 0);
				break;
			case SRV_MAC_RX_PARAM_SETUP_REQ:
				status = 0x07;
				datarate = 0;

				dr_offset = (mac_cmd_buffer[mac_index] >> 4) & 0x07;
				datarate = mac_cmd_buffer[mac_index] & 0x0F;
				mac_index++;

				freq = (uint32_t) mac_cmd_buffer[mac_index++];
				freq |= (uint32_t) mac_cmd_buffer[mac_index++] << 8;
				freq |= (uint32_t) mac_cmd_buffer[mac_index++] << 16;
				freq *= 100;

				// Check if the frequency is valid
				if (check_frequency(freq, &band) == 0) {		// band is not used
					status &= 0xFE;
				}

				if (value_in_range(datarate, LORAMAC_RX_MIN_DATARATE, LORAMAC_RX_MAX_DATARATE) == 0) {
					status &= 0xFD; // Datarate KO
				}

				if (value_in_range(dr_offset, LORAMAC_MIN_RX1_DR_OFFSET, LORAMAC_MAX_RX1_DR_OFFSET) == 0) {
					status &= 0xFB; // Rx1DrOffset range KO
				}

				if ((status & 0x07) == 0x07) {
					loramac_params.rx2_channel.datarate = datarate;
					loramac_params.rx2_channel.frequency = freq;
					loramac_params.rx1_dr_offset = dr_offset;
				}
				add_mac_command(MOTE_MAC_RX_PARAM_SETUP_ANS, status, 0);
				break;
			case SRV_MAC_DEV_STATUS_REQ:
				battery_level = BAT_LEVEL_NO_MEASURE;
				if ((battery_callback != NULL) && (battery_callback->get_battery_level != NULL)) {
					battery_level = battery_callback->get_battery_level();
				}
				add_mac_command(MOTE_MAC_DEV_STATUS_ANS, battery_level, snr);
				break;
			case SRV_MAC_NEW_CHANNEL_REQ:
				status = 0x03;

				channel_index = mac_cmd_buffer[mac_index++];
				chParam.uplink_frequency = (uint32_t) mac_cmd_buffer[mac_index++];
				chParam.uplink_frequency |= (uint32_t) mac_cmd_buffer[mac_index++] << 8;
				chParam.uplink_frequency |= (uint32_t) mac_cmd_buffer[mac_index++] << 16;
				chParam.uplink_frequency *= 100;
				chParam.rx1_downlink_frequency = 0;
				chParam.dr_range.value = mac_cmd_buffer[mac_index++];

				allow_change_channel = 1;
				if (chParam.uplink_frequency == 0) {
					if (channel_index < 3) {
						status &= 0xFC;
					} else {
						if (loramac_channel_remove(channel_index) != LORAMAC_STATUS_OK) {
							status &= 0xFC;
						}
					}
				} else {
					switch (loramac_channel_add(channel_index, chParam)) {
						case LORAMAC_STATUS_OK:
							break;
						case LORAMAC_STATUS_FREQUENCY_INVALID:
							status &= 0xFE;
							break;
						case LORAMAC_STATUS_DATARATE_INVALID:
							status &= 0xFD;
							break;
						case LORAMAC_STATUS_FREQ_AND_DR_INVALID:
							status &= 0xFC;
							break;
						default:
							status &= 0xFC;
							break;
					}
				}
				allow_change_channel = 0;

				add_mac_command(MOTE_MAC_NEW_CHANNEL_ANS, status, 0);
				break;
			case SRV_MAC_RX_TIMING_SETUP_REQ:
				delay = mac_cmd_buffer[mac_index++] & 0x0F;

				if (delay == 0) {
					delay++;
				}
				loramac_params.receive_delay_1 = delay * 1000;				// to milliseconds
				loramac_params.receive_delay_2 = loramac_params.receive_delay_1 + 1000; // 1 second after the first reception slot
				add_mac_command(MOTE_MAC_RX_TIMING_SETUP_ANS, 0, 0);
				break;
			case SRV_MAC_DL_CHANNEL_REQ:
				status = 0x03;

				channel_index = mac_cmd_buffer[mac_index++];
				chParam.rx1_downlink_frequency = (uint32_t) mac_cmd_buffer[mac_index++];
				chParam.rx1_downlink_frequency |= (uint32_t) mac_cmd_buffer[mac_index++] << 8;
				chParam.rx1_downlink_frequency |= (uint32_t) mac_cmd_buffer[mac_index++] << 16;
				chParam.rx1_downlink_frequency *= 100;

				allow_change_channel = 1;

				// Check if the frequency is valid
				if (check_frequency(chParam.rx1_downlink_frequency, &band) == 0) {		// band is not used
					status &= 0xFE;
				}

				// Check if the channel id exists
				if (channels[channel_index].uplink_frequency == 0) {
					status &= 0xFD;
				}

				if (status == 0x03) {
					channels[channel_index].rx1_downlink_frequency = chParam.rx1_downlink_frequency;
				}

				add_mac_command(MOTE_MAC_DL_CHANNEL_ANS, status, 0);
				break;
			default:
				// Unknown command. ABORT MAC commands processing
				return;
		}
	}
}

uint8_t parse_mac_commands_to_repeat(uint8_t *cmd_buf_in, uint8_t length, uint8_t *cmd_buf_out)
{
	uint8_t i = 0;
	uint8_t cmd_count = 0;

	if ((cmd_buf_in == NULL) || (cmd_buf_out == NULL)) {
		return 0;
	}

	for (i = 0; i < length; i++) {
		switch (cmd_buf_in[i]) {
			case MOTE_MAC_DL_CHANNEL_ANS:
			case MOTE_MAC_RX_PARAM_SETUP_ANS:
				cmd_buf_out[cmd_count++] = cmd_buf_in[i++];
				cmd_buf_out[cmd_count++] = cmd_buf_in[i];
				break;
			case MOTE_MAC_RX_TIMING_SETUP_ANS:
				cmd_buf_out[cmd_count++] = cmd_buf_in[i];
				break;
			case MOTE_MAC_DEV_STATUS_ANS:
				i += 2;
				break;
			case MOTE_MAC_LINK_ADR_ANS:
			case MOTE_MAC_NEW_CHANNEL_ANS:
				i++;
				break;
			case MOTE_MAC_TX_PARAM_SETUP_ANS:
			case MOTE_MAC_DUTY_CYCLE_ANS:
			case MOTE_MAC_LINK_CHECK_REQ:
				break;
			default:
				break;
		}
	}
	return cmd_count;
}

static void on_tx_delayed_timer_event(osal_job_t *j)
{
	if (message_tx_rx.type == MSG_TYPE_JOIN_REQ) {
		// Only for request frame
		osal_printf("Trying to JOIN network (OTAA) : retry\n");

		/* In case of a join request retransmission, the stack must prepare
		 * the frame again, because the network server keeps track of the random
		 * LoRaMacDevNonce values to prevent reply attacks. */
		message_compose_join_OTA(MSG_TYPE_JOIN_REQ, dev_deveui, dev_appeui, &loramac_dev_nonce, dev_appkey, &message_tx_rx);
	}

	schedule_tx();
}

static void on_ack_timeout_timer_event(osal_job_t *j)
{
	if (open_receive_window == 1) {
		if (message_ack_request(&message_tx_rx) == WITH_ACK) {
			if ((ack_timeout_retries_counter < ack_timeout_retries) && (ack_timeout_retries_counter <= MAX_ACK_RETRIES)) {
				ack_timeout_retries_counter++;

				if ((ack_timeout_retries_counter % 2) == 1) {
					loramac_params.channels_datarate = MAX(loramac_params.channels_datarate - 1, LORAMAC_TX_MIN_DATARATE);
				}
				// Sends the same frame again
				schedule_tx();
			} else {
				// Re-enable default channels LC1, LC2, LC3
				loramac_params.channels_mask[0] = loramac_params.channels_mask[0] | (LC(1) + LC(2) + LC(3));

				loramac_state = LORAMAC_STATE_IDLE;
				mac_state = MAC_STATE_IDLE;
				uplink_counter++;
				if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->save_uplink_counter != NULL)) {
					flash_callback->save_uplink_counter(uplink_counter);
				}
			}
		}
	} else {
		ack_timeout_reached = 1;
	}
}

static void lock_low_power_event(osal_job_t *j) {
	mac_lowpower_sleep_lock();
}

static void tx_done_cb_event(osal_job_t* j) {
	if (message_callbacks != NULL && message_callbacks->mac_tx_done) {
		message_callbacks->mac_tx_done(&mac_tx_info);
	} else {
		osal_printf("MAC: no callback set for tx\n");
	}
}

static void mac_state_cb_event(osal_job_t* j) {
	if (message_callbacks != NULL && message_callbacks->mac_state != NULL) {
		message_callbacks->mac_state(mac_state);
	} else {
		osal_printf("MAC: no callback set for tx\n");
	}
}

static void on_rx_window1_timer_event(osal_job_t *j)
{
	int8_t datarate = 0;
	enum osal_bw_t bandwidth = OSAL_BW125; // LoRa 125 kHz
	osal_time_t rxtime = 0;

	rx_slot = 0;

	if (loramac_device_class == CLASS_C) {
		open_receive_window = 1; // Allow to open RX1 because RX2 is open before in class C
	}

	rxtime = timestamp_tx_done + ms2us(rx_windows1_delay) + osal_get_additional_cpu_wakeup_latency() + MARGIN_TIME + RADIO_CONFIG_TIME;

	datarate = loramac_params.channels_datarate - loramac_params.rx1_dr_offset;

	if (datarate < 0) {
		datarate = DR_0;
	}

	if (datarate == DR_6) {
		// LoRa 250 kHz
		bandwidth = OSAL_BW250;
	}

	if (channels[current_channel].rx1_downlink_frequency != 0) {
		rx_window_setup(channels[current_channel].rx1_downlink_frequency, datarate, bandwidth, 0, rxtime);
	} else {
		rx_window_setup(channels[current_channel].uplink_frequency, datarate, bandwidth, 0, rxtime);
	}
}

static void on_rx_window2_timer_event(osal_job_t *j)
{
	enum osal_bw_t bandwidth = OSAL_BW125; // LoRa 125 kHz
	osal_time_t rxtime = 0;

	rx_slot = 1;

	rxtime = timestamp_tx_done + ms2us(rx_windows2_delay) + osal_get_additional_cpu_wakeup_latency() + MARGIN_TIME + RADIO_CONFIG_TIME;

	// For higher datarates, we increase the number of symbols generating a Rx Timeout
	if (loramac_params.rx2_channel.datarate == DR_6) {
		// LoRa 250 kHz
		bandwidth = OSAL_BW250;
	}

	if (loramac_device_class != CLASS_C) {
		rx_window_setup(loramac_params.rx2_channel.frequency, loramac_params.rx2_channel.datarate, bandwidth, 0, rxtime);
	} else {
		rx_window_setup(loramac_params.rx2_channel.frequency, loramac_params.rx2_channel.datarate, bandwidth, 1, rxtime);
	}
}

static void rx_window_setup(uint32_t freq, int8_t datarate, enum osal_bw_t bandwidth, uint8_t rx_continuous, osal_time_t start_rx_delay)
{
	enum osal_sf_t downlink_datarate = (enum osal_sf_t) datarate_table[datarate];
	uint32_t t_sym = 0;
	int32_t rx_window_offset = 0;

	if (open_receive_window == 1) {
		open_receive_window = 0;

		osal_radio(OSAL_RADIO_STOP);
		osal_clear_radio_callback();

		radio_params.freq = freq;
		radio_params.cr = RX_TX_CR;
		radio_params.sf = downlink_datarate;
		radio_params.bw = bandwidth;
		t_sym = get_time_symbol(datarate);
		//Calculating the number of symbols and rounding above
		radio_params.rxsyms = (uint32_t) ((((2 * loramac_params.min_rx_symbols - 8) * t_sym + 2 * 9800) + (t_sym - 1)) / t_sym);
		radio_params.rxsyms = MAX(radio_params.rxsyms, loramac_params.min_rx_symbols);
		radio_params.crc = RX_TX_CRC;
		radio_params.ih = 0;
		radio_params.sync_word = MAC_PREAMBLE;
		radio_params.preamble_len = PREAMBLE_LENGTH;
		radio_params.iq = OSAL_IQ_INVERTED;			//I/Q inverted for rx
		radio_params.ppm_offset = 0;
		//Calculating the offset of the receiving window and rounding above
		rx_window_offset = (int32_t) ((4.0 * t_sym) - ((radio_params.rxsyms * t_sym) / 2.0)
				 + (((4 * t_sym) - (radio_params.rxsyms * t_sym)) % 2 > 0));
		radio_params.rxtime = start_rx_delay + rx_window_offset;

		osal_set_radio_callback(internal_rxdone_cb);

		current_datarate = datarate;
		radio_params.payload_length = message_max_payload_length() + LORA_MAC_FRMPAYLOAD_OVERHEAD;

		osal_set_radio_params(&radio_params);

		osal_lowpower_sleep_lock();

		if (rx_continuous == 0) {
			osal_post_delayed_job(&start_rx_single_job, start_rx_delay - RADIO_CONFIG_TIME, start_rx_single);
		} else {
			osal_post_delayed_job(&start_rx_continuous_job, start_rx_delay - RADIO_CONFIG_TIME, start_rx_continuous);
		}
	}
}

static void start_rx_single(osal_job_t *j)
{
	osal_lowpower_sleep_unlock();
	if (rx_slot == 0) {
		loramac_state = LORAMAC_STATE_WAITING_DATA_RX1;
	} else {
		loramac_state = LORAMAC_STATE_WAITING_DATA_RX2;
	}
	osal_radio(OSAL_RADIO_START_RX_SINGLE);
}

static void start_rx_continuous(osal_job_t *j)
{
	osal_lowpower_sleep_unlock();
	loramac_state = LORAMAC_STATE_DOWNLINK_IDLE;
	osal_radio(OSAL_RADIO_START_RX);
}

static uint8_t set_next_channel(osal_time_t *time)
{
	uint8_t nb_enabled_channels = 0;
	uint8_t delay_tx = 0;
	uint8_t enabled_channels[LORA_MAX_NB_CHANNELS];
	osal_time_t next_tx_delay = OSAL_TIME_MAX;
	uint8_t i = 0, j = 0, k = 0;

	osal_memset(enabled_channels, 0, LORA_MAX_NB_CHANNELS);

	if (count_bits(loramac_params.channels_mask[0], 16) == 0) {
		// Re-enable default channels, if no channel is enabled
		loramac_params.channels_mask[0] = loramac_params.channels_mask[0] | (LC(1) + LC(2) + LC(3));
	}

	// Update Aggregated duty cycle
	if (aggregated_time_off <= (osal_get_time() - aggregated_last_tx_done_time)) { // aggregated_last_tx_done_time -> update in radio tx done
		aggregated_time_off = 0;

		// Update bands Time OFF
		for (i = 0; i < LORA_MAX_NB_BANDS; i++) {
			if (duty_cycle_on == 1) {
				if (bands[i].time_off <= (osal_get_time() - bands[i].last_tx_done_time)) {
					bands[i].time_off = 0;
				}
				if (bands[i].time_off != 0) {
					next_tx_delay = MIN(bands[i].time_off - (osal_get_time() - bands[i].last_tx_done_time), next_tx_delay);
				}
			} else {
				next_tx_delay = 0;
				bands[i].time_off = 0;
			}
		}

		// Search how many channels are enabled
		for (i = 0, k = 0; i < LORA_MAX_NB_CHANNELS; i += 16, k++) {
			for (j = 0; j < 16; j++) {
				if ((loramac_params.channels_mask[k] & (1 << j)) != 0) {
					if (channels[i + j].uplink_frequency == 0) {
						// Check if the channel is enabled
						continue;
					}
					if (is_loramac_network_joined == 0) {
						if ((JOIN_CHANNELS & (1 << j)) == 0) {
							continue;
						}
					}
					if (((channels[i + j].dr_range.bits.min <= loramac_params.channels_datarate) &&
					     (loramac_params.channels_datarate <= channels[i + j].dr_range.bits.max)) == 0) {
						// Check if the current channel selection supports the given datarate
						continue;
					}
					if (bands[channels[i + j].band].time_off > 0) {
						// Check if the band is available for transmission
						delay_tx++;
						continue;
					}
					enabled_channels[nb_enabled_channels++] = i + j;
				}
			}
		}
	} else {
		delay_tx++;
		next_tx_delay = aggregated_time_off - (osal_get_time() - aggregated_last_tx_done_time);
	}

	if (nb_enabled_channels > 0) {
		current_channel = enabled_channels[randr(0, nb_enabled_channels - 1)];
		*time = 0;
		return 1;
	} else {
		if (delay_tx > 0) {
			// Delay transmission due to AggregatedTimeOff or to a band time off
			*time = next_tx_delay;
			return 1;
		}
		// Datarate not supported by any channel
		*time = 0;
		return 0;
	}
}

static void _set_device_class(device_class_t device_class)
{
		switch (device_class) {
		case CLASS_A:
#ifdef CFG_SUPPORT_CLASS_C
		case CLASS_C:
#endif
			break;

		default:
			/* Unsupported class */
			return;
	}

	if (is_loramac_network_joined) {
		if ((loramac_state != LORAMAC_STATE_IDLE) && (loramac_state != LORAMAC_STATE_DOWNLINK_IDLE)) {
			/* Cannot change the class while the MAC is busy */
			return;
		}

		/* Stop current class */
		switch (loramac_device_class) {
			case CLASS_A:
				break;

#ifdef CFG_SUPPORT_CLASS_C
			case CLASS_C:
				osal_radio(OSAL_RADIO_STOP);
				osal_clear_radio_callback();
				break;
#endif

			default:
				break;
		}

		/* Start new class */
		switch (device_class) {
			case CLASS_A:
				break;

#ifdef CFG_SUPPORT_CLASS_C
			case CLASS_C:
				osal_post_job(&rx2_job, on_rx_window2_timer_event);
				break;
#endif

			default:
				break;
		}
	}

	loramac_device_class = device_class;
}

static void prepare_rx_done_abort(void)
{
	if (message_ack_request(&message_tx_rx) == WITH_ACK) {
		osal_post_job(&ack_timeout_job, on_ack_timeout_timer_event);
	} else {
		loramac_state = LORAMAC_STATE_IDLE;
		mac_state = MAC_STATE_IDLE;
	}
	if ((rx_slot == 0) && (loramac_device_class == CLASS_C)) {
		osal_post_job(&rx2_job, on_rx_window2_timer_event);
	} else {
		loramac_state = LORAMAC_STATE_IDLE;
		mac_state = MAC_STATE_IDLE;
	}
}

static void internal_rxdone_cb(void)
{
	message_t msg;
	uint8_t i = 0, k = 0;
	uint8_t multicast = 0;
	uint16_t sequence_counter = 0;
	uint16_t sequence_counter_prev = 0;
	uint16_t sequence_counter_diff = 0;
	uint32_t downlink_counter_temp = 0;
	uint32_t downlink_counter_overflow = 0;
	multicast_params_t *current_multicast_params = NULL;
	uint8_t cf_list[16] = {0};
	channel_params_t param;
	uint8_t rx_delay = 0;
	uint8_t is_same_counters = 0;
	message_status_t message_state;
	osal_radio_status_t status;

	open_receive_window = 1; // radio ready to be used

	msg.valid = 0;

	osal_radio(OSAL_RADIO_STOP);
	osal_clear_radio_callback();
	osal_get_radio_status(&status);

	/* Initializing variables before calling callbacks */
	mac_rx_info.payload_len = 0;
	mac_rx_info.snr = status.snr;
	mac_rx_info.rssi = status.rssi;
	mac_tx_info.status = MAC_INFO_STATUS_TX_ACK_TIMEOUT;

	/* We received something, check if it is a valid message for us */
	if (status.crc_ok && osal_get_radio_buffer_length() > 0) {
		osal_cancel_job(&rx2_job); // if RX1 done only (not timeout)

		message_get_header(osal_get_radio_buffer(), osal_get_radio_buffer_length(), &msg);
		switch (msg.type) {
			case MSG_TYPE_JOIN_ACCEPT:
				message_state = message_decode_join_OTA(osal_get_radio_buffer(), osal_get_radio_buffer_length(), dev_appkey,
									dev_appskey, dev_nwkskey, &loramac_dev_nonce, &mac_netid, &mac_devaddr,
									&loramac_params.rx1_dr_offset, &loramac_params.rx2_channel.datarate,
									&rx_delay, cf_list);
				if (message_state < 0) {
					prepare_rx_done_abort();
					mac_rx_info.status = MAC_INFO_STATUS_DOWNLINK_DECODING_ERROR;
					goto end_function;
				}

				loramac_init_activation_personalization(mac_netid, mac_devaddr, dev_nwkskey, dev_appskey);
				is_loramac_network_ABP = 0;

				if (rx_delay == 0) {
					loramac_params.receive_delay_1 = 1000;
				} else {
					loramac_params.receive_delay_1 = rx_delay * 1000;
				}
				loramac_params.receive_delay_2 = loramac_params.receive_delay_1 + 1000;

				// CFList
				if ((osal_get_radio_buffer_length() - 1) > 16) {
					param.dr_range.value = (DR_5 << 4) | DR_0;

					allow_change_channel = 1;
					for (i = 3, k = 0; i < (5 + 3); i++, k += 3) {
						param.uplink_frequency = ((uint32_t) cf_list[k] | ((uint32_t) cf_list[k + 1] << 8) |
								   ((uint32_t) cf_list[k + 2] << 16)) *
								  100;
						loramac_channel_add(i, param);
					}
					allow_change_channel = 0;
				}

				loramac_params.channels_datarate = loramac_params_defaults.channels_datarate;

				msg.valid = 1;
				uplink_counter = 0;
				if ((flash_callback != NULL) && (flash_callback->save_uplink_counter != NULL)) {
					flash_callback->save_uplink_counter(uplink_counter);
				}
				downlink_counter = 0;
				if ((flash_callback != NULL) && (flash_callback->save_downlink_counter != NULL)) {
					flash_callback->save_downlink_counter(downlink_counter);
				}

				_set_device_class(loramac_device_class);

				loramac_state = LORAMAC_STATE_IDLE;
				mac_state = MAC_STATE_IDLE;
				mac_rx_info.status = MAC_INFO_STATUS_JOIN_ACCEPT_OK;

				break;

			case MSG_TYPE_DATA_CONFIRMED_DOWN:
			case MSG_TYPE_DATA_UNCONFIRMED_DOWN:
				message_get_address(osal_get_radio_buffer(), osal_get_radio_buffer_length(), &msg);
				if (msg.devaddr != mac_devaddr) {
					// Test multicast channels
					current_multicast_params = multicast_channels;
					while (current_multicast_params != NULL) {
						if (msg.devaddr == current_multicast_params->address) {
							multicast = 1;
							message_init_session_keys(current_multicast_params->nwkskey,
										  current_multicast_params->appskey);
							downlink_counter_temp = current_multicast_params->downlink_counter;
							break;
						}
						current_multicast_params = current_multicast_params->next;
					}
					if (multicast == 0) {
						// We are not the destination of this frame.
						osal_printf("MAC: valid msg received but not for me\n");
						prepare_rx_done_abort();
						mac_rx_info.status = MAC_INFO_STATUS_RX_ERROR;
						goto end_function;
					}
				} else {
					multicast = 0;
					message_init_session_keys(dev_nwkskey, dev_appskey);
					downlink_counter_temp = downlink_counter;
				}

				message_state = message_decode(osal_get_radio_buffer(), osal_get_radio_buffer_length(), &msg);
				if (message_state < 0) {
					prepare_rx_done_abort();
					mac_rx_info.status = MAC_INFO_STATUS_DOWNLINK_DECODING_ERROR;
					goto end_function;
				}

				osal_printf("raw %d, %d, ", msg.msg_len, msg.fopts_len);
				for (i = 0; i < msg.msg_len; i++) {
					osal_printf("%02X ", msg.raw[i]);
				}
				osal_printf("\n");

				if (msg.valid != 0) {
					osal_printf("MSG valid\n");

					sequence_counter = msg.seqno;
					sequence_counter_prev = (uint16_t) downlink_counter_temp;
					sequence_counter_diff = sequence_counter - sequence_counter_prev;

					if (sequence_counter_diff < (1 << 15)) {
						downlink_counter_temp += sequence_counter_diff;
						if (sequence_counter == downlink_counter_temp) {
							is_same_counters = 1;
						}
					} else {
						// check for sequence roll-over
						downlink_counter_overflow = downlink_counter_temp + 0x10000 + (int16_t) sequence_counter_diff;
						if (sequence_counter == downlink_counter_overflow) {
							is_same_counters = 1;
							downlink_counter_temp = downlink_counter_overflow;
						}
					}
					// Check for a the maximum allowed counter difference
					if (sequence_counter_diff >= MAX_FCNT_GAP) {
						osal_printf("Too many frames loss\n");
						mac_rx_info.status = MAC_INFO_STATUS_RX_ERROR;
						prepare_rx_done_abort();
						goto end_function;
					}

					if (is_same_counters == 1) {
						adr_ack_counter = 0;
						mac_commands_buffer_to_repeat_index = 0;
						if (multicast == 1) {
							if ((current_multicast_params->downlink_counter == downlink_counter_temp) && (current_multicast_params->downlink_counter != 0)) {
								mac_rx_info.status = MAC_INFO_STATUS_RX_ERROR;
								prepare_rx_done_abort();
								goto end_function;
							}
							current_multicast_params->downlink_counter = downlink_counter_temp;
						} else {
							if (msg.type != MSG_TYPE_DATA_CONFIRMED_DOWN) {
								if ((downlink_counter == downlink_counter_temp) && (downlink_counter != 0)) {
									mac_rx_info.status = MAC_INFO_STATUS_RX_ERROR;
									prepare_rx_done_abort();
									goto end_function;
								}
							}
							downlink_counter = downlink_counter_temp;
							if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->save_downlink_counter != NULL)) {
								flash_callback->save_downlink_counter(downlink_counter);
							}
						}
						mac_rx_info.status = MAC_INFO_STATUS_OK;
						if ((msg.port == MAC_PORT_MAC_COMMAND) || (msg.fopts_len != 0)) {
							osal_printf("MAC: mac command received\n");
							if (msg.port == MAC_PORT_MAC_COMMAND) {
								if (msg.fopts_len == 0) {
									process_mac_command(msg.payload, msg.payload_len, status.snr);
								} else {
									mac_rx_info.status = MAC_INFO_STATUS_DOWNLINK_DECODING_ERROR;
								}
							} else {
								process_mac_command(msg.fopts, msg.fopts_len, status.snr);
							}
						}
						if ((msg.payload_len > 0) && (msg.port != MAC_PORT_MAC_COMMAND)) {
							osal_printf("MAC: message received\n");
							mac_rx_info.snr = status.snr;
							mac_rx_info.rssi = status.rssi;
							mac_rx_info.port = msg.port;
							mac_rx_info.payload = msg.payload;
							mac_rx_info.payload_len = msg.payload_len;
						}
						if (message_ack_request(&message_tx_rx) == WITH_ACK) {
							if (msg.ack == 1) {
								// Confirm the reception of server acknowledge
								mac_tx_info.ack_status = ACK;
								mac_tx_info.status = MAC_INFO_STATUS_OK;
								loramac_state = LORAMAC_STATE_IDLE;
								mac_state = MAC_STATE_IDLE;
								// Stop the job for ack timeout
								osal_cancel_job(&ack_timeout_job);
								uplink_counter++;
								if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->save_uplink_counter != NULL)) {
									flash_callback->save_uplink_counter(uplink_counter);
								}
							} else {
								mac_tx_info.ack_status = NO_ACK;
								mac_tx_info.status = MAC_INFO_STATUS_TX_ACK_TIMEOUT;
								if (ack_timeout_reached == 1) {
									ack_timeout_reached = 0;
									loramac_state = LORAMAC_STATE_CHECKING_DUTY_CYCLE;
									mac_state = MAC_STATE_WAITING_RETRY;
									osal_post_job(&ack_timeout_job, on_ack_timeout_timer_event);
								} else {
									// If no more retries available, stop the job for ack timeout
									if (ack_timeout_retries_counter > ack_timeout_retries) {
										osal_cancel_job(&ack_timeout_job);
										loramac_state = LORAMAC_STATE_IDLE;
										mac_state = MAC_STATE_IDLE;
										uplink_counter++;
										if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->save_uplink_counter != NULL)) {
											flash_callback->save_uplink_counter(uplink_counter);
										}
									}
								}
							}
						} else {
							loramac_state = LORAMAC_STATE_IDLE;
							mac_state = MAC_STATE_IDLE;
						}
						/* For the next frame to send */
						if (msg.type == MSG_TYPE_DATA_CONFIRMED_DOWN) {
							message_tx_rx.ack = 1;
						} else {
							message_tx_rx.ack = 0;
						}
					} else {
						prepare_rx_done_abort();
						mac_rx_info.status = MAC_INFO_STATUS_MIC_FAILED;
						goto end_function;
					}
				}
				break;

			case MSG_TYPE_PROPRIETARY:
				// We don't count the first byte because is the MAC header.
				msg.payload_len = osal_get_radio_buffer_length() - 1;
				osal_memcpy((void *) msg.payload, (void *) (osal_get_radio_buffer() + 1), osal_get_radio_buffer_length() - 1);
				loramac_state = LORAMAC_STATE_IDLE;
				mac_state = MAC_STATE_IDLE;
				break;

			default:
				osal_printf("Wrong receive frame type !\n");
				prepare_rx_done_abort();
				break;
		}
	} else {
		if (osal_get_radio_buffer_length() > 0) {		// Wrong CRC
			mac_rx_info.status = MAC_INFO_STATUS_RX_ERROR;
		} else {						// No message received
			mac_rx_info.status = MAC_INFO_STATUS_DOWNLINK_TIMEOUT;
		}

		if (rx_slot == 1 && (message_ack_request(&message_tx_rx) == WITH_ACK)) {
			if (ack_timeout_retries_counter < ack_timeout_retries) {
				loramac_state = LORAMAC_STATE_CHECKING_DUTY_CYCLE;
				mac_state = MAC_STATE_WAITING_RETRY;
			} else {
				loramac_state = LORAMAC_STATE_IDLE;
				mac_state = MAC_STATE_IDLE;
			}
		}
	}

	if (osal_get_radio_buffer_length() > 0) {
		if (msg.valid == 0) {
			osal_printf("MAC: rx msg invalid\n");
			loramac_state = LORAMAC_STATE_IDLE;
			mac_state = MAC_STATE_IDLE;
			mac_rx_info.status = MAC_INFO_STATUS_DOWNLINK_DECODING_ERROR;
		}
	} else {
		if ((rx_slot == 1) && (message_ack_request(&message_tx_rx) == WITHOUT_ACK)) {
			// Retransmission unconfirmed uplink frame, to increase radio performance.
			if (channels_nb_rep_counter >= loramac_params.channels_nb_rep) {
				channels_nb_rep_counter = 0;

				adr_ack_counter++;
				// If no message receive from an unconfirmed frame, increment uplink counter
				uplink_counter++;
				if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->save_uplink_counter != NULL)) {
					flash_callback->save_uplink_counter(uplink_counter);
				}

				loramac_state = LORAMAC_STATE_IDLE;
				mac_state = MAC_STATE_IDLE;
			} else {
				// Sends the same frame again
				mac_state = MAC_STATE_WAITING_RETRY;
				osal_post_job(&tx_delayed_job, on_tx_delayed_timer_event);
			}
		}
	}

	if ((rx_slot == 0) && (loramac_device_class == CLASS_C)) {
		osal_post_job(&rx2_job, on_rx_window2_timer_event);
	}

end_function:
	if (message_callbacks != NULL && message_callbacks->mac_state != NULL) {
		message_callbacks->mac_state(mac_state);
	}
	if (message_ack_request(&message_tx_rx) == WITH_ACK && network_check_request == 0) {
		if ((mac_tx_info.status == MAC_INFO_STATUS_OK) || ((ack_timeout_retries_counter >= ack_timeout_retries) && (rx_slot == 1))) {
			if (message_callbacks != NULL && message_callbacks->mac_tx_done) {
				mac_tx_info.nb_retries = ack_timeout_retries_counter;
				/* Send the payload to the application layer */
				message_callbacks->mac_tx_done(&mac_tx_info);
			} else {
				osal_printf("MAC: no callback set for tx\n");
			}
		}
	}
	if (msg.port != MAC_PORT_MAC_COMMAND && network_check_request == 0) {
		if ((mac_rx_info.status >= MAC_INFO_STATUS_OK) || ((ack_timeout_retries_counter >= ack_timeout_retries) && (rx_slot == 1))) {
			if (message_callbacks != NULL && message_callbacks->mac_rx_done) {
				/* Send the payload to the application layer */
				message_callbacks->mac_rx_done(&mac_rx_info);
			} else {
				osal_printf("MAC: no callback set for rx\n");
			}
		}
	}
	/* MAC command answer was not received */
	if ((network_check_request == 1) && (rx_slot == 1)) {
		network_check_request = 0;
		if (message_callbacks != NULL && message_callbacks->mac_network_state) {
			mac_network_info.status = NETWORK_UNAVAILABLE;
			/* Send info to the application layer */
			message_callbacks->mac_network_state(&mac_network_info);
		}
	}
}

static void internal_txdone_cb(void)
{
	enum ack_request message_ack = WITHOUT_ACK;
	osal_time_t current_time = osal_get_time();
	osal_radio_status_t status;

	mac_lowpower_sleep_unlock();

	osal_get_radio_status(&status);

	osal_radio(OSAL_RADIO_STOP);
	osal_clear_radio_callback();

	open_receive_window = 1;

	if (loramac_device_class == CLASS_C) {
		osal_post_job(&rx2_job, on_rx_window2_timer_event);
	}

	// Store last Tx channel
	last_tx_channel = current_channel;
	// Update last tx done time for the current channel
	bands[channels[last_tx_channel].band].last_tx_done_time = current_time;
	// Update Aggregated last tx done time
	aggregated_last_tx_done_time = current_time;

	message_ack = message_ack_request(&message_tx_rx);

	timestamp_tx_done = status.txend;

	/* Received the ack with margin and number of gateway */
	osal_post_delayed_job(&rx1_job, timestamp_tx_done + ms2us(rx_windows1_delay), on_rx_window1_timer_event);
	if (loramac_device_class != CLASS_C) {
		osal_post_delayed_job(&rx2_job, timestamp_tx_done + ms2us(rx_windows2_delay), on_rx_window2_timer_event);
	}
	if ((loramac_device_class == CLASS_C) || (message_ack == WITH_ACK)) {
		osal_post_delayed_job(&ack_timeout_job,
				      timestamp_tx_done + ms2us(rx_windows2_delay + ACK_TIMEOUT + randr(-ACK_TIMEOUT_RND, ACK_TIMEOUT_RND)),
				      on_ack_timeout_timer_event);
	}

	loramac_state = LORAMAC_STATE_WAITING_OPENING_WINDOW;
	if (message_ack == WITH_ACK) {
		osal_printf("With ACK\n");
	} else {
		osal_printf("Without ACK\n");
		channels_nb_rep_counter++;
		if (message_callbacks != NULL && message_callbacks->mac_tx_done) {
			mac_tx_info.nb_retries = ack_timeout_retries;
			mac_tx_info.status = MAC_INFO_STATUS_OK;
			mac_tx_info.ack_status = UNDEF_ACK;
			/* Send info to the application layer */
			message_callbacks->mac_tx_done(&mac_tx_info);
		} else {
			osal_printf("MAC: no callback set for tx\n");
		}
	}

	if (message_callbacks != NULL && message_callbacks->mac_state != NULL) {
		mac_state = MAC_STATE_WAITING_DATA;
		message_callbacks->mac_state(mac_state);
	}
}

static uint16_t retransmission_duty_cycle(void)
{
	uint16_t duty_cycle = 0;

	osal_time_t time_elapsed = osal_get_time();

	if (time_elapsed < ms2us(3600000)) {		//1 hour
		duty_cycle = BACKOFF_DC_1_HOUR;
	} else if (time_elapsed < ms2us(39600000)) {	//11 hours
		duty_cycle = BACKOFF_DC_10_HOURS;
	} else {
		duty_cycle = BACKOFF_DC_24_HOURS;
	}

	return duty_cycle;
}

static void calculate_back_off(uint8_t channel)
{
	uint16_t duty_cycle = bands[channels[channel].band].d_cycle;
	uint16_t join_duty_cycle = 0;
	uint8_t rnd_time_off = 0;

	if (is_loramac_network_joined == 0) {
		join_duty_cycle = retransmission_duty_cycle();
		duty_cycle = MAX(duty_cycle, join_duty_cycle);

		// Make sure to not apply the random back-off to the first TX
		if (tx_time_on_air > 0) {
			rnd_time_off = 1;
		}
	}

	// Update Band Time OFF
	if (duty_cycle_on == 1) {
		bands[channels[channel].band].time_off = tx_time_on_air * duty_cycle - tx_time_on_air;
	} else {
		bands[channels[channel].band].time_off = 0;
	}

	if (rnd_time_off == 1) {
		bands[channels[channel].band].time_off =
			randr(bands[channels[channel].band].time_off, bands[channels[channel].band].time_off + BACKOFF_RND_OFFSET);
	}

	// Update Aggregated Time OFF
	aggregated_time_off = aggregated_time_off + (tx_time_on_air * aggregated_duty_cycle - tx_time_on_air);
}

static loramac_status_t schedule_tx(void)
{
	osal_time_t duty_cycle_time_off = 0;
	int8_t tx_power_index = 0;

	loramac_state = LORAMAC_STATE_CHECKING_DUTY_CYCLE;

	// Check if the device is off
	if (max_duty_cycle == 255) {
		osal_printf("Device turning OFF\n");
		return LORAMAC_STATUS_DEVICE_OFF;
	}
	if (max_duty_cycle == 0) {
		aggregated_time_off = 0;
	}

	calculate_back_off(last_tx_channel);

	// Select channel
	while (set_next_channel(&duty_cycle_time_off) == 0) {
		// Set the default datarate
		loramac_params.channels_datarate = loramac_params_defaults.channels_datarate;

		// Re-enable default channels LC1, LC2, LC3
		loramac_params.channels_mask[0] = loramac_params.channels_mask[0] | (LC(1) + LC(2) + LC(3));
	}

	// Schedule transmission of frame
	if (duty_cycle_time_off == 0) {
		// Try to send now

		tx_rx_channel.freq = channels[current_channel].uplink_frequency;
		tx_rx_channel.sf = datarate_table[loramac_params.channels_datarate];
		tx_rx_channel.bw = OSAL_BW125;

		tx_power_index = loramac_params.channels_tx_power;
		tx_rx_channel.pmax = tx_powers[tx_power_index] - osal_get_antenna_gain();
		if (tx_rx_channel.pmax > osal_get_max_power()) {
			tx_rx_channel.pmax = osal_get_max_power();
		} else if (tx_rx_channel.pmax < osal_get_min_power()) {
			tx_rx_channel.pmax = osal_get_min_power();
		}

		send_packet(tx_rx_channel.freq, tx_rx_channel.sf, tx_rx_channel.bw, (uint8_t *) message_tx_rx.raw, message_tx_rx.msg_len,
			    tx_rx_channel.pmax);
		return LORAMAC_STATUS_OK;
	} else {
		loramac_state = LORAMAC_STATE_IDLE;

		if (network_check_request) {
			network_check_request = 0;
			if (message_callbacks != NULL && message_callbacks->mac_network_state) {
				mac_network_info.status = NETWORK_UNKNOWN;
				/* Send info to the application layer */
				message_callbacks->mac_network_state(&mac_network_info);
			}
		}

		mac_tx_info.status = MAC_INFO_STATUS_DUTY_CYCLE_LIMITATION;
		mac_tx_info.nb_retries = ack_timeout_retries_counter;
		mac_tx_info.ack_status = UNDEF_ACK;
		osal_post_job(&tx_done_cb_job, tx_done_cb_event);

		if (mac_state != MAC_STATE_IDLE) {
			mac_state = MAC_STATE_IDLE;
			osal_post_job(&mac_state_cb_job, mac_state_cb_event);
		}

		return LORAMAC_STATUS_OK;
	}
}

static void send_packet(uint32_t freq, enum osal_sf_t sf, enum osal_bw_t bw, uint8_t *data, int len, uint8_t power)
{
	osal_radio(OSAL_RADIO_STOP);
	osal_clear_radio_callback();

	radio_params.freq = freq;
	radio_params.cr = RX_TX_CR;
	radio_params.sf = sf;
	radio_params.bw = bw;
	radio_params.crc = RX_TX_CRC;
	radio_params.ih = 0;
	radio_params.txpow = power;
	radio_params.sync_word = MAC_PREAMBLE;
	radio_params.preamble_len = PREAMBLE_LENGTH;
	radio_params.payload_length = len;
	radio_params.iq = OSAL_IQ_NORMAL;
	radio_params.ppm_offset = 0;
	osal_set_radio_params(&radio_params);

	osal_write_radio_buffer(data, len);

	tx_time_on_air = get_time_on_air(len);
	timestamp_tx_done = osal_get_time() + (tx_time_on_air * 90) / 100 - ms2us(CPU_WAKEUP_TIME);
	osal_post_delayed_job(&lock_low_power_job, timestamp_tx_done, lock_low_power_event);

	/* Setup radio callback */
	osal_set_radio_callback(internal_txdone_cb);

	loramac_state = LORAMAC_STATE_SENDING_DATA;
	if (message_callbacks != NULL && message_callbacks->mac_state != NULL) {
		mac_state = MAC_STATE_SENDING_DATA;
		message_callbacks->mac_state(mac_state);
	}
	osal_radio(OSAL_RADIO_START_TX);
}

static uint8_t _check_network_available(void)
{
	mac_status_t state;
	network_check_request = 1;
	osal_printf("Checking for network availability\n");

	add_mac_command(MOTE_MAC_LINK_CHECK_REQ, 0, 0);
	state = loramac_send_when_possible_confirmed(1, NULL, 0, 1, 0);
	if (state == MAC_STATUS_NO_ERROR) {
		return 1;
	} else {
		network_check_request = 0;
		if (message_callbacks != NULL && message_callbacks->mac_network_state) {
			mac_network_info.status = NETWORK_UNKNOWN;
			/* Send info to the application layer */
			message_callbacks->mac_network_state(&mac_network_info);
		}
		return 0;
	}
}

/*!
 * \brief Function to reset the MAC
 */
static void reset_mac_parameters(void)
{
	is_loramac_network_joined = 0;

	// Counters
	uplink_counter = 0;
	downlink_counter = 0;

	channels_nb_rep_counter = 0;

	ack_timeout_retries = 1;
	ack_timeout_retries_counter = 1;

	max_duty_cycle = 0;
	aggregated_duty_cycle = 1;

	mac_commands_buffer_index = 0;
	mac_commands_buffer_to_repeat_index = 0;

	loramac_params.channels_tx_power = loramac_params_defaults.channels_tx_power;
	loramac_params.channels_datarate = loramac_params_defaults.channels_datarate;
	current_datarate = loramac_params.channels_datarate;

	loramac_params.max_rx_window = loramac_params_defaults.max_rx_window;
	loramac_params.receive_delay_1 = loramac_params_defaults.receive_delay_1;
	loramac_params.receive_delay_2 = loramac_params_defaults.receive_delay_2;
	loramac_params.join_accept_delay_1 = loramac_params_defaults.join_accept_delay_1;
	loramac_params.join_accept_delay_2 = loramac_params_defaults.join_accept_delay_2;

	loramac_params.channels_nb_rep = loramac_params_defaults.channels_nb_rep;
	loramac_params.rx1_dr_offset = loramac_params_defaults.rx1_dr_offset;
	loramac_params.min_rx_symbols = loramac_params_defaults.min_rx_symbols;
	loramac_params.max_rx_error = loramac_params_defaults.max_rx_error;

	loramac_params.rx2_channel = loramac_params_defaults.rx2_channel;

	osal_memcpy((uint8_t *) loramac_params.channels_mask, (uint8_t *) loramac_params_defaults.channels_mask,
		    sizeof(loramac_params.channels_mask));

	mac_commands_in_next_tx = 0;

	// Reset Multicast downlink counters
	multicast_params_t *cur = multicast_channels;
	while (cur != NULL) {
		cur->downlink_counter = 0;
		cur = cur->next;
	}

	// Initialize channel index.
	current_channel = LORA_MAX_NB_CHANNELS;
}



/*!
 * \brief Initializes the MAC.
 *
 * \param cb Callback receives informations after every transmission, reception, check of
 *           network, and change of MAC state.
 * \param battery_cb Callback will be called by the MAC to know the battery value of your
 *                   node.
 * \param flash_cbs Callbacks are used by the MAC to store and load data from the node’s
 *                  flash.
 */
static void loramac_init(mac_message_callbacks_t *cb, mac_battery_callback_t *battery_cb, mac_flash_callback_t *flash_cbs)
{
	message_callbacks = cb;
	battery_callback = battery_cb;
	flash_callback = flash_cbs;

	loramac_device_class = CLASS_A;
	loramac_state = LORAMAC_STATE_IDLE;
	mac_state = MAC_STATE_IDLE;

	max_duty_cycle = 0;
	aggregated_duty_cycle = 1;

	repeater_support = 0;

	// Reset duty cycle times
	aggregated_last_tx_done_time = 0;
	aggregated_time_off = 0;

	// Duty cycle
	duty_cycle_on = 1;

	// Reset to defaults
	loramac_params_defaults.channels_tx_power = LORAMAC_DEFAULT_TX_POWER;
	loramac_params_defaults.channels_datarate = LORAMAC_DEFAULT_DATARATE;

	loramac_params_defaults.max_rx_window = MAX_RX_WINDOW;
	loramac_params_defaults.receive_delay_1 = RECEIVE_DELAY1;
	loramac_params_defaults.receive_delay_2 = RECEIVE_DELAY2;
	loramac_params_defaults.join_accept_delay_1 = JOIN_ACCEPT_DELAY1;
	loramac_params_defaults.join_accept_delay_2 = JOIN_ACCEPT_DELAY2;

	loramac_params_defaults.channels_nb_rep = 1;
	loramac_params_defaults.rx1_dr_offset = 0;
	loramac_params_defaults.min_rx_symbols = 6;
	loramac_params_defaults.max_rx_error = 10000;

	loramac_params_defaults.rx2_channel = (rx2_channel_params_t) RX_WND_2_CHANNEL;

	// Channel mask
	loramac_params_defaults.channels_mask[0] = LC(1) + LC(2) + LC(3);

	reset_mac_parameters();

	// Use the last value if flash callback defined
	if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->get_uplink_counter != NULL)) {
		uplink_counter = flash_callback->get_uplink_counter();
	}
	if (is_loramac_network_ABP && (flash_callback != NULL) && (flash_callback->get_downlink_counter != NULL)) {
		downlink_counter = flash_callback->get_downlink_counter();
	}

	// Random seed initialization
	srand1((uint32_t) ((osal_rand16() << 16) | (osal_rand16())));
}

/**
  * @brief  Deinitializes the MAC
  */
static void loramac_deinit(void)
{
	/* Stop any running job */
	osal_cancel_job(&rx1_job);
	osal_cancel_job(&rx2_job);
	osal_cancel_job(&start_rx_single_job);
	osal_cancel_job(&start_rx_continuous_job);
	osal_cancel_job(&tx_delayed_job);
	osal_cancel_job(&ack_timeout_job);
	osal_cancel_job(&lock_low_power_job);

	/* Stop the Radio */
	osal_radio(OSAL_RADIO_STOP);
	osal_clear_radio_callback();

	/* Reset the variables */
	loramac_state = LORAMAC_STATE_IDLE;
	message_callbacks = NULL;
	battery_callback = NULL;
	flash_callback = NULL;
	is_loramac_network_joined = 0;
}

/*!
 * \brief Initializes the network ID. The sessions keys must be provided.
 *
 * \param netid 24 bits containing the network identifier.
 * \param devaddr 32 bits containing the device address.
 * \param nwkskey 16 bytes array containing the network session key.
 * \param appskey 16 bytes array containing the application session key.
 */
static void loramac_init_activation_personalization(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey)
{
	is_loramac_network_ABP = 1;
	is_loramac_network_joined = 0;

	mac_devaddr = devaddr;
	mac_netid = netid;
	message_init_ids(netid, devaddr, nwkskey, appskey);
	osal_memcpy(dev_nwkskey, nwkskey, sizeof(dev_nwkskey));
	osal_memcpy(dev_appskey, appskey, sizeof(dev_appskey));

	is_loramac_network_joined = 1;

	_set_device_class(loramac_device_class);
}

/*!
 * \brief Initiates the Over The Air Activation.
 *
 * \param dev_eui 8 bytes array containing the device EUI.
 * \param app_eui 8 bytes array containing the application EUI.
 * \param app_key 16 bytes array containing the application key.
 *
 * \retval mac_status_t
 */
static mac_status_t loramac_init_activation_on_air(uint8_t *dev_eui, uint8_t *app_eui, uint8_t *app_key)
{
	uint8_t i = 0;
	message_status_t message_state;
	loramac_status_t schedule_state;

	is_loramac_network_joined = 0;

	osal_memcpy(dev_deveui, dev_eui, sizeof(dev_deveui));
	osal_memcpy(dev_appeui, app_eui, sizeof(dev_appeui));
	osal_memcpy(dev_appkey, app_key, sizeof(dev_appkey));

	if ((loramac_state != LORAMAC_STATE_IDLE)
#ifdef CFG_SUPPORT_CLASS_C
		&& ((loramac_device_class == CLASS_C) && (loramac_state != LORAMAC_STATE_DOWNLINK_IDLE))
#endif
		) {
		return MAC_STATUS_BUSY;
	}

	osal_printf("Trying to JOIN network (OTAA)\n");

	message_state = message_compose_join_OTA(MSG_TYPE_JOIN_REQ, dev_eui, app_eui, &loramac_dev_nonce, app_key, &message_tx_rx);
	if (message_state < 0) {
		if (message_state == MSG_STATUS_HEADER_ERROR) {
			return MAC_STATUS_MSG_HEADER_ERROR;
		}
	}

	reset_mac_parameters();

	rx_windows1_delay = loramac_params.join_accept_delay_1 - us2ms_round(osal_get_radio_wakeup_latency()) -
			    us2ms_round(osal_get_additional_cpu_wakeup_latency()) - us2ms_round(MARGIN_TIME) - us2ms_round(RADIO_CONFIG_TIME);
	rx_windows2_delay = loramac_params.join_accept_delay_2 - us2ms_round(osal_get_radio_wakeup_latency()) -
			    us2ms_round(osal_get_additional_cpu_wakeup_latency()) - us2ms_round(MARGIN_TIME) - us2ms_round(RADIO_CONFIG_TIME);

	osal_printf("message len %d, ", message_tx_rx.msg_len);
	for (i = 0; i < message_tx_rx.msg_len; i++) {
		osal_printf("%02X ", message_tx_rx.raw[i]);
	}
	osal_printf("\n");

	schedule_state = schedule_tx();
	if (schedule_state == LORAMAC_STATUS_DEVICE_OFF) {
		return MAC_STATUS_DEVICE_OFF;
	}

	return MAC_STATUS_NO_ERROR;
}

/*!
 * \brief Prepares the next uplink to send a LinkCheckReq command.
 */
static mac_status_t loramac_link_check_req(void)
{
	return _check_network_available();
}

static mac_status_t _send_when_possible(uint8_t type, uint8_t port, const uint8_t *payload, uint8_t payload_len, uint8_t nb_retries)
{
	uint8_t fopts_len = mac_commands_buffer_index + mac_commands_buffer_to_repeat_index;
	message_status_t message_state;
	loramac_status_t schedule_state;
	uint16_t max_payload_length = 0;

	if (is_loramac_network_joined == 0) {
		return MAC_STATUS_NO_NETWORK_JOINED; // No network has been joined yet
	}

	if ((loramac_state != LORAMAC_STATE_IDLE)
#ifdef CFG_SUPPORT_CLASS_C
		&& ((loramac_device_class == CLASS_C) && (loramac_state != LORAMAC_STATE_DOWNLINK_IDLE))
#endif
		) {
		return MAC_STATUS_BUSY;
	}

	ack_timeout_retries = nb_retries;
	ack_timeout_retries_counter = 1;

	message_tx_rx.adr = adr_ctrl_on;
	message_tx_rx.adr_req = adr_next_dr(message_tx_rx.adr, 1, &loramac_params.channels_datarate);
	current_datarate = loramac_params.channels_datarate;
	max_payload_length = message_max_payload_length();

	// Testing if payload size is good to be sent
	if (max_payload_length < fopts_len) {
		return MAC_STATUS_MSG_LENGTH_ERROR;
	} else if (max_payload_length < (fopts_len + payload_len) || (max_payload_length > LORAMAC_PHY_MAXPAYLOAD)) {
		return MAC_STATUS_MSG_LENGTH_ERROR;
	}

	// Copy the MAC commands which must be re-send into the MAC command buffer
	osal_memcpy(&mac_commands_buffer[mac_commands_buffer_index], mac_commands_buffer_to_repeat, mac_commands_buffer_to_repeat_index);
	mac_commands_buffer_index += mac_commands_buffer_to_repeat_index;

	if ((payload != NULL) && (payload_len > 0)) {
		if ((mac_commands_buffer_index <= LORA_MAC_COMMAND_MAX_LENGTH) && (mac_commands_in_next_tx == 1)) {
			message_state = message_compose(type, port, payload, payload_len, mac_commands_buffer,
							mac_commands_buffer_index, uplink_counter, &message_tx_rx);
		} else {
			message_state = message_compose(type, port, payload, payload_len, NULL, 0, uplink_counter,
							&message_tx_rx);
		}
	} else {
		if ((mac_commands_buffer_index > 0) && (mac_commands_in_next_tx == 1)) {
			// MAC Commands in the payload
			message_state =
				message_compose(type, 0, mac_commands_buffer, mac_commands_buffer_index, NULL, 0, uplink_counter, &message_tx_rx);
		} else {
			return MAC_STATUS_NOTHING_TO_SEND;
		}
	}
	if (message_state < 0) {
		if (message_state == MSG_STATUS_HEADER_ERROR) {
			return MAC_STATUS_MSG_HEADER_ERROR;
		}
	}

	mac_commands_in_next_tx = 0;
	// Store MAC commands which must be re-send in case the device does not receive a downlink anymore
	mac_commands_buffer_to_repeat_index =
		parse_mac_commands_to_repeat(mac_commands_buffer, mac_commands_buffer_index, mac_commands_buffer_to_repeat);
	if (mac_commands_buffer_to_repeat_index > 0) {
		mac_commands_in_next_tx = 1;
	}
	mac_commands_buffer_index = 0;

	rx_windows1_delay = loramac_params.receive_delay_1 - us2ms_round(osal_get_radio_wakeup_latency()) -
			    us2ms_round(osal_get_additional_cpu_wakeup_latency()) - us2ms_round(MARGIN_TIME) - us2ms_round(RADIO_CONFIG_TIME);
	rx_windows2_delay = loramac_params.receive_delay_2 - us2ms_round(osal_get_radio_wakeup_latency()) -
			    us2ms_round(osal_get_additional_cpu_wakeup_latency()) - us2ms_round(MARGIN_TIME) - us2ms_round(RADIO_CONFIG_TIME);

	schedule_state = schedule_tx();
	if (schedule_state == LORAMAC_STATUS_DEVICE_OFF) {
		return MAC_STATUS_DEVICE_OFF;
	}

	return MAC_STATUS_NO_ERROR;
}

/*!
 * \brief Sends data without acknowledge.
 *
 * \details This function asks the MAC to send a message as soon as possible, without
 *          requesting an acknowledge from the network.
 *
 * \param port is a number that can be freely chosen between 1 and 223, while 0 is
 *             used for MAC commands. Values between 224 and 255 are reserved.
 * \param payload corresponds to the message to send.
 * \param payload_len is length of the message.
 * \param dl_mode ignored in this function.
 *
 * \retval mac_status_t
 */
static mac_status_t loramac_send_when_possible(uint8_t port, uint8_t *payload, uint8_t payload_len, downlink_mode_t dl_mode)
{
	return _send_when_possible(MSG_TYPE_DATA_UNCONFIRMED_UP, port, payload, payload_len, 1);
}

/*!
 * \brief Sends data with acknowledge.
 *
 * \details This function does the same thing as mac_send_when_possible(), but also
 *          asks an acknowledge from the network.
 *
 * \param port is a number that can be freely chosen between 1 and 223, while 0 is
 *             used for MAC commands. Values between 224 and 255 are reserved.
 * \param payload corresponds to the message to send.
 * \param payload_len is length of the message.
 * \param nb_retries allows to set how many times the MAC layer should retry to send
 *                   the message if it does not receive the expected acknowledge.
 * \param dl_mode ignored in this function.
 *
 * \retval mac_status_t
 */
static mac_status_t loramac_send_when_possible_confirmed(uint8_t port, uint8_t *payload, uint8_t payload_len, uint8_t nb_retries,
							 downlink_mode_t dl_mode)
{
	return _send_when_possible(MSG_TYPE_DATA_CONFIRMED_UP, port, payload, payload_len, nb_retries);
}

/*!
 * \brief Sets the LoRaWAN device class.
 */
static void loramac_set_device_class(device_class_t device_class)
{
	if (device_class == loramac_device_class) {
		return;
	}

	_set_device_class(device_class);
}

/*!
 * \brief Gets the current LoRaMAC device address.
 * \return uint32_t
 */
uint32_t loramac_get_device_address(void)
{
	if (!is_loramac_network_joined) {
		return 0x00000000;
	} else {
		return mac_devaddr;
	}
}

/*!
 * \remark Internal function.
 * \brief Returns max message length.
 * \return uint16_t
 */
uint16_t lora_mac_max_message_length(void)
{
	// Get the maximum payload length
	if (repeater_support == 1) {
		return max_payload_of_datarate_repeater[current_datarate];
	} else {
		return max_payload_of_datarate[current_datarate];
	}
}

/*!
 * \brief Sets the datarate of the RX2 window.
 * \param 0-6 corresponding to DR_X.
 */
void lora_mac_set_rx2_datarate(uint8_t dr)
{
	loramac_params.rx2_channel.datarate = dr;
}

/*!
 * \brief Enables/disables the duty cycle limitation (for test purpose).
 * \param 0: disable
 *        1: enable
 */
void lora_mac_test_set_duty_cycle(uint8_t enable)
{
	duty_cycle_on = enable;
}

mac_interface_t loramac = {
	.mac_init = &loramac_init,
	.mac_deinit = &loramac_deinit,
	.mac_init_activation_personalization = &loramac_init_activation_personalization,
	.mac_init_activation_on_air = &loramac_init_activation_on_air,
	.mac_network_available = &loramac_link_check_req,
	.mac_send_when_possible = &loramac_send_when_possible,
	.mac_send_when_possible_confirmed = &loramac_send_when_possible_confirmed,
	.mac_set_device_class = &loramac_set_device_class,
	.mac_get_device_address = &loramac_get_device_address,
};
