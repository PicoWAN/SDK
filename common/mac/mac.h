/** \file
 *
 * \brief MAC unified API for PicoWAN and LoRaWAN network access
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

#ifndef _MAC_H_
#define _MAC_H_

#include <stdint.h>

#define MAC_PORT_MAC_COMMAND	0
#define MAC_PORT_FIRST_APP	1

typedef enum {
	UNDEF_MAC = -1,					///< MAC is not set
	PICOMAC = 0,					///< PicoWAN protocol
	LORAMAC = 1,					///< LoRaWAN protocol
} mac_type_t;

typedef enum {
	DL_DISABLED = 0,				///< Do not wait for downlink
	DL_ENABLED = 1,					///< Wait for downlink
} downlink_mode_t;

enum ack_status {
	UNDEF_ACK,
	NO_ACK,
	ACK
};

typedef enum e_device_class {
	CLASS_A,
	CLASS_B,
	CLASS_C,
} device_class_t;


typedef enum {
	MAC_STATE_IDLE = 0,				///< MAC is in an idle state
	MAC_STATE_SEARCHING_NETWORK = 1,		///< MAC is searching network
	MAC_STATE_SENDING_DATA = 2,			///< MAC is sending data
	MAC_STATE_WAITING_ACK = 3,			///< MAC is waiting an acknowledge
	MAC_STATE_WAITING_RETRY = 4,			///< MAC is waiting to retry
	MAC_STATE_WAITING_DATA = 5,			///< MAC is waiting data
} mac_state_t;

typedef enum {
	MAC_INFO_STATUS_DUTY_CYCLE_LIMITATION = -8,	///< The MAC cannot send right now due to duty cycle limitation
	MAC_INFO_STATUS_DOWNLINK_DECODING_ERROR = -7,	///< Problem while decoding the received message
	MAC_INFO_STATUS_DOWNLINK_TIMEOUT = -6,	 	///< No message received
	MAC_INFO_STATUS_RX_ERROR = -5,			///< Problem in the reception
	MAC_INFO_STATUS_TX_MAX_MSG_SIZE_EXCEEDED = -4,	///< Payload length exceed the maximum size allowed
	MAC_INFO_STATUS_MIC_FAILED = -3,		///< Message with the wrong MIC
	MAC_INFO_STATUS_BEACON_TIMEOUT = -2,		///< Too much time to receive a beacon
	MAC_INFO_STATUS_TX_ACK_TIMEOUT = -1,		///< Too much time waiting for TX acknowledge
	MAC_INFO_STATUS_OK = 0,				///< No problem
	MAC_INFO_STATUS_JOIN_ACCEPT_OK = 1,		///< Network is joined
} mac_info_status_t;

typedef enum {
	NETWORK_UNAVAILABLE = 0,			///< Network unavailable
	NETWORK_AVAILABLE = 1,				///< Network available
	NETWORK_UNKNOWN = 2,				///< Network unknown
} mac_network_status_t;

typedef enum {
	MAC_STATUS_NOTHING_TO_SEND = -8,		///< The MAC has nothing to send
	MAC_STATUS_DEVICE_OFF = -7,			///< The device is switched off by software (LoRaWAN only)
	MAC_STATUS_BUSY = -6,				///< The MAC is busy doing something
	MAC_STATUS_FUNCTION_NOT_FOUND = -5,		///< Function not implemented for this MAC
	MAC_STATUS_NOT_INITIALIZED = -4,		///< MAC not initialized
	MAC_STATUS_MSG_LENGTH_ERROR = -3,		///< Message length error
	MAC_STATUS_MSG_HEADER_ERROR = -2,		///< Message header error
	MAC_STATUS_NO_NETWORK_JOINED = -1,		///< Impossible to send a message because the network has not been joined
	MAC_STATUS_NO_ERROR = 0,			///< Message successfully sent
} mac_status_t;

/*!
 * \brief MAC transmit information.
 */
typedef struct {
	mac_info_status_t status;			///< Sending status
	enum ack_status ack_status;			///< Acknowledge status
	uint8_t nb_retries;				///< Number of sending retries
	int64_t waiting_time;				///< Time to wait in us before to send again
} mac_tx_info_t;

/*!
 * \brief MAC receive information.
 */
typedef struct {
	mac_info_status_t status;
	uint8_t port;					///< MAC port the message is send to (1..223, while 0 is used for MAC commands)
	uint8_t *payload;				///< pointer to received message
	uint8_t payload_len;				///< length of the message
	int16_t rssi;					///< received signal strength in dBm
	int8_t snr;					///< received signal to noise ratio in dB
} mac_rx_info_t;

/*!
 * \brief MAC network state information.
 */
typedef struct {
	mac_network_status_t status;
} mac_network_info_t;

/*!
 * \brief MAC events structure.
 *        Used to notify upper layers after each MAC events done.
 */
typedef struct {
	/*!
	 * \brief MAC layer tx callback prototype.
	 *
	 * \param info  Details about MAC tx occurred (status, ack status, number of retries).
	 */
	void (*mac_tx_done)(mac_tx_info_t *info);

	/*!
	 * \brief MAC layer rx callback prototype.
	 *
	 * \param info  Details about MAC rx occurred (status, port, payload and lenght, rssi, snr).
	 */
	void (*mac_rx_done)(mac_rx_info_t *info);

	/*!
	 * \brief MAC layer network state info callback prototype.
	 *
	 * \param info  Details about MAC network state.
	 */
	void (*mac_network_state)(mac_network_info_t *info);

	/*!
	 * \brief MAC layer state callback prototype.
	 *
	 * \param info  Details about MAC state.
	 */
	void (*mac_state)(mac_state_t state);

} mac_message_callbacks_t;

typedef struct {
	/*!
	 * \brief   Measures the battery level.
	 *
	 * \retval  Battery level [0: node is connected to an external
	 *          power source, 1..254: battery level, where 1 is the minimum
	 *          and 254 is the maximum value, 255: the node was not able
	 *          to measure the battery level]
	 */
	uint8_t (*get_battery_level)(void);
} mac_battery_callback_t;

typedef struct {
	/*!
	 * \brief   Reads the Uplink counter from flash (last counter value needed after a reset).
	 *
	 * \retval  Uplink counter
	 */
	uint32_t (*get_uplink_counter)(void);

	/*!
	 * \brief   Stores the Uplink counter.
	 */
	void (*save_uplink_counter)(uint32_t uplink_counter);

	/*!
	 * \brief   Reads the Downlink counter from flash (last counter value needed after a reset).
	 *
	 * \retval  Downlink counter
	 */
	uint32_t (*get_downlink_counter)(void);

	/*!
	 * \brief   Stores the Downlink counter.
	 */
	void (*save_downlink_counter)(uint32_t downlink_counter);
} mac_flash_callback_t;


/*!
 * \brief Initializes the MAC.
 *
 * \details This function initializes the MAC layer with the type of MAC and callbacks
 *          to use. You can choose between PICOMAC and LORAMAC. Both callbacks, battery_cb
 *          and flash_cbs, are dedicated to the LoRaWAN. The battery callback will be
 *          called by the MAC to know the battery value of your node, and therefore must be
 *          implemented for a working LoRaWAN. The battery level of your node have to be
 *          converted to a value between 1 and 254. 0 means your node is connected to an
 *          external power source, 255 means that you cannot measure the battery  value.
 *
 *          The flash callbacks are used by the MAC to store and load data from the node’s
 *          flash. For now, only the LoRa counters in ABP mode needs to be stored.
 *
 *          Use NULL for unused callbacks.
 *
 * \remark flash_cbs only used with LoRaWAN.
 *
 * \param mac The MAC to use.
 * \param cb Callback receives informations after every transmission, reception, check of
 *           network, and change of MAC state.
 * \param battery_cb Callback will be called by the MAC to know the battery value of your
 *                   node (LoRaWAN specific).
 * \param flash_cbs Callbacks are used by the MAC to store and load data from the node’s
 *                  flash (LoRaWAN specific).
 *
 * \retval mac_status_t
 */
mac_status_t mac_init(mac_type_t mac, mac_message_callbacks_t *cb, mac_battery_callback_t *battery_cb, mac_flash_callback_t *flash_cbs);

/*!
 * \brief Deinitializes the MAC.
 *
 * \details This function properly deinitializes the MAC by performing all necessary cleanup
 *          actions.
 *
 * \retval mac_status_t
 */
mac_status_t mac_deinit(void);

/*!
 * \brief Initiates the Activation By Personalization (ABP). The session keys must be provided.
 *
 * \details This is the first activation mode. This mode is the only one currently
 *          available for PicoMAC stack. In this mode, the various keys and IDs are
 *          already present in the node, and must be forwarded to the MAC layer.
 *          You will have to handle that way the network ID, the device address,
 *          and the session keys (NwkSKey and AppSKey).
 *
 * \param netid 24 bits containing the network identifier.
 * \param devaddr 32 bits containing the device address.
 * \param nwkskey 16 bytes array containing the network session key.
 * \param appskey 16 bytes array containing the application session key.
 *
 * \retval mac_status_t
 */
mac_status_t mac_init_activation_personalization(uint32_t netid, uint32_t devaddr, uint8_t *nwkskey, uint8_t *appskey);

/*!
 * \brief Initiates the Over The Air Activation (OTAA). Sends a request with the device and
 *        application EUI, and waits for a response to compute sessions keys.
 *
 * \details This is the second activation mode. In this mode, the node just knows
 *          the application EUI and key, as well as the device EUI. Thanks to those,
 *          the MAC will try to join the network, which, in turn, will provide the
 *          needed keys and IDs to the node.
 *
 * \param dev_eui 8 bytes array containing the device EUI in little-endian.
 * \param app_eui 8 bytes array containing the application EUI in little-endian.
 * \param app_key 16 bytes array containing the application key.
 *
 * \remark For now, only used with LoRaWAN.
 *
 * \retval mac_status_t
 */
mac_status_t mac_init_activation_on_air(uint8_t *dev_eui, uint8_t *app_eui, uint8_t *app_key);

/*!
 * \brief Sends data without acknowledge.
 *
 * \details This function asks the MAC to send a message as soon as possible, without
 *          requesting an acknowledge from the network.
 *
 * \param port is a number that can be freely chosen between 1 and 223, while 0 is
 *             used for MAC commands. Values between 224 and 255 are reserved.
 * \param payload corresponds to the message to send.
 * \param payload_len is the length of the message.
 * \param dl_mode indicates if the MAC should listen for downlink or not (when available).
 *
 * \retval mac_status_t
 */
mac_status_t mac_send_when_possible(uint8_t port, uint8_t *payload, uint8_t payload_len, downlink_mode_t dl_mode);

/*!
 * \brief Sends data with acknowledge.
 *
 * \details This function does the same thing as mac_send_when_possible(), but also
 *          asks an acknowledge from the network.
 *
 * \param port is a number that can be freely chosen between 1 and 223, while 0 is
 *             used for MAC commands. Values between 224 and 255 are reserved.
 * \param payload corresponds to the message to send.
 * \param payload_len is the length of the message.
 * \param nb_retries allows to set how many times the MAC layer should try to send
 *                   the message if it does not receive the expected acknowledge.
 * \param dl_mode indicates if the MAC should listen for downlink or not (when available).
 *
 * \retval mac_status_t
 */
mac_status_t mac_send_when_possible_confirmed(uint8_t port, uint8_t *payload, uint8_t payload_len, uint8_t nb_retries, downlink_mode_t dl_mode);

/*!
 * \brief  Reports network availability using callback. It corresponds to the LoRaWAN MAC
 *         command Link Check Req.
 *
 * \details This function allows to see if your node can reach a gateway. The state will
 *          be received by the appropriate callback.
 *
 * \retval mac_status_t
 */
mac_status_t mac_network_available(void);

/*!
 * \brief Sets end-device class. (class A, B or C for LoRaWAN, A or C for PicoWAN)
 *
 * \details This function allows to select the class to use.
 *          For both MAC, you can choose between class A and class C.
 *          Those classes correspond respectively to "low power" (the node is listening
 *          for incoming data for a short time after transmission) and "high power"
 *          (the node is always listening) modes.
 *
 * \param device_class class to set. [CLASS_A, CLASS_B, CLASS_C]
 *
 * \retval mac_status_t
 */
mac_status_t mac_set_device_class(device_class_t device_class);

/*!
 * \brief Gets device address.
 *
 * \details This function returns the device address of the current MAC.
 *          0x00000000 is the default device address, before joining a network.
 *
 * \retval mac_status_t
 */
mac_status_t mac_get_device_address(uint32_t *device_address);

/*!
 * \brief  Returns maximum allowed bytes for user payload.
 *
 * \details This function returns the maximum length allowed for a message. Note that
 *          for PicoMAC max length is 20 bytes.
 *
 * \retval uint16_t Max payload length.
 */
uint16_t mac_message_max_payload_length(void);

/*!
 * \brief  Returns the stack in use.
 *
 * \retval mac_type_t.
 */
mac_type_t mac_get_stack(void);

#endif /* _MAC_H_ */
