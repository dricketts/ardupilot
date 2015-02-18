// MESSAGE SHIM_CMD2 PACKING

#define MAVLINK_MSG_ID_SHIM_CMD2 231

typedef struct __mavlink_shim_cmd2_t
{
 uint32_t window_time; ///< The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
} mavlink_shim_cmd2_t;

#define MAVLINK_MSG_ID_SHIM_CMD2_LEN 4
#define MAVLINK_MSG_ID_231_LEN 4

#define MAVLINK_MSG_ID_SHIM_CMD2_CRC 96
#define MAVLINK_MSG_ID_231_CRC 96



#define MAVLINK_MESSAGE_INFO_SHIM_CMD2 { \
	"SHIM_CMD2", \
	1, \
	{  { "window_time", NULL, MAVLINK_TYPE_UINT32_T, 0, 0, offsetof(mavlink_shim_cmd2_t, window_time) }, \
         } \
}


/**
 * @brief Pack a shim_cmd2 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_cmd2_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD2_LEN];
	_mav_put_uint32_t(buf, 0, window_time);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#else
	mavlink_shim_cmd2_t packet;
	packet.window_time = window_time;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_CMD2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
}

/**
 * @brief Pack a shim_cmd2 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_cmd2_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD2_LEN];
	_mav_put_uint32_t(buf, 0, window_time);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#else
	mavlink_shim_cmd2_t packet;
	packet.window_time = window_time;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_CMD2;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
}

/**
 * @brief Encode a shim_cmd2 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param shim_cmd2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_cmd2_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_shim_cmd2_t* shim_cmd2)
{
	return mavlink_msg_shim_cmd2_pack(system_id, component_id, msg, shim_cmd2->window_time);
}

/**
 * @brief Encode a shim_cmd2 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param shim_cmd2 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_cmd2_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_shim_cmd2_t* shim_cmd2)
{
	return mavlink_msg_shim_cmd2_pack_chan(system_id, component_id, chan, msg, shim_cmd2->window_time);
}

/**
 * @brief Send a shim_cmd2 message
 * @param chan MAVLink channel to send the message
 *
 * @param window_time The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_cmd2_send(mavlink_channel_t chan, uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD2_LEN];
	_mav_put_uint32_t(buf, 0, window_time);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
#else
	mavlink_shim_cmd2_t packet;
	packet.window_time = window_time;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, (const char *)&packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, (const char *)&packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SHIM_CMD2_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_shim_cmd2_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint32_t window_time)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint32_t(buf, 0, window_time);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, buf, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
#else
	mavlink_shim_cmd2_t *packet = (mavlink_shim_cmd2_t *)msgbuf;
	packet->window_time = window_time;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, (const char *)packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN, MAVLINK_MSG_ID_SHIM_CMD2_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD2, (const char *)packet, MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SHIM_CMD2 UNPACKING


/**
 * @brief Get field window_time from shim_cmd2 message
 *
 * @return The shim sends various values to the GCS that are computed over the last n time. For example, the percent of time the safety mode of the shim was engaged. This field gives the amount of time in milliseconds over which these values are computed.
 */
static inline uint32_t mavlink_msg_shim_cmd2_get_window_time(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint32_t(msg,  0);
}

/**
 * @brief Decode a shim_cmd2 message into a struct
 *
 * @param msg The message to decode
 * @param shim_cmd2 C-struct to decode the message contents into
 */
static inline void mavlink_msg_shim_cmd2_decode(const mavlink_message_t* msg, mavlink_shim_cmd2_t* shim_cmd2)
{
#if MAVLINK_NEED_BYTE_SWAP
	shim_cmd2->window_time = mavlink_msg_shim_cmd2_get_window_time(msg);
#else
	memcpy(shim_cmd2, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_CMD2_LEN);
#endif
}
