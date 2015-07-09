// MESSAGE SHIM_STATS PACKING

#define MAVLINK_MSG_ID_SHIM_STATS 232

typedef struct __mavlink_shim_stats_t
{
 float percent_rejected; ///< The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 float avg_accel_diff; ///< The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 uint8_t set_motors_from_acc_failed; ///< 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
} mavlink_shim_stats_t;

#define MAVLINK_MSG_ID_SHIM_STATS_LEN 9
#define MAVLINK_MSG_ID_232_LEN 9

#define MAVLINK_MSG_ID_SHIM_STATS_CRC 36
#define MAVLINK_MSG_ID_232_CRC 36



#define MAVLINK_MESSAGE_INFO_SHIM_STATS { \
	"SHIM_STATS", \
	3, \
	{  { "percent_rejected", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_stats_t, percent_rejected) }, \
         { "avg_accel_diff", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_stats_t, avg_accel_diff) }, \
         { "set_motors_from_acc_failed", NULL, MAVLINK_TYPE_UINT8_T, 0, 8, offsetof(mavlink_shim_stats_t, set_motors_from_acc_failed) }, \
         } \
}


/**
 * @brief Pack a shim_stats message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_uint8_t(buf, 8, set_motors_from_acc_failed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}

/**
 * @brief Pack a shim_stats message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_stats_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           float percent_rejected,float avg_accel_diff,uint8_t set_motors_from_acc_failed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_uint8_t(buf, 8, set_motors_from_acc_failed);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_STATS;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}

/**
 * @brief Encode a shim_stats struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param shim_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_stats_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_shim_stats_t* shim_stats)
{
	return mavlink_msg_shim_stats_pack(system_id, component_id, msg, shim_stats->percent_rejected, shim_stats->avg_accel_diff, shim_stats->set_motors_from_acc_failed);
}

/**
 * @brief Encode a shim_stats struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param shim_stats C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_stats_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_shim_stats_t* shim_stats)
{
	return mavlink_msg_shim_stats_pack_chan(system_id, component_id, chan, msg, shim_stats->percent_rejected, shim_stats->avg_accel_diff, shim_stats->set_motors_from_acc_failed);
}

/**
 * @brief Send a shim_stats message
 * @param chan MAVLink channel to send the message
 *
 * @param percent_rejected The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 * @param avg_accel_diff The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 * @param set_motors_from_acc_failed 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_stats_send(mavlink_channel_t chan, float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_STATS_LEN];
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_uint8_t(buf, 8, set_motors_from_acc_failed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t packet;
	packet.percent_rejected = percent_rejected;
	packet.avg_accel_diff = avg_accel_diff;
	packet.set_motors_from_acc_failed = set_motors_from_acc_failed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)&packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SHIM_STATS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_shim_stats_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  float percent_rejected, float avg_accel_diff, uint8_t set_motors_from_acc_failed)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, percent_rejected);
	_mav_put_float(buf, 4, avg_accel_diff);
	_mav_put_uint8_t(buf, 8, set_motors_from_acc_failed);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, buf, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#else
	mavlink_shim_stats_t *packet = (mavlink_shim_stats_t *)msgbuf;
	packet->percent_rejected = percent_rejected;
	packet->avg_accel_diff = avg_accel_diff;
	packet->set_motors_from_acc_failed = set_motors_from_acc_failed;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)packet, MAVLINK_MSG_ID_SHIM_STATS_LEN, MAVLINK_MSG_ID_SHIM_STATS_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_STATS, (const char *)packet, MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SHIM_STATS UNPACKING


/**
 * @brief Get field percent_rejected from shim_stats message
 *
 * @return The percentage of iterations in the last window_time milliseconds that the verified shim rejected the proposed acceleration and issued its own.
 */
static inline float mavlink_msg_shim_stats_get_percent_rejected(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field avg_accel_diff from shim_stats message
 *
 * @return The average difference in the last window_time milliseconds between proposed acceleration and the acceleration issued by the verified shim.
 */
static inline float mavlink_msg_shim_stats_get_avg_accel_diff(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field set_motors_from_acc_failed from shim_stats message
 *
 * @return 1 if set_motors_from_acc failed since the last time this message was sent. 0 otherwise.
 */
static inline uint8_t mavlink_msg_shim_stats_get_set_motors_from_acc_failed(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  8);
}

/**
 * @brief Decode a shim_stats message into a struct
 *
 * @param msg The message to decode
 * @param shim_stats C-struct to decode the message contents into
 */
static inline void mavlink_msg_shim_stats_decode(const mavlink_message_t* msg, mavlink_shim_stats_t* shim_stats)
{
#if MAVLINK_NEED_BYTE_SWAP
	shim_stats->percent_rejected = mavlink_msg_shim_stats_get_percent_rejected(msg);
	shim_stats->avg_accel_diff = mavlink_msg_shim_stats_get_avg_accel_diff(msg);
	shim_stats->set_motors_from_acc_failed = mavlink_msg_shim_stats_get_set_motors_from_acc_failed(msg);
#else
	memcpy(shim_stats, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_STATS_LEN);
#endif
}
