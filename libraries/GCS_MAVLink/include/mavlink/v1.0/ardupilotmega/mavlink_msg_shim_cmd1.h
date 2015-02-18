// MESSAGE SHIM_CMD1 PACKING

#define MAVLINK_MSG_ID_SHIM_CMD1 230

typedef struct __mavlink_shim_cmd1_t
{
 float ubverified; ///< the height upper bound in cm of the verified shim
 float ubunverified; ///< the height upper bound in cm of the unverified smoothing shim
 float amin; ///< The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 float pwm_accel_scale; ///< The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 float throttle_accel_scale; ///< The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 uint8_t enable; ///< 1 to enable, 0 to disable the shim
 uint8_t before; ///< 1 to run the shim before the motor mixing code, 0 for after
 uint8_t smooth; ///< 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
} mavlink_shim_cmd1_t;

#define MAVLINK_MSG_ID_SHIM_CMD1_LEN 23
#define MAVLINK_MSG_ID_230_LEN 23

#define MAVLINK_MSG_ID_SHIM_CMD1_CRC 161
#define MAVLINK_MSG_ID_230_CRC 161



#define MAVLINK_MESSAGE_INFO_SHIM_CMD1 { \
	"SHIM_CMD1", \
	8, \
	{  { "ubverified", NULL, MAVLINK_TYPE_FLOAT, 0, 0, offsetof(mavlink_shim_cmd1_t, ubverified) }, \
         { "ubunverified", NULL, MAVLINK_TYPE_FLOAT, 0, 4, offsetof(mavlink_shim_cmd1_t, ubunverified) }, \
         { "amin", NULL, MAVLINK_TYPE_FLOAT, 0, 8, offsetof(mavlink_shim_cmd1_t, amin) }, \
         { "pwm_accel_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 12, offsetof(mavlink_shim_cmd1_t, pwm_accel_scale) }, \
         { "throttle_accel_scale", NULL, MAVLINK_TYPE_FLOAT, 0, 16, offsetof(mavlink_shim_cmd1_t, throttle_accel_scale) }, \
         { "enable", NULL, MAVLINK_TYPE_UINT8_T, 0, 20, offsetof(mavlink_shim_cmd1_t, enable) }, \
         { "before", NULL, MAVLINK_TYPE_UINT8_T, 0, 21, offsetof(mavlink_shim_cmd1_t, before) }, \
         { "smooth", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_shim_cmd1_t, smooth) }, \
         } \
}


/**
 * @brief Pack a shim_cmd1 message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param enable 1 to enable, 0 to disable the shim
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_cmd1_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t enable, uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD1_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint8_t(buf, 20, enable);
	_mav_put_uint8_t(buf, 21, before);
	_mav_put_uint8_t(buf, 22, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#else
	mavlink_shim_cmd1_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.enable = enable;
	packet.before = before;
	packet.smooth = smooth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_CMD1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
}

/**
 * @brief Pack a shim_cmd1 message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param enable 1 to enable, 0 to disable the shim
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_shim_cmd1_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t enable,uint8_t before,uint8_t smooth,float ubverified,float ubunverified,float amin,float pwm_accel_scale,float throttle_accel_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD1_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint8_t(buf, 20, enable);
	_mav_put_uint8_t(buf, 21, before);
	_mav_put_uint8_t(buf, 22, smooth);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#else
	mavlink_shim_cmd1_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.enable = enable;
	packet.before = before;
	packet.smooth = smooth;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_SHIM_CMD1;
#if MAVLINK_CRC_EXTRA
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
}

/**
 * @brief Encode a shim_cmd1 struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param shim_cmd1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_cmd1_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_shim_cmd1_t* shim_cmd1)
{
	return mavlink_msg_shim_cmd1_pack(system_id, component_id, msg, shim_cmd1->enable, shim_cmd1->before, shim_cmd1->smooth, shim_cmd1->ubverified, shim_cmd1->ubunverified, shim_cmd1->amin, shim_cmd1->pwm_accel_scale, shim_cmd1->throttle_accel_scale);
}

/**
 * @brief Encode a shim_cmd1 struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param shim_cmd1 C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_shim_cmd1_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_shim_cmd1_t* shim_cmd1)
{
	return mavlink_msg_shim_cmd1_pack_chan(system_id, component_id, chan, msg, shim_cmd1->enable, shim_cmd1->before, shim_cmd1->smooth, shim_cmd1->ubverified, shim_cmd1->ubunverified, shim_cmd1->amin, shim_cmd1->pwm_accel_scale, shim_cmd1->throttle_accel_scale);
}

/**
 * @brief Send a shim_cmd1 message
 * @param chan MAVLink channel to send the message
 *
 * @param enable 1 to enable, 0 to disable the shim
 * @param before 1 to run the shim before the motor mixing code, 0 for after
 * @param smooth 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 * @param ubverified the height upper bound in cm of the verified shim
 * @param ubunverified the height upper bound in cm of the unverified smoothing shim
 * @param amin The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 * @param pwm_accel_scale The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 * @param throttle_accel_scale The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_shim_cmd1_send(mavlink_channel_t chan, uint8_t enable, uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_SHIM_CMD1_LEN];
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint8_t(buf, 20, enable);
	_mav_put_uint8_t(buf, 21, before);
	_mav_put_uint8_t(buf, 22, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
#else
	mavlink_shim_cmd1_t packet;
	packet.ubverified = ubverified;
	packet.ubunverified = ubunverified;
	packet.amin = amin;
	packet.pwm_accel_scale = pwm_accel_scale;
	packet.throttle_accel_scale = throttle_accel_scale;
	packet.enable = enable;
	packet.before = before;
	packet.smooth = smooth;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, (const char *)&packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, (const char *)&packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
#endif
}

#if MAVLINK_MSG_ID_SHIM_CMD1_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_shim_cmd1_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t enable, uint8_t before, uint8_t smooth, float ubverified, float ubunverified, float amin, float pwm_accel_scale, float throttle_accel_scale)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_float(buf, 0, ubverified);
	_mav_put_float(buf, 4, ubunverified);
	_mav_put_float(buf, 8, amin);
	_mav_put_float(buf, 12, pwm_accel_scale);
	_mav_put_float(buf, 16, throttle_accel_scale);
	_mav_put_uint8_t(buf, 20, enable);
	_mav_put_uint8_t(buf, 21, before);
	_mav_put_uint8_t(buf, 22, smooth);

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, buf, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
#else
	mavlink_shim_cmd1_t *packet = (mavlink_shim_cmd1_t *)msgbuf;
	packet->ubverified = ubverified;
	packet->ubunverified = ubunverified;
	packet->amin = amin;
	packet->pwm_accel_scale = pwm_accel_scale;
	packet->throttle_accel_scale = throttle_accel_scale;
	packet->enable = enable;
	packet->before = before;
	packet->smooth = smooth;

#if MAVLINK_CRC_EXTRA
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, (const char *)packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN, MAVLINK_MSG_ID_SHIM_CMD1_CRC);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_SHIM_CMD1, (const char *)packet, MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
#endif
}
#endif

#endif

// MESSAGE SHIM_CMD1 UNPACKING


/**
 * @brief Get field enable from shim_cmd1 message
 *
 * @return 1 to enable, 0 to disable the shim
 */
static inline uint8_t mavlink_msg_shim_cmd1_get_enable(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  20);
}

/**
 * @brief Get field before from shim_cmd1 message
 *
 * @return 1 to run the shim before the motor mixing code, 0 for after
 */
static inline uint8_t mavlink_msg_shim_cmd1_get_before(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  21);
}

/**
 * @brief Get field smooth from shim_cmd1 message
 *
 * @return 1 to run the unverified smoothing shim, 0 to not run the unverified smoothing shim
 */
static inline uint8_t mavlink_msg_shim_cmd1_get_smooth(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field ubverified from shim_cmd1 message
 *
 * @return the height upper bound in cm of the verified shim
 */
static inline float mavlink_msg_shim_cmd1_get_ubverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  0);
}

/**
 * @brief Get field ubunverified from shim_cmd1 message
 *
 * @return the height upper bound in cm of the unverified smoothing shim
 */
static inline float mavlink_msg_shim_cmd1_get_ubunverified(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  4);
}

/**
 * @brief Get field amin from shim_cmd1 message
 *
 * @return The acceleration in cm/s/s issued when the verified shim is engaged. Should be at least the acceleration due to gravity.
 */
static inline float mavlink_msg_shim_cmd1_get_amin(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  8);
}

/**
 * @brief Get field pwm_accel_scale from shim_cmd1 message
 *
 * @return The ratio between the signal sent to a motor and the acceleration in cm/s/s produced by that motor
 */
static inline float mavlink_msg_shim_cmd1_get_pwm_accel_scale(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  12);
}

/**
 * @brief Get field throttle_accel_scale from shim_cmd1 message
 *
 * @return The ratio between the throttle issued to the motor mixing code and the acceleration in cm/s/s produced by that motor
 */
static inline float mavlink_msg_shim_cmd1_get_throttle_accel_scale(const mavlink_message_t* msg)
{
	return _MAV_RETURN_float(msg,  16);
}

/**
 * @brief Decode a shim_cmd1 message into a struct
 *
 * @param msg The message to decode
 * @param shim_cmd1 C-struct to decode the message contents into
 */
static inline void mavlink_msg_shim_cmd1_decode(const mavlink_message_t* msg, mavlink_shim_cmd1_t* shim_cmd1)
{
#if MAVLINK_NEED_BYTE_SWAP
	shim_cmd1->ubverified = mavlink_msg_shim_cmd1_get_ubverified(msg);
	shim_cmd1->ubunverified = mavlink_msg_shim_cmd1_get_ubunverified(msg);
	shim_cmd1->amin = mavlink_msg_shim_cmd1_get_amin(msg);
	shim_cmd1->pwm_accel_scale = mavlink_msg_shim_cmd1_get_pwm_accel_scale(msg);
	shim_cmd1->throttle_accel_scale = mavlink_msg_shim_cmd1_get_throttle_accel_scale(msg);
	shim_cmd1->enable = mavlink_msg_shim_cmd1_get_enable(msg);
	shim_cmd1->before = mavlink_msg_shim_cmd1_get_before(msg);
	shim_cmd1->smooth = mavlink_msg_shim_cmd1_get_smooth(msg);
#else
	memcpy(shim_cmd1, _MAV_PAYLOAD(msg), MAVLINK_MSG_ID_SHIM_CMD1_LEN);
#endif
}
