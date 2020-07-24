#pragma once
// MESSAGE STG_STATUS PACKING

#define MAVLINK_MSG_ID_STG_STATUS 20000


typedef struct __mavlink_stg_status_t {
 uint16_t voltage_battery; /*< [mV] Battery voltage.*/
 uint16_t voltage_generator; /*< [mV] Generator voltage.*/
 uint16_t current_battery; /*< [mA] Battery current.*/
 uint16_t current_generator; /*< [mA] Generator current.*/
 uint16_t power_load; /*< [W] Load power.*/
 uint16_t current_charge; /*< [mA] Charger current. 0 - charging stopped.*/
 int16_t temperarture_bridge; /*< [degC] Bridge temperature.*/
 int16_t voltage_drop; /*< [mV] Voltage drop during starting.*/
 int16_t rpm_cranckshaft; /*< [rpm] Cranckshaft rotations per minute.*/
 int16_t halls_errors; /*<  Hall errors durin motor operation.*/
 uint16_t uptime; /*< [s] Uptime.*/
 uint8_t current_starter; /*< [A] Maximal starting current.*/
 uint8_t motor_state; /*<  Motor state.*/
 uint8_t stg_errors_bitmask; /*<  Errors bitmask.*/
} mavlink_stg_status_t;

#define MAVLINK_MSG_ID_STG_STATUS_LEN 25
#define MAVLINK_MSG_ID_STG_STATUS_MIN_LEN 25
#define MAVLINK_MSG_ID_20000_LEN 25
#define MAVLINK_MSG_ID_20000_MIN_LEN 25

#define MAVLINK_MSG_ID_STG_STATUS_CRC 136
#define MAVLINK_MSG_ID_20000_CRC 136



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_STG_STATUS { \
    20000, \
    "STG_STATUS", \
    14, \
    {  { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_stg_status_t, voltage_battery) }, \
         { "voltage_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_stg_status_t, voltage_generator) }, \
         { "current_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_stg_status_t, current_battery) }, \
         { "current_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_stg_status_t, current_generator) }, \
         { "power_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_stg_status_t, power_load) }, \
         { "current_charge", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_stg_status_t, current_charge) }, \
         { "temperarture_bridge", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_stg_status_t, temperarture_bridge) }, \
         { "current_starter", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_stg_status_t, current_starter) }, \
         { "voltage_drop", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_stg_status_t, voltage_drop) }, \
         { "rpm_cranckshaft", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_stg_status_t, rpm_cranckshaft) }, \
         { "halls_errors", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_stg_status_t, halls_errors) }, \
         { "uptime", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_stg_status_t, uptime) }, \
         { "motor_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_stg_status_t, motor_state) }, \
         { "stg_errors_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_stg_status_t, stg_errors_bitmask) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_STG_STATUS { \
    "STG_STATUS", \
    14, \
    {  { "voltage_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_stg_status_t, voltage_battery) }, \
         { "voltage_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 2, offsetof(mavlink_stg_status_t, voltage_generator) }, \
         { "current_battery", NULL, MAVLINK_TYPE_UINT16_T, 0, 4, offsetof(mavlink_stg_status_t, current_battery) }, \
         { "current_generator", NULL, MAVLINK_TYPE_UINT16_T, 0, 6, offsetof(mavlink_stg_status_t, current_generator) }, \
         { "power_load", NULL, MAVLINK_TYPE_UINT16_T, 0, 8, offsetof(mavlink_stg_status_t, power_load) }, \
         { "current_charge", NULL, MAVLINK_TYPE_UINT16_T, 0, 10, offsetof(mavlink_stg_status_t, current_charge) }, \
         { "temperarture_bridge", NULL, MAVLINK_TYPE_INT16_T, 0, 12, offsetof(mavlink_stg_status_t, temperarture_bridge) }, \
         { "current_starter", NULL, MAVLINK_TYPE_UINT8_T, 0, 22, offsetof(mavlink_stg_status_t, current_starter) }, \
         { "voltage_drop", NULL, MAVLINK_TYPE_INT16_T, 0, 14, offsetof(mavlink_stg_status_t, voltage_drop) }, \
         { "rpm_cranckshaft", NULL, MAVLINK_TYPE_INT16_T, 0, 16, offsetof(mavlink_stg_status_t, rpm_cranckshaft) }, \
         { "halls_errors", NULL, MAVLINK_TYPE_INT16_T, 0, 18, offsetof(mavlink_stg_status_t, halls_errors) }, \
         { "uptime", NULL, MAVLINK_TYPE_UINT16_T, 0, 20, offsetof(mavlink_stg_status_t, uptime) }, \
         { "motor_state", NULL, MAVLINK_TYPE_UINT8_T, 0, 23, offsetof(mavlink_stg_status_t, motor_state) }, \
         { "stg_errors_bitmask", NULL, MAVLINK_TYPE_UINT8_T, 0, 24, offsetof(mavlink_stg_status_t, stg_errors_bitmask) }, \
         } \
}
#endif

/**
 * @brief Pack a stg_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param current_generator [mA] Generator current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param temperarture_bridge [degC] Bridge temperature.
 * @param current_starter [A] Maximal starting current.
 * @param voltage_drop [mV] Voltage drop during starting.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param halls_errors  Hall errors durin motor operation.
 * @param uptime [s] Uptime.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stg_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t current_generator, uint16_t power_load, uint16_t current_charge, int16_t temperarture_bridge, uint8_t current_starter, int16_t voltage_drop, int16_t rpm_cranckshaft, int16_t halls_errors, uint16_t uptime, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, current_generator);
    _mav_put_uint16_t(buf, 8, power_load);
    _mav_put_uint16_t(buf, 10, current_charge);
    _mav_put_int16_t(buf, 12, temperarture_bridge);
    _mav_put_int16_t(buf, 14, voltage_drop);
    _mav_put_int16_t(buf, 16, rpm_cranckshaft);
    _mav_put_int16_t(buf, 18, halls_errors);
    _mav_put_uint16_t(buf, 20, uptime);
    _mav_put_uint8_t(buf, 22, current_starter);
    _mav_put_uint8_t(buf, 23, motor_state);
    _mav_put_uint8_t(buf, 24, stg_errors_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STG_STATUS_LEN);
#else
    mavlink_stg_status_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.current_generator = current_generator;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.temperarture_bridge = temperarture_bridge;
    packet.voltage_drop = voltage_drop;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.halls_errors = halls_errors;
    packet.uptime = uptime;
    packet.current_starter = current_starter;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STG_STATUS;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
}

/**
 * @brief Pack a stg_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param current_generator [mA] Generator current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param temperarture_bridge [degC] Bridge temperature.
 * @param current_starter [A] Maximal starting current.
 * @param voltage_drop [mV] Voltage drop during starting.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param halls_errors  Hall errors durin motor operation.
 * @param uptime [s] Uptime.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_stg_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint16_t voltage_battery,uint16_t voltage_generator,uint16_t current_battery,uint16_t current_generator,uint16_t power_load,uint16_t current_charge,int16_t temperarture_bridge,uint8_t current_starter,int16_t voltage_drop,int16_t rpm_cranckshaft,int16_t halls_errors,uint16_t uptime,uint8_t motor_state,uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, current_generator);
    _mav_put_uint16_t(buf, 8, power_load);
    _mav_put_uint16_t(buf, 10, current_charge);
    _mav_put_int16_t(buf, 12, temperarture_bridge);
    _mav_put_int16_t(buf, 14, voltage_drop);
    _mav_put_int16_t(buf, 16, rpm_cranckshaft);
    _mav_put_int16_t(buf, 18, halls_errors);
    _mav_put_uint16_t(buf, 20, uptime);
    _mav_put_uint8_t(buf, 22, current_starter);
    _mav_put_uint8_t(buf, 23, motor_state);
    _mav_put_uint8_t(buf, 24, stg_errors_bitmask);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_STG_STATUS_LEN);
#else
    mavlink_stg_status_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.current_generator = current_generator;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.temperarture_bridge = temperarture_bridge;
    packet.voltage_drop = voltage_drop;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.halls_errors = halls_errors;
    packet.uptime = uptime;
    packet.current_starter = current_starter;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_STG_STATUS_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_STG_STATUS;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
}

/**
 * @brief Encode a stg_status struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param stg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stg_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_stg_status_t* stg_status)
{
    return mavlink_msg_stg_status_pack(system_id, component_id, msg, stg_status->voltage_battery, stg_status->voltage_generator, stg_status->current_battery, stg_status->current_generator, stg_status->power_load, stg_status->current_charge, stg_status->temperarture_bridge, stg_status->current_starter, stg_status->voltage_drop, stg_status->rpm_cranckshaft, stg_status->halls_errors, stg_status->uptime, stg_status->motor_state, stg_status->stg_errors_bitmask);
}

/**
 * @brief Encode a stg_status struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param stg_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_stg_status_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_stg_status_t* stg_status)
{
    return mavlink_msg_stg_status_pack_chan(system_id, component_id, chan, msg, stg_status->voltage_battery, stg_status->voltage_generator, stg_status->current_battery, stg_status->current_generator, stg_status->power_load, stg_status->current_charge, stg_status->temperarture_bridge, stg_status->current_starter, stg_status->voltage_drop, stg_status->rpm_cranckshaft, stg_status->halls_errors, stg_status->uptime, stg_status->motor_state, stg_status->stg_errors_bitmask);
}

/**
 * @brief Send a stg_status message
 * @param chan MAVLink channel to send the message
 *
 * @param voltage_battery [mV] Battery voltage.
 * @param voltage_generator [mV] Generator voltage.
 * @param current_battery [mA] Battery current.
 * @param current_generator [mA] Generator current.
 * @param power_load [W] Load power.
 * @param current_charge [mA] Charger current. 0 - charging stopped.
 * @param temperarture_bridge [degC] Bridge temperature.
 * @param current_starter [A] Maximal starting current.
 * @param voltage_drop [mV] Voltage drop during starting.
 * @param rpm_cranckshaft [rpm] Cranckshaft rotations per minute.
 * @param halls_errors  Hall errors durin motor operation.
 * @param uptime [s] Uptime.
 * @param motor_state  Motor state.
 * @param stg_errors_bitmask  Errors bitmask.
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_stg_status_send(mavlink_channel_t chan, uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t current_generator, uint16_t power_load, uint16_t current_charge, int16_t temperarture_bridge, uint8_t current_starter, int16_t voltage_drop, int16_t rpm_cranckshaft, int16_t halls_errors, uint16_t uptime, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_STG_STATUS_LEN];
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, current_generator);
    _mav_put_uint16_t(buf, 8, power_load);
    _mav_put_uint16_t(buf, 10, current_charge);
    _mav_put_int16_t(buf, 12, temperarture_bridge);
    _mav_put_int16_t(buf, 14, voltage_drop);
    _mav_put_int16_t(buf, 16, rpm_cranckshaft);
    _mav_put_int16_t(buf, 18, halls_errors);
    _mav_put_uint16_t(buf, 20, uptime);
    _mav_put_uint8_t(buf, 22, current_starter);
    _mav_put_uint8_t(buf, 23, motor_state);
    _mav_put_uint8_t(buf, 24, stg_errors_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS, buf, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
#else
    mavlink_stg_status_t packet;
    packet.voltage_battery = voltage_battery;
    packet.voltage_generator = voltage_generator;
    packet.current_battery = current_battery;
    packet.current_generator = current_generator;
    packet.power_load = power_load;
    packet.current_charge = current_charge;
    packet.temperarture_bridge = temperarture_bridge;
    packet.voltage_drop = voltage_drop;
    packet.rpm_cranckshaft = rpm_cranckshaft;
    packet.halls_errors = halls_errors;
    packet.uptime = uptime;
    packet.current_starter = current_starter;
    packet.motor_state = motor_state;
    packet.stg_errors_bitmask = stg_errors_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS, (const char *)&packet, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
#endif
}

/**
 * @brief Send a stg_status message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_stg_status_send_struct(mavlink_channel_t chan, const mavlink_stg_status_t* stg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_stg_status_send(chan, stg_status->voltage_battery, stg_status->voltage_generator, stg_status->current_battery, stg_status->current_generator, stg_status->power_load, stg_status->current_charge, stg_status->temperarture_bridge, stg_status->current_starter, stg_status->voltage_drop, stg_status->rpm_cranckshaft, stg_status->halls_errors, stg_status->uptime, stg_status->motor_state, stg_status->stg_errors_bitmask);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS, (const char *)stg_status, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
#endif
}

#if MAVLINK_MSG_ID_STG_STATUS_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_stg_status_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t voltage_battery, uint16_t voltage_generator, uint16_t current_battery, uint16_t current_generator, uint16_t power_load, uint16_t current_charge, int16_t temperarture_bridge, uint8_t current_starter, int16_t voltage_drop, int16_t rpm_cranckshaft, int16_t halls_errors, uint16_t uptime, uint8_t motor_state, uint8_t stg_errors_bitmask)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint16_t(buf, 0, voltage_battery);
    _mav_put_uint16_t(buf, 2, voltage_generator);
    _mav_put_uint16_t(buf, 4, current_battery);
    _mav_put_uint16_t(buf, 6, current_generator);
    _mav_put_uint16_t(buf, 8, power_load);
    _mav_put_uint16_t(buf, 10, current_charge);
    _mav_put_int16_t(buf, 12, temperarture_bridge);
    _mav_put_int16_t(buf, 14, voltage_drop);
    _mav_put_int16_t(buf, 16, rpm_cranckshaft);
    _mav_put_int16_t(buf, 18, halls_errors);
    _mav_put_uint16_t(buf, 20, uptime);
    _mav_put_uint8_t(buf, 22, current_starter);
    _mav_put_uint8_t(buf, 23, motor_state);
    _mav_put_uint8_t(buf, 24, stg_errors_bitmask);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS, buf, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
#else
    mavlink_stg_status_t *packet = (mavlink_stg_status_t *)msgbuf;
    packet->voltage_battery = voltage_battery;
    packet->voltage_generator = voltage_generator;
    packet->current_battery = current_battery;
    packet->current_generator = current_generator;
    packet->power_load = power_load;
    packet->current_charge = current_charge;
    packet->temperarture_bridge = temperarture_bridge;
    packet->voltage_drop = voltage_drop;
    packet->rpm_cranckshaft = rpm_cranckshaft;
    packet->halls_errors = halls_errors;
    packet->uptime = uptime;
    packet->current_starter = current_starter;
    packet->motor_state = motor_state;
    packet->stg_errors_bitmask = stg_errors_bitmask;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_STG_STATUS, (const char *)packet, MAVLINK_MSG_ID_STG_STATUS_MIN_LEN, MAVLINK_MSG_ID_STG_STATUS_LEN, MAVLINK_MSG_ID_STG_STATUS_CRC);
#endif
}
#endif

#endif

// MESSAGE STG_STATUS UNPACKING


/**
 * @brief Get field voltage_battery from stg_status message
 *
 * @return [mV] Battery voltage.
 */
static inline uint16_t mavlink_msg_stg_status_get_voltage_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Get field voltage_generator from stg_status message
 *
 * @return [mV] Generator voltage.
 */
static inline uint16_t mavlink_msg_stg_status_get_voltage_generator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  2);
}

/**
 * @brief Get field current_battery from stg_status message
 *
 * @return [mA] Battery current.
 */
static inline uint16_t mavlink_msg_stg_status_get_current_battery(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  4);
}

/**
 * @brief Get field current_generator from stg_status message
 *
 * @return [mA] Generator current.
 */
static inline uint16_t mavlink_msg_stg_status_get_current_generator(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  6);
}

/**
 * @brief Get field power_load from stg_status message
 *
 * @return [W] Load power.
 */
static inline uint16_t mavlink_msg_stg_status_get_power_load(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  8);
}

/**
 * @brief Get field current_charge from stg_status message
 *
 * @return [mA] Charger current. 0 - charging stopped.
 */
static inline uint16_t mavlink_msg_stg_status_get_current_charge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  10);
}

/**
 * @brief Get field temperarture_bridge from stg_status message
 *
 * @return [degC] Bridge temperature.
 */
static inline int16_t mavlink_msg_stg_status_get_temperarture_bridge(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  12);
}

/**
 * @brief Get field current_starter from stg_status message
 *
 * @return [A] Maximal starting current.
 */
static inline uint8_t mavlink_msg_stg_status_get_current_starter(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  22);
}

/**
 * @brief Get field voltage_drop from stg_status message
 *
 * @return [mV] Voltage drop during starting.
 */
static inline int16_t mavlink_msg_stg_status_get_voltage_drop(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  14);
}

/**
 * @brief Get field rpm_cranckshaft from stg_status message
 *
 * @return [rpm] Cranckshaft rotations per minute.
 */
static inline int16_t mavlink_msg_stg_status_get_rpm_cranckshaft(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  16);
}

/**
 * @brief Get field halls_errors from stg_status message
 *
 * @return  Hall errors durin motor operation.
 */
static inline int16_t mavlink_msg_stg_status_get_halls_errors(const mavlink_message_t* msg)
{
    return _MAV_RETURN_int16_t(msg,  18);
}

/**
 * @brief Get field uptime from stg_status message
 *
 * @return [s] Uptime.
 */
static inline uint16_t mavlink_msg_stg_status_get_uptime(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint16_t(msg,  20);
}

/**
 * @brief Get field motor_state from stg_status message
 *
 * @return  Motor state.
 */
static inline uint8_t mavlink_msg_stg_status_get_motor_state(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  23);
}

/**
 * @brief Get field stg_errors_bitmask from stg_status message
 *
 * @return  Errors bitmask.
 */
static inline uint8_t mavlink_msg_stg_status_get_stg_errors_bitmask(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  24);
}

/**
 * @brief Decode a stg_status message into a struct
 *
 * @param msg The message to decode
 * @param stg_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_stg_status_decode(const mavlink_message_t* msg, mavlink_stg_status_t* stg_status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    stg_status->voltage_battery = mavlink_msg_stg_status_get_voltage_battery(msg);
    stg_status->voltage_generator = mavlink_msg_stg_status_get_voltage_generator(msg);
    stg_status->current_battery = mavlink_msg_stg_status_get_current_battery(msg);
    stg_status->current_generator = mavlink_msg_stg_status_get_current_generator(msg);
    stg_status->power_load = mavlink_msg_stg_status_get_power_load(msg);
    stg_status->current_charge = mavlink_msg_stg_status_get_current_charge(msg);
    stg_status->temperarture_bridge = mavlink_msg_stg_status_get_temperarture_bridge(msg);
    stg_status->voltage_drop = mavlink_msg_stg_status_get_voltage_drop(msg);
    stg_status->rpm_cranckshaft = mavlink_msg_stg_status_get_rpm_cranckshaft(msg);
    stg_status->halls_errors = mavlink_msg_stg_status_get_halls_errors(msg);
    stg_status->uptime = mavlink_msg_stg_status_get_uptime(msg);
    stg_status->current_starter = mavlink_msg_stg_status_get_current_starter(msg);
    stg_status->motor_state = mavlink_msg_stg_status_get_motor_state(msg);
    stg_status->stg_errors_bitmask = mavlink_msg_stg_status_get_stg_errors_bitmask(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_STG_STATUS_LEN? msg->len : MAVLINK_MSG_ID_STG_STATUS_LEN;
        memset(stg_status, 0, MAVLINK_MSG_ID_STG_STATUS_LEN);
    memcpy(stg_status, _MAV_PAYLOAD(msg), len);
#endif
}
