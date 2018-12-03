
#include "AP_GPS_ZHD.h"

const char AP_GPS_ZHD::_initialisation_blob[] = ZHD_UNLOGALL ZHD_CUSTOM_5HZ ZHD_SAVECONFIG;

AP_GPS_ZHD::AP_GPS_ZHD(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port)
        : AP_GPS_Backend(_gps, _state, _port) {
    decode_init();
    _gps.send_blob_start(state.instance, _initialisation_blob, sizeof(_initialisation_blob));
}

void AP_GPS_ZHD::decode_init() {
    _step = 0;
    _ck_xor = 0;
    _payload_counter = 0;
    _fix_counter = 0;
}

bool AP_GPS_ZHD::read() {
    uint8_t data;
    int32_t numc;
    bool parsed = false;

    numc = port->available();
    for (uint16_t i = 0; i < numc; i++) {
        // read the next byte
        data = static_cast<uint8_t>(port->read());

restart:
        switch (_step) {
            case 0:
                if (data == ZHD_SYNC1) {
                    add_byte_to_check_xor(data);
                    _buffer[_payload_counter] = data;
                    _payload_counter++;
                    _step++;
                }
                break;

            case 1:
                if (data == ZHD_SYNC2) {
                    add_byte_to_check_xor(data);
                    _buffer[_payload_counter] = data;
                    _payload_counter++;
                    _step++;
                } else {
                    decode_init();
                    goto restart;
                }
                break;

            case 2:
                if (_payload_counter <= sizeof(_buffer) - 1 - 2) {
                    add_byte_to_check_xor(data);
                }

                _buffer[_payload_counter] = data;
                _payload_counter++;

                if (_payload_counter >= sizeof(_buffer)) {
                    if (_ck_xor == _buffer.msg.checksum) {
                        _step++;
                    }
                } else {
                    decode_init();
                    goto restart;
                }
                break;

            case 3:
                // parse fix
                if (_buffer.msg.fix_type == FIX_3D_RTK_FLOAT) {
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FLOAT;

                } else if (_buffer.msg.fix_type == FIX_3D_RTK_FIXED){
                    state.status = AP_GPS::GPS_OK_FIX_3D_RTK_FIXED;

                } else {
                    state.status = AP_GPS::NO_FIX;
                }

                // location
                state.location.lat = static_cast<int32_t>(_buffer.msg.latitude * 1e7f);
                state.location.lng = static_cast<int32_t>(_buffer.msg.longitude * 1e7f);
                state.location.alt = static_cast<int32_t>(_buffer.msg.altitude * 1e2f); // from m to cm

                state.num_sats = static_cast<uint8_t>(_buffer.msg.satellites_used);
                state.hdop = static_cast<uint16_t>(_buffer.msg.eph * 1e2f); // from m to cm
                state.vdop = static_cast<uint16_t>(_buffer.msg.epv * 1e2f); // from m to cm

                state.have_horizontal_accuracy = true;
                state.horizontal_accuracy = _buffer.msg.eph;

                state.have_vertical_accuracy = true;
                state.vertical_accuracy = _buffer.msg.epv;

                // speed
                state.ground_speed = _buffer.msg.vel_ground_m_s;
                state.ground_course = _buffer.msg.angle_tracktrue;
                state.velocity.x = static_cast<float>(_buffer.msg.vel_n_m_s);
                state.velocity.y = static_cast<float>(_buffer.msg.vel_e_m_s);
                state.have_vertical_velocity = true;
                state.velocity.z = static_cast<float>(-_buffer.msg.vel_u_m_s); // negative speed against the sky

                // time

                if (state.status >= AP_GPS::GPS_OK_FIX_2D) {
                    if (_fix_counter == 0) {
                        uint16_t year = _buffer.msg.year_utc;
                        auto mon = static_cast<uint8_t>(_buffer.msg.month_utc);
                        auto day = static_cast<uint8_t>(_buffer.msg.day_utc);
                        auto hour = static_cast<uint8_t>(_buffer.msg.hour_utc);
                        auto min = static_cast<uint8_t>(_buffer.msg.min_utc);
                        auto sec = static_cast<uint8_t>(_buffer.msg.sec_utc / 100);
                        auto msec = static_cast<uint16_t>(_buffer.msg.sec_utc % 100 * 10);

                        int8_t rmon = mon - 2;
                        if (0 >= rmon) {
                            rmon += 12;
                            year -= 1;
                        }

                        // get time in seconds since unix epoch
                        uint32_t ret = (year/4) - (GPS_LEAPSECONDS_MILLIS / 1000UL) + 367*rmon/12 + day;
                        ret += year*365 + 10501;
                        ret = ret*24 + hour;
                        ret = ret*60 + min;
                        ret = ret*60 + sec;

                        // convert to time since GPS epoch
                        ret -= 272764785UL;

                        // get GPS week and time
                        state.time_week = ret / AP_SEC_PER_WEEK;
                        state.time_week_ms = (ret % AP_SEC_PER_WEEK) * AP_MSEC_PER_SEC;
                        state.time_week_ms += msec;

                        state.last_gps_time_ms = AP_HAL::millis();
                    }
                    // the _fix_counter is to reduce the cost of the GPS
                    // BCD time conversion by only doing it every 10s
                    // between those times we use the HAL system clock as
                    // an offset from the last fix
                    _fix_counter++;
                    if (_fix_counter == 50) {
                        _fix_counter = 0;
                    }
                }
                decode_init();
                parsed = true;
                break;

            default:
                break;
        }
    }

    return parsed;
}

bool AP_GPS_ZHD::_detect(struct ZHD_detect_state &state, uint8_t data) {
restart:
    switch (state.step) {
        case 0:
            state.ck_xor = state.payload_counter = 0;
            if (data == ZHD_SYNC1) {
                state.ck_xor ^= data;
                state.payload_counter++;
                state.step++;
            }
            break;

        case 1:
            if (data == ZHD_SYNC2) {
                state.ck_xor ^= data;
                state.payload_counter++;
                state.step++;
            } else {
                state.step = 0;
                goto restart;
            }
            break;

        case 2:
            state.ck_xor ^= data;
            if (++state.payload_counter == sizeof(_buffer) - 2)
                state.step++;
            break;

        case 3:
            if (state.ck_xor % 0xff == data) {
                state.step++;
            } else {
                state.step = 0;
                goto restart;
            }
        case 4:
            if ((state.ck_xor >> 8) % 0xff == data) {
                return true;
            } else {
                state.step = 0;
                goto restart;
            }
    }
    return false;
}

void AP_GPS_ZHD::add_byte_to_check_xor(uint8_t b) {
    _ck_xor ^= b;
}

