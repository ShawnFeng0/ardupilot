
#pragma once

#include "AP_GPS.h"
#include "GPS_Backend.h"

#define ZHD_SYNC1 0xaa
#define ZHD_SYNC2 0x33

#define ZHD_UNLOGALL        "unlogall com1\r\n"
#define ZHD_CUSTOM_5HZ      "zhd log com1 gpsdata ontime 0.2\r\n"
#define ZHD_CUSTOM_10HZ     "zhd log com1 gpsdata ontime 0.1\r\n"
#define ZHD_SAVECONFIG      "saveconfig\r\n"

#define ZHD_BAUDRATE 115200
#define ZHD_TIMEOUT_5HZ 400

typedef enum {
    ZHD_DECODE_UNINIT,
    ZHD_DECODE_GOT_HEAD1,
    ZHD_DECODE_GOT_HEAD2,
} rtk_zhd_decode_state_t;

class AP_GPS_ZHD : public AP_GPS_Backend {
public:
    AP_GPS_ZHD(AP_GPS &_gps, AP_GPS::GPS_State &_state, AP_HAL::UARTDriver *_port);

    bool read() override;

    static bool _detect(struct ZHD_detect_state &state, uint8_t data);

    const char *name() const override { return "ZHD"; }

private:
    struct PACKED diyd_zhd_msg {
        uint8_t head[2];             //2 bytes deviation 0
        uint16_t version;            //2 bytes deviation 2
        uint16_t length;             //2 bytes deviation 4
        uint16_t freq;               //2 bytes deviation 6
        float time_utc;              //4 bytes deviation 8
        uint16_t year_utc;           //2 bytes deviation 12
        uint16_t month_utc;          //2 bytes deviation 14
        uint16_t day_utc;            //2 bytes deviation 16
        uint16_t hour_utc;           //2 bytes deviation 18
        uint16_t min_utc;            //2 bytes deviation 20
        uint16_t sec_utc_in10ms;     //2 bytes deviation 22
        double latitude;             //8 bytes deviation 24
        double longitude;            //8 bytes deviation 32
        double altitude;             //8 bytes deviation 40
        float eph;                   //4 bytes deviation 48
        float epv;                   //4 bytes deviation 52
        float vel_ground_m_s;        //4 bytes deviation 56
        float angle_tracktrue;       //4 bytes deviation 60
        float angle_heading;         //4 bytes deviation 64
        float angle_pitch;           //4 bytes deviation 68
        double vel_n_m_s;            //8 bytes deviation 72
        double vel_e_m_s;            //8 bytes deviation 80
        double vel_u_m_s;            //8 bytes deviation 88
        uint16_t satellites_used;    //2 bytes deviation 96
        uint16_t satellites_track;   //2 bytes deviation 98
        float vel_ned_valid;         //4 bytes deviation 100
        uint16_t fix_type;           //2 bytes deviation 104
        float head_state;            //4 bytes deviation 106
        float head_deviation;        //4 bytes deviation 110
        uint16_t ins_state;          //2 bytes deviation 114
        double gnss_alt_delta;       //8 bytes deviation 116
        double ellipsoidal_h;        //8 bytes deviation 124
        uint16_t diff_age;           //2 bytes deviation 132
        uint8_t reserve[2];          //2 bytes deviation 134
        uint16_t checksum;           //2 bytes deviation 136
    };

    enum diyd_zhd_fix_type {
        FIX_NONE = 0,
        FIX_3D_RTK_FLOAT = 1,
        FIX_3D_RTK_FIXED = 4,
    };

    // Packet xor(exclusive or) check
    uint8_t _ck_xor;
    void add_byte_to_check_xor(uint8_t c);
    void decode_init();

    // State machine state
    uint8_t _step;
    uint8_t _payload_counter;

    uint8_t _fix_counter;

    // Receiver buffer
    union {
        DEFINE_BYTE_ARRAY_METHODS
        diyd_zhd_msg msg;
    } _buffer;

    static const char _initialisation_blob[];
};
