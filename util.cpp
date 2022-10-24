
#include <stdint.h>
#include <NeoSWSerial.h>

// https://stackoverflow.com/a/24396562
// Very nice angle normalization
inline int16_t normalize_angle(int16_t angle)
{
    // if (angle > 180)
    //     angle -= 360;
    // else if (angle < -180)
    //     angle += 360;
    // return angle;
    angle %= 360;
    int16_t fix = angle / 180; // Integer division!!
    return (fix) ? angle - (360 * (fix)) : angle;
}

#ifdef FAST_GNSS
// https://github.com/SlashDevin/NeoGPS/blob/master/examples/ubloxRate/ubloxRate.ino
inline void send_ubx(NeoSWSerial *gps_port, const unsigned char *progmemBytes, uint16_t len)
{
    gps_port->write(0xB5); // SYNC1
    gps_port->write(0x62); // SYNC2

    uint8_t a = 0, b = 0;
    while (len-- > 0)
    {
        uint8_t c = pgm_read_byte(progmemBytes++);
        a += c;
        b += a;
        gps_port->write(c);
    }

    gps_port->write(a); // CHECKSUM A
    gps_port->write(b); // CHECKSUM B

} // sendUBX
#endif