// ARDUINO (UNO) SETUP:
// =======================
// Ping sensor = 5V, GRND, D11 (for both trigger & echo)
// Adafruit GPS = D7 & D8 (GPS Shield, but pins used internally)
// Adafruit Magnetometer Adafruit HMC5883L = I2C :  SCL (A5) &  SDA (A4)
// SD Card (D10, D11, D12, D13)
#include "bot.h"

NeoSWSerial gps_port(GNSS_PORT);

#ifdef COMPASS
LSM303 compass;
#endif

/******************************
 *       GPS information       *
 ******************************/
static NMEAGPS gps; // this parses the GPS characters
static gps_fix fix; // this holds the latest GPS fix

/******************************
   Waypoint information
******************************/
#define target_waypoint (*waypoints[waypoint_number])
#define NUM_WAYPOINTS 2
NeoGPS::Location_t *waypoints[NUM_WAYPOINTS];
int waypoint_number = 0;

#ifdef ULTRASONIC_SENSOR
NewPing sonar(US_TRIGGER_PIN, US_ECHO_PIN, US_MAX_DISTANCE); // NewPing setup of pins and maximum distance.
#endif

// NOTE: delay BLOCKS THE ISR FROM RECEIVING DATA.
static void gnss_isr(uint8_t c)
{
    gps.handle(c);
}

void setup()
{
#ifdef DEBUG
    Serial.begin(PRINTER_BAUD_RATE);
#endif

#ifdef COMPASS
    Wire.begin();
    compass.init();
    compass.enableDefault();

    compass.m_min = (LSM303::vector<int16_t>){COMPASS_CAL_MIN};
    compass.m_max = (LSM303::vector<int16_t>){COMPASS_CAL_MAX};

#endif
    DEBUG_PRINTLN(F("GNSS"));

    // GPS configurations
    gps_port.begin(GNSS_BAUD_RATE);

#ifdef FAST_GNSS
    gps.send_P(&gps_port, F("PUBX,41,1,3,3,38400,0")); // SET BAUD RATE TO 57600
    gps_port.flush();
    delay(100);
    gps_port.end();
    gps_port.begin(38400);
    send_ubx(&gps_port, UBX_UPDATE_5HZ, sizeof(UBX_UPDATE_5HZ));
#endif
    gps_port.attachInterrupt(gnss_isr);

    DEBUG_PRINTLN(F("Getting starting position"));
    // FIRST GO TO THE TARGET WAYPOINT...
    waypoints[0] = new NeoGPS::Location_t(TARGET_POSITION);

    while (1)
    {
        if (gps.available())
        {
            fix = gps.read();
            DEBUG_PRINT(F("SIV "));
            DEBUG_PRINTLN(fix.satellites);
            if (fix.valid.location && fix.satellites >= SATELLITES_MIN
                // TODO: try and use the hdop condition for getting a good starting location
                // also can get error in cm from this library - check error to end location?
                //  && fix.hdop < 2000 // from wiki - 1(000)-2(000) is excellent. note that the hdop value is a thousanth.
            )
            {
                // ...THEN GO TO THE START WAYPOINT.
                waypoints[1] = new NeoGPS::Location_t(fix.location.lat(), fix.location.lon());
                break;
            }
        }
    }

    INITIALIZE_DRIVE();

    digitalWrite(LED_BUILTIN, HIGH);
    DEBUG_PRINTLN(F("STARTED"));
}

void loop()
{
#ifndef COMPASS
    static uint8_t err_location_counter = 0;
    static uint8_t err_heading_counter = 0;
#endif

    // #define DEBUG_TIME

#ifndef COMPASS
#define MAX_ERROR 20
#define RETRY_HEADING_TIME 1000
#endif

#ifdef DEBUG_TIME
    static long old_time = millis();
#endif

    if (gps.available())
    {
        fix = gps.read();
#ifndef COMPASS
        check_stop();
        if (!fix.valid.location)
        {
            err_location_counter++;
        }
        else if (!fix.valid.heading)
        {
            err_heading_counter++;
        }
        else if (fix.satellites >= SATELLITES_MIN)
            adjust_course();
#else
        if (fix.valid.location && fix.satellites >= SATELLITES_MIN)
        {
            check_stop();
            adjust_course();
        }
#endif

#ifdef DEBUG_TIME
        DEBUG_PRINTLN(millis() - old_time);
        old_time = millis();
#endif
    }

#ifndef COMPASS
    if (err_location_counter > MAX_ERROR)
    {
        DEBUG_PRINTLN(F("GPS location invalid!"));
        err_location_counter = 0;
        diff_drive(STOP, STRAIGHT);
    }
    // DO NOT TRY TO FIND HEADING IF LOCATION IS STILL INVALID
    else if (err_heading_counter > MAX_ERROR)
    {
        DEBUG_PRINTLN(F("GPS heading invalid!"));
        err_heading_counter = 0;
        steer(STRAIGHT);
        drive(FORWARD, DRIVE_SPEED);
        delay(RETRY_HEADING_TIME);
        drive(STOP, DRIVE_SPEED);
        delay(RETRY_HEADING_TIME);
        // MOVE FORWARD FOR A SECOND TO GET A CHANGE IN POSITION
        // TO GET THE HEADING.
    }
#endif
}

void check_stop()
{
    float distance_k = fix.location.DistanceKm(target_waypoint);
    if (distance_k * 1000 <= DISTANCE_THRESHOLD_M)
    {
#ifdef ULTRASONIC_SENSOR
        find_cone();
#endif
        waypoint_number++;
        DEBUG_PRINT(F("REACHED WAYPOINT "));
        DEBUG_PRINTLN(waypoint_number);
        for (int i = 0; i < 50; i++)
        {
            digitalWrite(LED_BUILTIN, HIGH);
            delay(100);
            digitalWrite(LED_BUILTIN, LOW);
            delay(100);
        }
        if (waypoint_number == NUM_WAYPOINTS)
        {
            digitalWrite(LED_BUILTIN, LOW);
            while (1)
                ;
        }
    }
    else
    {
        DEBUG_PRINT("SIV ");
        DEBUG_PRINT(fix.satellites);
    }
}

// The function for adjusting the course of the vehicle
// Based on the current bearing and desired bearing
void adjust_course()
{
    // the distance to the current target coordinate is calculated
    // calculates the desired/"optimal" bearing based on the current location and target location
    float target_heading = fix.location.BearingToDegrees(target_waypoint);
    float distance_k = fix.location.DistanceKm(target_waypoint);

#ifdef COMPASS
    compass.read();
    float heading = compass.heading();
#else
    float heading = fix.heading();
#endif

    DEBUG_PRINT(F("angle/velocity: "));
    DEBUG_PRINT(fix.heading());
    DEBUG_PRINT(F("/"));
    DEBUG_PRINTLN(fix.speed_kph());

    DEBUG_PRINT(F("Current Location: "));
    DEBUG_PRINT(fix.latitudeL());
    DEBUG_PRINT(F(", "));
    DEBUG_PRINTLN(fix.longitudeL());

    DEBUG_PRINT(F("Current heading: "));
    DEBUG_PRINTLN(String(heading, 10));

    DEBUG_PRINT(KM_M_INT(distance_k));
    DEBUG_PRINTLN(F(" m to target"));

    DEBUG_PRINT(F("Target bearing: "));
    DEBUG_PRINTLN(target_heading);

    DEBUG_PRINT(F("NEXT WAYPOINT - "));
    DEBUG_PRINT(target_waypoint.lat());
    DEBUG_PRINT(F(","));
    DEBUG_PRINTLN(target_waypoint.lon());

    // find the difference between the current bearing and the target bearing
    int16_t error = normalize_angle(int16_t(heading - target_heading));

    DEBUG_PRINT(F(" - Error: "));
    DEBUG_PRINT(error);

    // depending on the error, the vehicle will then adjust its course
    // small delays for the left and right turns are added to ensure it doesn't oversteer.

    int drive_delay = 0;
    if (abs(error) <= HEADING_THRESHOLD)
    {
        if (KM_M_INT(distance_k) <= DISTANCE_SLOW)
            drive_delay = DRIVE_SLOW_DELAY;
        else
            drive_delay = DRIVE_FAST_DELAY;
        DEBUG_PRINTLN(F(" On course"));
        diff_drive(FORWARD, STRAIGHT);
    }
    else if (error > 0)
    {
        DEBUG_PRINTLN(F(" Adjusting towards the left"));
        diff_drive(FORWARD, LEFT);
        drive_delay = TURNING_DELAY(error);
    }
    else // if (error > 0)
    {
        DEBUG_PRINTLN(F(" Adjusting towards the right"));
        diff_drive(FORWARD, RIGHT);
        drive_delay = TURNING_DELAY(error);
    }
    DEBUG_PRINT(F("Drive delay "));
    DEBUG_PRINTLN(drive_delay);
    DEBUG_PRINTLN();
    // DO NOT BLOCK INTERRUPTS FROM GNSS BY USING SINGLE LONG delay()
    for (int i = 0; i < drive_delay; i++)
        delay(10);
    diff_drive(STOP, STRAIGHT); // WAIT FOR UPDATED LOCATION THEN ADJUST
}

unsigned long window_sum(unsigned long windowing[], uint8_t sizeof_window, unsigned long next)
{
    DEBUG_PRINT(F("sum="));
    for (uint8_t i = sizeof_window - 1; i >= 1; i--)
    {
        windowing[i] = windowing[i - 1];
        DEBUG_PRINT(windowing[i]);
        DEBUG_PRINT(",");
    }
    windowing[0] = next;
    DEBUG_PRINTLN(windowing[0]);
    unsigned long sum = 0;
    for (uint8_t i = 0; i < sizeof_window; i++)
        sum += windowing[i];
    return sum;
}

#ifdef ULTRASONIC_SENSOR
void find_cone(void)
{
    DEBUG_PRINTLN(F("Finding nearest cone..."));
    unsigned long distance = sonar.ping_cm();
    unsigned long prev_distance = distance;
#define WINDOW_SIZE 10
    unsigned long windowing[WINDOW_SIZE] = {0};
    memset(windowing, 0, WINDOW_SIZE * sizeof(unsigned long));
    unsigned long sum = 0;
    for (int i = 0; i < US_SEARCH_TIME; i++)
    {
        distance = sonar.ping_cm();
        sum = window_sum(windowing, WINDOW_SIZE, distance);
        float distance_k = fix.location.DistanceKm(target_waypoint);
        DEBUG_PRINT(F("Distance to target "));
        DEBUG_PRINT(distance);
        DEBUG_PRINT(F(", distance to gnss "));
        DEBUG_PRINT(distance_k * 1000);
        DEBUG_PRINT(F(", i="));
        DEBUG_PRINTLN(i);
        if (distance > 0) // && distance_k * 1000 <= DISTANCE_THRESHOLD_ERR_M)
        {
            // HACK TO MAKE IT WORK
            memset(windowing, 10, WINDOW_SIZE * sizeof(unsigned long));
            sum = 1000;
            // END HACK
            for (uint8_t i = 0; i < 2; i++)
            {
                // TURNING A FEW MORE STEPS.
                diff_drive(FORWARD, LEFT);
                delay(100);
                diff_drive(STOP, STRAIGHT);
                delay(100);
            }

            DEBUG_PRINTLN(F("TARGET ACQUIRED. DRIVING..."));
            diff_drive(FORWARD, STRAIGHT);
            while (sum > 10 * WINDOW_SIZE)
            {
                delay(100);
                distance = sonar.ping_cm();
                sum = window_sum(windowing, WINDOW_SIZE, distance);
                DEBUG_PRINT(F("Distance "));
                DEBUG_PRINTLN(distance);
            }
            DEBUG_PRINTLN(F("DONE."));
            diff_drive(STOP, STRAIGHT); // WAIT FOR UPDATED LOCATION THEN ADJUST
            delay(10);
            return;
        }
        diff_drive(FORWARD, LEFT);
        delay(100);
        diff_drive(STOP, STRAIGHT);
        delay(100);
    }
}
#endif