#include "RocketMath.h"             // for Kalman filter, trajectory, and flaps
#include "Vehicle.h"                // for rocket vehicle characteristics

#include <iostream>
#include <stdint.h>

/* FLIGHT STATE DEFINITIONS */
#define ON_LAUNCHPAD                0 // indicates rocket is on the launchpad.
#define MOTOR_BURN                  1 // indicates motor is burning. accel > 0.
#define APOGEE_COAST                2 // indicates motor has burned out. accel < 0.
#define POST_APOGEE                 3 // indicates vehicle is falling
#define NOMINAL_DT                  0.05 // seconds

double LAUNCHPAD_ALT;               // altitude of launchsite
double REST_ACCEL;                  // acceleration of non-accelerating vehicle
long int BEGIN_TIME;                // time the main loop begins
uint8_t stage = ON_LAUNCHPAD;       // current state of the vehicle

KalmanFilter filter(0,0.0001);

/*
 *      __
 *  ___( o)>   i am the wise duck of code, your code will compile
 *  \ <_. )    without errors, but only if you say "compile well ducko"
 *   `---'
 */

void update_time(double* current_time, double* dt)
{
    // get current time and time since last call
    static double currenttime;
    *current_time = currenttime;
    *dt = NOMINAL_DT;
    currenttime += NOMINAL_DT;
}

double getAcceleration(uint8_t measurements)
{
    return 20;
}

double getAltitude(uint8_t measurements)
{
    return 0;
}

int main()
{
    std::ifstream in_alt("alt");
    std::ifstream in_accel("accel");
    REST_ACCEL = 0;
    float raw_altitude;
    float raw_accel;

    // initialize the Kalman filter
    // state transition matrix, F
    filter.F[0][0] = 1;
    filter.F[1][1] = 1;
    // sensor covariance matrix, R
    filter.R[0][0] = 0.08;
    filter.R[1][1] = 0.001;
    // state matrix, X
    filter.X[0][0] = 0;
    filter.X[1][0] = 0;
    // uncertainty matrix, P
    filter.P[0][0] = 100;
    filter.P[1][1] = 100;

    double current_time = 0;

    std::cout << "time\traw alt\tk alt\traw accel\tk accel\tvel\ts1\ts2\n";

    while(!in_alt.fail() && !in_accel)
    {
      in_alt>>raw_altitude;
      in_accel>>raw_accel;

    bool FLAP_STATE = false;
    static double alt_prev;
    double dt, raw_altitude, altitude, raw_accel, accel;
    update_time(&current_time, &dt);

    // filter wizardry to clean up alt and accel data
    filter.F[0][1] = dt * dt;
    float Z[MEAS] = {raw_altitude, raw_accel};
    float* X = filter.step((float*) Z);
    altitude = X[0];
    accel = X[1];
    double velocity = (altitude - alt_prev)/dt;

    // status indicators allow internal state to be recorded efficiently
    int major_status = 100;
    int minor_status = 0;
    // stage-specific progression logic
    const uint8_t ticksToAdvance = 0.25/NOMINAL_DT;
    if (stage == ON_LAUNCHPAD)
    {
        static uint8_t high_vel_count;
        if (velocity > 3) high_vel_count++;
        else if (high_vel_count > 0) high_vel_count--;
        if (high_vel_count > ticksToAdvance) stage++;

        major_status = ON_LAUNCHPAD;
        minor_status = high_vel_count;
        }
        if (stage == MOTOR_BURN)
        {
        static uint8_t low_accel_count;
        if (accel < -10) low_accel_count++;
        else if (low_accel_count > 0) low_accel_count--;
        if (low_accel_count > ticksToAdvance) stage++;

        major_status = MOTOR_BURN;
        minor_status = low_accel_count;
        }
        if (stage == APOGEE_COAST)
        {
        /*
        * alt_next and vel_next are the predicted state of the rocket in
        * dt seconds, assuming the flaps are deployed, and are found by
        * solving the following differential equation:
        *
        *     dv/dt + (kv^2 / m) + g = 0
        *
        * This yields the model found in Equations.h.
        * ta_predict is the time to apogee which is predicted for the
        * next step, and apo_predict is the apogee altitude,
        * assuming the flaps are retracted for the rest of the flight.
        * As long as this predicted altitude is greater than the target,
        * it is desirable to lower it for this timestep, so the flaps
        * are opened. Otherwise, opening the flaps this step will cause the
        * vehicle to brake too hard, so the flaps are closed.
        */

        const uint8_t vmin = 8; // minimum velocity for flap control

        major_status = APOGEE_COAST;

        // predictive calculations determine where vehicle will be next step
        double alt_next = trajectory::alt(altitude, velocity, DRY_MASS, K_ACTIVE, dt);
        double vel_next = trajectory::vel(velocity, DRY_MASS, K_ACTIVE, dt);

        // describe the trajectory the vehicle will be on during next step
        double ta_predict = trajectory::t_a(vel_next, DRY_MASS, K_PASSIVE);
        double apo_predict = trajectory::alt(alt_next, vel_next, DRY_MASS, K_PASSIVE, ta_predict);

        // if the predicted altitude is acceptable, engage the flaps
        if (apo_predict > TARGET_ALT && vel_next > vmin)
        {
          FLAP_STATE = true;
        }
        else // otherwise, retract the flaps
        {
          FLAP_STATE = false;
        }

        static uint8_t low_accel_count;
        if (velocity < vmin)
        {
          low_accel_count++;
          FLAP_STATE = false;
        }
        else if (low_accel_count > 0) low_accel_count--;
        if (low_accel_count > ticksToAdvance) stage++;
        minor_status = FLAP_STATE;
        }

        alt_prev = altitude; // save previous altitude for faux derivative

        printf("%5.2f\t%5.2f\t%5.2f\t%5.2f\t\t%5.2f\t%5.2f\t%d\t%d\n",
            current_time, raw_altitude, altitude, raw_accel, accel,
            velocity, major_status, minor_status);
    }
}
