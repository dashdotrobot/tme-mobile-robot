/* Motor Control Library
*  Utilities for configuring and controlling DC motors.
*
*  @author Matthew Ford
*  @version 2024.1
*/

// Define motor control pins
#define RMOTOR 0
#define RMOTOR_DIR_A 6
#define RMOTOR_DIR_B 7
#define RMOTOR_SPD 5

#define LMOTOR 1
#define LMOTOR_DIR_A 10
#define LMOTOR_DIR_B 11
#define LMOTOR_SPD 7

// Define PID options
#define PID_INT 20       // PID update interval [ms]
#define PID_REP_INT 250  // PID report interval [ms]

/* Global variables for PID motor control. You will need to tune these
*  parameters for your own robot. */
const float K_P = 0.1;    // Proportional gain
const float K_I = 0.001;  // Integral gain
const float K_D = 0.1;    // Derivative gain (damping)
const long max_err_i = 255 / K_I;

long last_enc_l = 0;  // Last encoder position [steps]
long last_enc_r = 0;
long last_pid_time = 0;  // Time of last control loop update [microsecs]
long last_pid_rep_time = 0;
long err_int_l = 0;  // Error integrals
long err_int_r = 0;
long last_speed_l = 0;  // Previous measured speed [steps/sec]
long last_speed_r = 0;

// Attach encoders
Encoder enc_r(20, 21);  // Change pin numbers to match your own circuit
Encoder enc_l(18, 19);

// Initialize motors
void initMotors() {
  pinMode(LMOTOR_DIR_A, OUTPUT);
  pinMode(LMOTOR_DIR_B, OUTPUT);
  pinMode(LMOTOR_SPD, OUTPUT);

  pinMode(RMOTOR_DIR_A, OUTPUT);
  pinMode(RMOTOR_DIR_B, OUTPUT);
  pinMode(RMOTOR_SPD, OUTPUT);
}

/* Turn motor on at desired speed
*
*  @param motor Which motor to turn on (LMOTOR or RMOTOR).
*  @param speed Desired speed, -255 to 255. Negative = backwards.
*/
void motorOn(int motor, int speed) {
  if (motor == LMOTOR) {
    digitalWrite(LMOTOR_DIR_A, (speed > 0));
    digitalWrite(LMOTOR_DIR_B, !(speed > 0));
    analogWrite(LMOTOR_SPD, abs(speed));
  } else if (motor == RMOTOR) {
    digitalWrite(RMOTOR_DIR_A, (speed > 0));
    digitalWrite(RMOTOR_DIR_B, !(speed > 0));
    analogWrite(RMOTOR_SPD, abs(speed));
  }
}

/* Stop motor FAST (electronic braking)
*
*  @param motor Which motor to turn on (LMOTOR or RMOTOR).
*/
void motorHalt(int motor) {
  if (motor == LMOTOR) {
    digitalWrite(LMOTOR_DIR_A, LOW);
    digitalWrite(LMOTOR_DIR_B, LOW);
    digitalWrite(LMOTOR_SPD, HIGH);
  } else if (motor == RMOTOR) {
    digitalWrite(RMOTOR_DIR_A, LOW);
    digitalWrite(RMOTOR_DIR_B, LOW);
    digitalWrite(RMOTOR_SPD, HIGH);
  }
}

/* Update motor voltages using feedback control
*
*  This function should be called repeatedly in a fast loop to update
*  the control signals. The PID update interval can be adjusted by
*  changing the constant PID_INT.
*
*  @param speed_l Desired speed of left motor [ticks/second].
*  @param speed_r ...              right motor.
*  @param report Print data to Serial Monitor periodically.
*/
void updatePID(long speed_l, long speed_r, bool report) {

  long curr_time = millis();  // Get current clock time
  long dt = curr_time - last_pid_time;

  if (dt >= PID_INT) {
    // Calculate motor speeds
    long curr_enc_l = enc_l.read();
    long curr_enc_r = enc_r.read();

    long speed_meas_l = (1000 * (curr_enc_l - last_enc_l)) / dt;
    long speed_meas_r = (1000 * (curr_enc_r - last_enc_r)) / dt;

    // Calculate error signals
    long err_l = speed_l - speed_meas_l;
    long err_r = speed_r - speed_meas_r;

    // Update error integrals, with anti-windup correction.
    err_int_l = constrain(err_int_l + err_l * dt, -max_err_i, max_err_i);
    err_int_r = constrain(err_int_r + err_r * dt, -max_err_i, max_err_i);

    // Calculate control signals
    int u_P_l = K_P * err_l;
    int u_P_r = K_P * err_r;
    int u_I_l = K_I * err_int_l;
    int u_I_r = K_I * err_int_r;
    int u_D_l = K_D * ((speed_meas_l - last_speed_l) / dt);
    int u_D_r = K_D * ((speed_meas_r - last_speed_r) / dt);

    int u_l = constrain(int(u_P_l + u_I_l + u_D_l), -255, 255);
    int u_r = constrain(int(u_P_r + u_I_r + u_D_r), -255, 255);

    // Update "memory" variables
    last_enc_l = curr_enc_l;
    last_enc_r = curr_enc_r;
    last_pid_time = curr_time;
    last_speed_l = speed_meas_l;
    last_speed_r = speed_meas_r;

    // Set motor speeds
    motorOn(LMOTOR, u_l);
    motorOn(RMOTOR, u_r);

    // Report values to Serial Monitor
    if (report && (curr_time - last_pid_rep_time) > PID_REP_INT) {
      Serial.print("time:");
      Serial.print(curr_time);
      Serial.print(", v_meas_l:");
      Serial.print(speed_meas_l);
      Serial.print(", v_meas_r:");
      Serial.print(speed_meas_r);
      Serial.print(", voltage_l:");
      Serial.print(u_l);
      Serial.print(", voltage_r:");
      Serial.println(u_r);

      last_pid_rep_time = curr_time;
    }
  }
}