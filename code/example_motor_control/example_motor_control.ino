/* UWT-ME Mobile Robot Platform
*  Example: Motor Control
*
*  Control gearmotor speeds with feedback control.
*
*  Set the correct pin numbers in motor_control.h depending on your specific
*  configuration. Change PID_INT and PID_REP_INT to change the feedback loop
*  update interval and the reporting interval (for printing data to the
*  serial monitor), respectively.
*
*  @author Matthew Ford
*/
#include <Encoder.h>
#include "motor_control.h"

void setup() {
  Serial.begin(9600);
  initMotors();  // Configure motor pins
}

void loop() {

  // Drive both motors forwards for 3 sec
  long t_start = millis();
  while ((millis() - t0) <= 3000) {
    updatePID(1500, 1500, true);
  }
  motorHalt(RMOTOR);
  motorHalt(LMOTOR);
  delay(500);

  // Drive both motors backwards for 3 sec
  t_start = millis();
  while ((millis() - t0) <= 3000) {
    updatePID(1500, 1500, true);
  }
  motorHalt(RMOTOR);
  motorHalt(LMOTOR);
  delay(500);

  // Drive motors in opposite directions for 3 sec (each)
  // Spin robot clockwise
  t_start = millis();
  while ((millis() - t0) <= 3000) {
    updatePID(1500, -1500, true);
  }

  // Spin robot counterclockwise
  t_start = millis();
  while ((millis() - t0) <= 3000) {
    updatePID(1500, -1500, true);
  }
  motorHalt(RMOTOR);
  motorHalt(LMOTOR);
  delay(500);

}
