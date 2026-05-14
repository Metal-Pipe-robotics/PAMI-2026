#include <Arduino.h>
#include "wheel.hpp"
#include "pins.hpp"
#include "tools.hpp"

static int switched_off = 0;
static SemaphoreHandle_t xStateMutex;
static int locked_wheels = 0;

typedef struct {
  enum side_t side;
  volatile long encoderTicks;
  long ticksCounter;

  long ticksAim;
  unsigned long lastUpdateTicksDiff;
  unsigned long currentUpdateTime;
  float lastUpdateTime;
  float pwm;
  float correctedpwm;

  int encpin1;
  int encpin2;
  int ctrlpin1;
  int ctrlpin2;
  int pwmchanelcw;
  int pwmchanelccw;
} DCMotor;

DCMotor ml = {
  LEFT,   /* side */
  0,      /* encoderTicks */
  0,      /* ticksCounter */

  0,      /* ticksAim */
  0,    /* lastUpdateTicksDiff */
  0,      /* currentUpdateTime */ 
  0,      /* lastUodateTime */
  0.f,    /* pwm */
  0.f,    /* correctedpwm */

  MLEP1,  /* encpin1 */
  MLEP2,  /* encpin2 */
  MLCP1,  /* ctrlpin1 */
  MLCP2,  /* ctrlpin2 */
  0,       /* pwmchanelcw */
  1       /* pwmchanelccw */
};

DCMotor mr = {
  RIGHT,  /* side */
  0,      /* encoderTicks */
  0,      /* ticksCounter */

  0,      /* ticksAim */
  0,    /* lastUpdateTicksDiff */
  0,      /* currentUpdateTime */ 
  0,      /* lastUodateTime */
  0.f,    /* pwm */
  0.f,    /* correctedpwm */

  MREP1,  /* encpin1 */
  MREP2,  /*encpin2 */ 
  MRCP1,  /* ctrlpin1 */
  MRCP2,  /* ctrlpin2 */
  2,       /* pwmchanelcw */
  3       /* pwmchanelccw */
};

static void IRAM_ATTR read_encoder_left() {
  if (digitalRead(ml.encpin1) != HIGH)
    ml.encoderTicks--;
  else  
    ml.encoderTicks++;
}

static void IRAM_ATTR read_encoder_right() {
  if (digitalRead(mr.encpin1) != HIGH)
    mr.encoderTicks--;
  else  
    mr.encoderTicks++;
}

static void wheel_init(DCMotor *m, void (*enc_func)()) {
  pinMode(STDBYP, OUTPUT);
  pinMode(m->encpin1, INPUT);
  pinMode(m->encpin2, INPUT);
  pinMode(m->ctrlpin1, OUTPUT);
  pinMode(m->ctrlpin2, OUTPUT);

  digitalWrite(STDBYP, LOW);

  ledcSetup(m->pwmchanelcw, 1000, 8);
  ledcAttachPin(m->ctrlpin1, m->pwmchanelcw);
  ledcWrite(m->pwmchanelcw, 0);

  ledcSetup(m->pwmchanelccw, 1000, 8);
  ledcAttachPin(m->ctrlpin2, m->pwmchanelccw);
  ledcWrite(m->pwmchanelccw, 0);

  attachInterrupt(digitalPinToInterrupt(m->encpin2), enc_func, RISING);
  m->ticksAim = 0;

  xStateMutex = xSemaphoreCreateMutex();
  configASSERT(xStateMutex);
}

void wheels_init() {
  wheel_init(&ml, &read_encoder_left);
  wheel_init(&mr, &read_encoder_right);
}

static double x,y,z;
void odometry_update(const long lticks, const long rticks) {
  const float ldist = ticks_to_distance(lticks);
  const float rdist = ticks_to_distance(rticks);
  const double dist = (ldist + rdist) / 2;
  const double dz = (rdist - ldist);


  if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
    z += normalizeAngle(dz / WHEELS_SPACING);
    x += dist * cosf(z);
    y -= dist * sinf(z);
    xSemaphoreGive(xStateMutex);
  }
  // Serial.printf("xyz: \t%lf %lf %lf (%lf)\n", x, y, z, z*180.f/M_PI);
}

void encoders_update() {
  unsigned long utime = micros();
  noInterrupts();
  const long mlticks = ml.encoderTicks;
  const long mrticks = mr.encoderTicks;
  ml.encoderTicks = 0;
  mr.encoderTicks = 0;
  interrupts();
  
  ml.lastUpdateTicksDiff = mlticks;
  mr.lastUpdateTicksDiff = mrticks;

  ml.ticksCounter += mlticks;
  mr.ticksCounter += mrticks;

  mr.lastUpdateTime = mr.currentUpdateTime;
  mr.currentUpdateTime = micros();

  ml.lastUpdateTime = ml.currentUpdateTime;
  ml.currentUpdateTime = micros();
  odometry_update(mlticks, mrticks);
}

void encoders_reset() {
  noInterrupts();
  ml.encoderTicks = 0;
  mr.encoderTicks = 0;
  interrupts();
}

void wheel_update(DCMotor *m) {
  const float magicNumber = 15.f / 50.f * WHEEL_UPDATE_INTERVAL_MS;
  const long ticksDiff = (m->ticksAim - m->ticksCounter) > 100 ? 30000 : (m->ticksAim - m->ticksCounter);
  float newpwm = constrainf(ticksDiff * 0.02f, m->pwm -0.6, m->pwm + 0.6);
  if (m->pwm < 1e-6f && ticksDiff > 130) {
    newpwm = (ticksDiff >= 0.f ? 1.f : -1.f) * 20.f;
  }
  if (fabs(newpwm) < magicNumber && abs(ticksDiff) > magicNumber) 
    newpwm = ticksDiff < 0.f ? -magicNumber : magicNumber;
  else if (fabs(newpwm) < magicNumber && abs(ticksDiff) <= magicNumber)
    newpwm = 0.f;
  
  newpwm = constrainf(newpwm, - WHEEL_MAX_PWM, WHEEL_MAX_PWM);
  const int dir = (m->side * m->pwm) >= 0 ? 0 : 1;
  m->pwm = newpwm;
}

void wheel_update_rotation(DCMotor *m) {
  // Serial.printf("%s\n", __func__);
  const float magicNumber = 15.f / 50.f * WHEEL_UPDATE_INTERVAL_MS;
  const int sign = (m->ticksAim - m->ticksCounter) >= 0 ? 1 : -1;
  // const long ticksDiff = fabs(m->ticksAim - m->ticksCounter) > 300 ? sign * 30000 : (m->ticksAim - m->ticksCounter);
  const long ticksDiff = m->ticksAim - m->ticksCounter;
  float newpwm = constrainf(ticksDiff * 0.14f, m->pwm -0.1, m->pwm + 0.1);
  // if (fabs(m->pwm) < 1) {
  //   if (ticksDiff >= 0)
  //     newpwm = WHEEL_MIN_PWM;
  //   else 
  //     newpwm = -WHEEL_MIN_PWM;
  // }
  if (newpwm < WHEEL_MIN_PWM && sign >= 0) 
    newpwm = WHEEL_MIN_PWM + newpwm;
  else if (newpwm > -WHEEL_MIN_PWM && sign < 0)
    newpwm = -WHEEL_MIN_PWM + newpwm;
  // Serial.printf("%f\n", newpwm);
  // if (m->pwm < 1e-6f && ticksDiff > 130) {
  //   newpwm = (ticksDiff >= 0.f ? 1.f : -1.f) * 20.f;
  // }
  // if (fabs(newpwm) < magicNumber && abs(ticksDiff) > magicNumber) 
  //   newpwm = ticksDiff < 0.f ? -magicNumber : magicNumber;
  // else if (fabs(newpwm) < magicNumber && abs(ticksDiff) <= magicNumber)
  //   newpwm = 0.f;
  
  newpwm = constrainf(newpwm, - WHEEL_MAX_PWM_ROTATION, WHEEL_MAX_PWM_ROTATION);
  const int dir = (m->side * m->pwm) >= 0 ? 0 : 1;
  m->pwm = newpwm;
}

int is_cmd_finished() {
  // Serial.printf("%d %d\n", mr.ticksAim - mr.ticksCounter, ml.ticksAim - ml.ticksCounter);
  if (fabs(mr.ticksAim - mr.ticksCounter) + fabs(ml.ticksAim - ml.ticksCounter) < 20)
    return 1;
  return 0;
}

void dual_wheels_correction_straight(float dz) {
  float ki_value = 0.f;
  const float ki = 0.; //-2.3;
  const float kp = 1.88;
  const int lsign = (ml.pwm) < 0 ? -1 : 1;
  const int rsign = (mr.pwm) < 0 ? -1 : 1;
  // Serial.printf("%d %d %f\n", lsign, rsign, dz);
  const float value = fabs(dz); 

  // Inegrale
  ki_value += dz;
  if (fabs(dz) > (M_PI / 20))
    ki_value = 0.f;

  const float correctionCoeff = kp * value + ki * ki_value;
  const float correctedCorrectionCoeff = constrainf(correctionCoeff, -40.f, 40.f);
  // Serial.printf("Coef: \t%f\n", correctedCorrectionCoeff);
  if (dz > 0) { // Rigth ahead
    mr.correctedpwm = mr.pwm - (rsign * correctedCorrectionCoeff) * mr.pwm;
    ml.correctedpwm = ml.pwm + (lsign * correctedCorrectionCoeff) * ml.pwm;
  } else { // Left ahead
    mr.correctedpwm = mr.pwm + (rsign * correctedCorrectionCoeff) * mr.pwm;
    ml.correctedpwm = ml.pwm - (lsign * correctedCorrectionCoeff) * ml.pwm;
  }

  // Serial.printf("pwm : %f \t %f   |   %f \t    %f\n", ml.correctedpwm, mr.correctedpwm, correctedCorrectionCoeff * ml.pwm, correctedCorrectionCoeff * mr.pwm);
}

static int dirmode = 0;
void dual_wheels_correction_rotation() {
  float ki_value = 0.f;
  const float ki = 0.; //-2.3;
  const float kp = 1.6;
  const int lsign = ml.pwm < 0 ? -1 : 1;
  const int rsign = mr.pwm < 0 ? -1 : 1;
  const float value = fabs(ml.ticksAim - ml.ticksCounter) - fabs(mr.ticksAim - mr.ticksCounter); 

  // Inegrale
  ki_value += z;
  if (fabs(z) > (M_PI / 20))
    ki_value = 0.f;

  const float correctionCoeff = kp * value + ki * ki_value;
  const float correctedCorrectionCoeff = constrainf(correctionCoeff, -0.f, 0.f);
  // Serial.printf("Coef: \t%f\n", correctedCorrectionCoeff);
  if (z > 0) { // Rigth ahead
    mr.correctedpwm = mr.pwm - (rsign * correctedCorrectionCoeff) * mr.pwm;
    ml.correctedpwm = ml.pwm + (lsign * correctedCorrectionCoeff) * ml.pwm;
  } else { // Left ahead
    mr.correctedpwm = mr.pwm + (rsign * correctedCorrectionCoeff) * mr.pwm;
    ml.correctedpwm = ml.pwm - (lsign * correctedCorrectionCoeff) * ml.pwm;
  }
  // Serial.printf("Origin pwm: %f %f\n", ml.pwm, mr.pwm);
  // Serial.printf("pwm : %f \t %f   |   %f \t    %f\n", ml.correctedpwm, mr.correctedpwm, correctedCorrectionCoeff * ml.pwm, correctedCorrectionCoeff * mr.pwm);
}

/* pwm > 0 is forward, backward otherward */
void wheel_setpwm(float pwm, const enum side_t side) {
  pwm = constrainf(pwm, -120.f, 120.f); // Avoid burning the driver down to ashes
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  // Serial.printf("Set pwm %f\n", pwm);

  if (!switched_off)
    digitalWrite(STDBYP, HIGH);

  /* Ensure the pwm>0 is forward rule */
  const int dir = (m->side * pwm) >= 0.f ? 0 : 1;
  pwm = fabs(pwm);

  if (dir) {
    ledcWrite(m->pwmchanelccw, 255-pwm);
    ledcWrite(m->pwmchanelcw, 255);
  } else {
    ledcWrite(m->pwmchanelccw, 255);
    ledcWrite(m->pwmchanelcw, 255-pwm);

  }
  // Serial.printf("%s: ctrlpin1 : %d | ctrlpin2 : %d | dir : %d | pwmchanel :%d | pwm : %f\n", side == LEFT ? "LEFT" : "RIGHT", m->ctrlpin1, m->ctrlpin2, dir, m->pwmchanel, pwm);
}

void wheels_update(float dz) {

  if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
    if (!locked_wheels) {
      if (dirmode == 0) {
        wheel_update(&ml);
        wheel_update(&mr);
        dual_wheels_correction_straight(dz);
      } else if (dirmode == 1) {
        wheel_update_rotation(&ml);
        wheel_update_rotation(&mr);
        dual_wheels_correction_rotation();
      }
    }
    xSemaphoreGive(xStateMutex);
  }

  wheel_setpwm(mr.correctedpwm, RIGHT);
  wheel_setpwm(ml.correctedpwm, LEFT);
  // Serial.printf("%d %d : %ld | %d %d : %ld\n", ldir, ltpwm, ml.ticksAim - ml.ticksCounter, rdir, rtpwm, mr.ticksAim - mr.ticksCounter);
  // Serial.printf("%ld %ld\n", ml.lastUpdateTicksDiff, mr.lastUpdateTicksDiff);
}
void wheels_setpoint_relative(const float dist, const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  m->ticksAim += distance_to_ticks(dist);
  Serial.printf("%ld %ld\n", ml.ticksAim, mr.ticksAim);
}

void wheels_setpoint(const float dist, const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  m->ticksCounter = 0;
  m->ticksAim = distance_to_ticks(dist);
  // m->pwm = 0;
  Serial.printf("%ld %ld\n", ml.ticksAim, mr.ticksAim);
}
 
// 0 : straight 
// 1 : rotation
void wheels_setmode(int mode) {
  dirmode = mode;
}

void wheels_lock_wheels() {
  mr.correctedpwm = 0;
  ml.correctedpwm = 0;
  mr.pwm = 0;
  ml.pwm = 0;
  locked_wheels = 12;
}

void wheels_unlock_wheels() {
  locked_wheels = locked_wheels ? locked_wheels - 1 : 0;
}

float wheels_get_rpm(const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  return 1e6 * ticks_to_rotations(m->lastUpdateTicksDiff)/(m->currentUpdateTime - m->lastUpdateTime);
}

float wheels_get_pwm(const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  return m->pwm;
}

void wheels_switch_off() {
  switched_off = 1;
  digitalWrite(STDBYP, LOW);
}

long wheels_get_ticks(const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  return m->ticksCounter;
}

void odometry_get_pos(double *_x, double *_y, double *_z) {
  if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
    *_x = x;
    *_y = y;
    *_z = z;
    xSemaphoreGive(xStateMutex);
  }
}
