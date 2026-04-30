#include <Arduino.h>
#include "wheel.hpp"
#include "pins.hpp"
#include "tools.hpp"

static SemaphoreHandle_t xStateMutex;

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

void wheel_update(DCMotor *m) {
  const float magicNumber = 15.f / 50.f * WHEEL_UPDATE_INTERVAL_MS;
  const long ticksDiff = m->ticksAim - m->ticksCounter;
  float newpwm = constrainf(ticksDiff * 0.02f, m->pwm -5.8, m->pwm + 5.8);
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

void dual_wheels_correction() {
  const float kp = .075 * 180.f / M_PI;
  const int lsign = ml.pwm < 0 ? -1 : 1;
  const int rsign = mr.pwm < 0 ? -1 : 1;
  const float correctionCoeff = fabs(kp * z * (ml.pwm + mr.pwm) / 2);
  const float correctedCorrectionCoeff = constrainf(correctionCoeff, -40.f, 40.f);
  // Serial.printf("Coef: \t%f\n", correctedCorrectionCoeff);
  if (z > 0) { // Rigth ahead
    mr.correctedpwm = mr.pwm - rsign * correctedCorrectionCoeff;
    ml.correctedpwm = ml.pwm + lsign * correctedCorrectionCoeff;
  } else { // Left ahead
    mr.correctedpwm = mr.pwm + rsign * correctedCorrectionCoeff;
    ml.correctedpwm = ml.pwm - lsign * correctedCorrectionCoeff;
  }
  // Serial.printf("pwm : %f \t %f\n", ml.correctedpwm, mr.correctedpwm);
}

/* pwm > 0 is forward, backward otherward */
void wheel_setpwm(float pwm, const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;

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

void wheels_update() {

  if (xSemaphoreTake(xStateMutex, portMAX_DELAY) == pdTRUE) {
    wheel_update(&ml);
    wheel_update(&mr);
    dual_wheels_correction();
    xSemaphoreGive(xStateMutex);
  }

  wheel_setpwm(mr.correctedpwm, RIGHT);
  wheel_setpwm(ml.correctedpwm, LEFT);
  // Serial.printf("%d %d : %ld | %d %d : %ld\n", ldir, ltpwm, ml.ticksAim - ml.ticksCounter, rdir, rtpwm, mr.ticksAim - mr.ticksCounter);
  // Serial.printf("%ld %ld\n", ml.lastUpdateTicksDiff, mr.lastUpdateTicksDiff);
}

void wheels_setpoint(const float dist, const enum side_t side) {
  DCMotor *m = (side == LEFT) ? &ml : &mr;
  m->ticksAim = distance_to_ticks(dist);
  Serial.printf("%ld %ld\n", ml.ticksAim, mr.ticksAim);
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
