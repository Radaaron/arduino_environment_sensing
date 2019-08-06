// returns absolute attitude angle (float), wind speed (float), wind direction(integer), geolocation (float), and altitude (float)

#include <Wire.h>
#include <TimerOne.h> // Timer Interrupt set to 2 second for read sensors
#include <math.h>
#include <QMC5883L.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>

SoftwareSerial gps_serial(2, 3);
SoftwareSerial hc12_serial(8, 9);
#define wind_sensor_pin (4)
#define wind_vane_pin (A3)

int gyro_x, gyro_y, gyro_z, cal_direction;
long acc_x, acc_y, acc_z, gyro_x_cal, gyro_y_cal, gyro_z_cal, gps_age;
float angle_pitch_output, angle_roll_output, dt, wind_speed, heading, abs_angle_output, offset, gps_lat, gps_long, gps_altitude;
volatile boolean angles_set, sample_required;
volatile unsigned int timer_count;
volatile unsigned long rotations, contact_bounce_time;
QMC5883L compass;
TinyGPS gps;
String hc12_output;

void setup() {
  Wire.begin(); // join i2c as master
  Serial.begin(19200);
  hc12_serial.begin(9600);
  gps_serial.begin(9600);

  // set up compass
  compass.init();

  // set up mpu6050
  Wire.beginTransmission(0x68); // address of mpu-650
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();
  // configure the accelerometer
  Wire.beginTransmission(0x68);
  Wire.write(0x1C);
  Wire.write(0x10);  // (+/-8g)
  Wire.endTransmission();
  // configure the gyro
  Wire.beginTransmission(0x68);
  Wire.write(0x1B);
  Wire.write(0x08); // 500 dps
  Wire.endTransmission();

  // read average gyro readings from 1000
  for (int cal_int = 0; cal_int < 1000 ; cal_int ++) {
    // read from mpu6050 registers
    Wire.beginTransmission(0x68);
    // starting register
    Wire.write(0x3B);
    Wire.endTransmission();
    // request 14 bytes
    Wire.requestFrom(0x68, 14);
    while (Wire.available() < 14);
    // byte shift -> bitwise or byte => 16 bits
    acc_x = Wire.read() << 8 | Wire.read();
    acc_y = Wire.read() << 8 | Wire.read();
    acc_z = Wire.read() << 8 | Wire.read();
    int temperature = Wire.read() << 8 | Wire.read(); // never used
    gyro_x = Wire.read() << 8 | Wire.read();
    gyro_y = Wire.read() << 8 | Wire.read();
    gyro_z = Wire.read() << 8 | Wire.read();
    gyro_x_cal += gyro_x;
    gyro_y_cal += gyro_y;
    gyro_z_cal += gyro_z;
    delay(4); // 4ms / 250Hz
  }

  // divide by 1000 to get offset for calibration
  gyro_x_cal /= 1000;
  gyro_y_cal /= 1000;
  gyro_z_cal /= 1000;
  dt = 0.004; // 4ms

  //set up va
  timer_count = 0;
  rotations = 0; // set rotations to 0 ready for calculations
  pinMode(wind_sensor_pin, INPUT);
  attachInterrupt(digitalPinToInterrupt(wind_sensor_pin), isr_rotation, FALLING);
  // setup the timer interupt
  Timer1.initialize(500000); // interrupt every 2.5 seconds
  Timer1.attachInterrupt(isr_timer);
}

void loop() {
  pollMPU6050();
  pollCompass();
  pollVA();
  pollGPS(10);
  // send to hc12
  hc12_output = String(abs_angle_output, 6) + " - " + String(heading, 6) + " - " + String(cal_direction) + " - " + String(wind_speed, 6) + " - " + String(gps_lat, 6) + " - " + String(gps_long, 6) + " - " + String(gps_altitude, 6) + "\n";
  hc12_serial.print(hc12_output);
  delay(4); // 4ms or 250Hz
}

void pollMPU6050() {
  // read from mpu6050 registers
  Wire.beginTransmission(0x68);
  // starting register
  Wire.write(0x3B);
  Wire.endTransmission();
  // request 14 bytes
  Wire.requestFrom(0x68, 14);
  while (Wire.available() < 14);
  // byte shift -> bitwise or byte => 16 bits
  acc_x = Wire.read() << 8 | Wire.read();
  acc_y = Wire.read() << 8 | Wire.read();
  acc_z = Wire.read() << 8 | Wire.read();
  int temperature = Wire.read() << 8 | Wire.read(); // never used
  gyro_x = Wire.read() << 8 | Wire.read();
  gyro_y = Wire.read() << 8 | Wire.read();
  gyro_z = Wire.read() << 8 | Wire.read();

  // subtract offset calibration value from the raw gyros value
  gyro_x -= gyro_x_cal;
  gyro_y -= gyro_y_cal;
  gyro_z -= gyro_z_cal;

  // gyro traveled angle calculations
  float anglePitchGyro = (gyro_x / 65.5);
  float angleRollGyro = (gyro_y / 65.5);

  // roll <-> pitch angle transfer using yaw
  // 0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
  anglePitchGyro += angleRollGyro * sin(gyro_z * 0.000001066);
  angleRollGyro -= anglePitchGyro * sin(gyro_z * 0.000001066);

  // convert accel values to float
  acc_x = (float) acc_x;
  acc_y = (float) acc_y;
  acc_z = (float) acc_z;

  // traveled angle calculations
  float anglePitchAcc = atan(acc_x / sqrt(sq(acc_y) + sq(acc_z))) * 180 / M_PI;
  float angleRollAcc = atan(acc_y / sqrt(sq(acc_x) + sq(acc_z))) * 180 / M_PI;

  // TODO: change values for accel calibration if needed
  anglePitchAcc -= 0.0;
  angleRollAcc -= 0.0;

  // gyro drift correction using complementary filter
  if (angles_set) {
    // already started, just adjust for gyro drift using accel
    angle_pitch_output = (angle_pitch_output + anglePitchGyro * dt) * 0.96 + anglePitchAcc * 0.04;
    angle_roll_output = (angle_roll_output + angleRollGyro * dt) * 0.96 + angleRollAcc * 0.04;
  }
  else {
    // first start, set gyro angles equal to accel angles
    angle_pitch_output = anglePitchAcc;
    angle_roll_output = angleRollAcc;
    angles_set = true;
  }

  abs_angle_output = abs(angle_pitch_output) + abs(angle_roll_output); // calculate absolute angle
}

void pollCompass() {
  int compassX, compassY, compassZ;
  compass.read(&compassX, &compassY, &compassZ);

  // calculate heading when the magnetometer is level, then correct for signs of axis.
  // atan2() automatically checks the correct formula taking care of quadrant
  heading = atan2(compassY, compassX);

  // get declination angle to offset heading to true north, found at: http://www.magnetic-declination.com/
  float declinationAngle = 2.25;
  heading += declinationAngle;

  // correct for when signs are reversed.
  if (heading < 0)
    heading += 2 * PI;

  // check for wrap due to addition of declination.
  if (heading > 2 * PI)
    heading -= 2 * PI;

  // convert radians to degrees for va north reference
  heading = heading * 180 / M_PI;
}

void pollVA() {
  // wind direction
  int vaneValue = analogRead(wind_vane_pin);
  int direction = map(vaneValue, 0, 1023, 0, 359);
  cal_direction = direction;

  if (cal_direction > 360) {
    cal_direction = cal_direction - 360;
  }

  if (cal_direction < 0) {
    cal_direction = cal_direction + 360;
  }

  // wind speed
  if (sample_required) {
    // convert to mp/h using the formula V=P(2.25/T)
    // V = P(2.25/2.5) = P * 0.9
    wind_speed = rotations * 0.9;
    rotations = 0; // Reset count for next sample
    sample_required = false;
  }
}

// isr handler for timer interrupt
void isr_timer() {
  timer_count++;
  if (timer_count == 6) {
    sample_required = true;
    timer_count = 0;
  }
}

// function that the interrupt calls to increment the rotation count
void isr_rotation() {
  if ((millis() - contact_bounce_time) > 15 ) { // debounce the switch contact.
    rotations++;
    contact_bounce_time = millis();
  }
}

// poll and process gps data
static void pollGPS(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (gps_serial.available()) {
      gps.encode(gps_serial.read());
    }
    // process data
    gps.f_get_position(&gps_lat, &gps_long, &gps_age);
    gps_altitude = gps.f_altitude();
  } while (millis() - start < ms);
}
