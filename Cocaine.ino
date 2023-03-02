// I didn't get paid for this.

#include <EEPROM.h>

#define M1_PH 2
#define M1_EN 3

#define M2_PH 10
#define M2_EN 11

#define BUZZER 9

// 1 and 16 aren't connected
//               2   3   4   5   6   7   8   9   10 11 12 13 14 15
constexpr uint8_t sensors[] = { /*4,*/ 16, 17, 18, 19, 12, 13, 14, 8, 7, 6, 5, 15/*, 1*/ };
constexpr size_t sensor_count = sizeof(sensors) / sizeof(*sensors);

struct Calibration {
  uint16_t white[sensor_count];
  uint16_t black[sensor_count];
} cal;

uint16_t sensor_values[sensor_count];

void setup() {
  // Serial.begin(115200);

  pinMode(M1_PH, OUTPUT); // M1 PH
  pinMode(M1_EN, OUTPUT); // M1 EN

  pinMode(M2_PH, OUTPUT); // M2 PH
  pinMode(M2_EN, OUTPUT); // M2 EN

  pinMode(BUZZER, OUTPUT); // BUZZER

  EEPROM.get(0, cal);
}

bool button() {
  return analogRead(A7) > 768;
}

// Yoinked from https://github.com/pololu/qtr-sensors-arduino
void read() {
  for (uint8_t i = 0; i < sensor_count; i++) {
    sensor_values[i] = 2500;
    // make sensor line an output (drives low briefly, but doesn't matter)
    pinMode(sensors[i], OUTPUT);
    // drive sensor line high
    digitalWrite(sensors[i], HIGH);
  }

  delayMicroseconds(10); // charge lines for 10 us

  {
    // disable interrupts so we can switch all the pins as close to the same
    // time as possible
    noInterrupts();

    // record start time before the first sensor is switched to input
    // (similarly, time is checked before the first sensor is read in the
    // loop below)
    uint32_t startTime = micros();
    uint16_t time = 0;

    for (uint8_t i = 0; i < sensor_count; i++) {
      // make sensor line an input (should also ensure pull-up is disabled)
      pinMode(sensors[i], INPUT);
    }

    interrupts(); // re-enable

    while (time < 2500) {
      // disable interrupts so we can read all the pins as close to the same
      // time as possible
      noInterrupts();

      time = micros() - startTime;
      for (uint8_t i = 0; i < sensor_count; i++)
      {
        if ((digitalRead(sensors[i]) == LOW) && (time < sensor_values[i]))
        {
          // record the first time the line reads low
          sensor_values[i] = time;
        }
      }

      interrupts(); // re-enable
    }
  }
  return;
}

void calibrate(uint16_t *target_values, bool min, uint16_t loops = 1600) {
  for (uint16_t i = 0; i < loops; i++) {
    read();

    if (i == 0) {
      for (uint16_t j = 0; j < sensor_count; j++) {
        target_values[j] = sensor_values[j];
      }
    } else {
      for (uint16_t j = 0; j < sensor_count; j++) {
        if (min ? (target_values[j] > sensor_values[j]) : (target_values[j] < sensor_values[j])) {
          target_values[j] = sensor_values[j];
        }
      }
    }
  }
}

void beep(uint16_t length) {
  analogWrite(BUZZER, 128);
  delay(length);
  analogWrite(BUZZER, 0);
  delay(length / 3);
}

void runCalibration() {
  beep(200);
  while (!button());
  beep(100);

  calibrate(cal.white, true);

  beep(70);
  beep(200);
  while (!button());
  beep(100);

  calibrate(cal.black, false);

  beep(100);
  EEPROM.put(0, cal);

  beep(300);
}

constexpr uint16_t base_speed = 140;
constexpr uint16_t max_speed = 255;
float scale = 0.0f;

void motors(float control) {
  float left = float(base_speed + control) * scale;
  float right = float(base_speed - control) * scale;

  if (left > 255.0f) {
    right -= left - 255.0f;
    left = 255.0f;
  }
  
  if (right > 255.0f) {
    left -= right - 255.0f;
    right = 255.0f;
  }

  bool right_dir = false;
  if (right < 0.0f) {
    right = -right;
    right_dir = true;
  }

  bool left_dir = false;
  if (left < 0.0f) {
    left = -left;
    left_dir = true;
  }

   // 1 BACKWARD 0 FORWARD
  digitalWrite(M1_PH, left_dir);
  digitalWrite(M2_PH, right_dir);

  // POWER 0-255
  analogWrite(M1_EN, left);
  analogWrite(M2_EN, right);
}

bool running = false;

bool last_pressed = false;
uint64_t button_time = 0;
uint64_t black_time = 0;

uint64_t transition_time = 0;
bool transition = false;
bool transition_start = true;

float error = 0.0f, last_error = 0.0f, total_error = 0.0f;

constexpr float Kp = 29.0f;
constexpr float Ki = 0.4f;
constexpr float Kd = 11.5f;

float control = 0.0f;
constexpr float max_control = 8.0f;

void loop() {
  bool pressed = button();
  if (pressed && !last_pressed) {
    last_pressed = true;
    button_time = millis();
  } else if (!pressed && last_pressed) {
    uint64_t length = millis() - button_time;
    if (length > 75) {
      last_pressed = false;
      if (length < 1000) {
        if(running = !running) {
          scale = 0.0f;
          transition = true;
          transition_start = true;
          transition_time = millis();
        }
      } else {
        runCalibration();
      }
    }
  }
  
  if (!running) {
    analogWrite(M1_EN, 0);
    analogWrite(M2_EN, 0);
    analogWrite(BUZZER, 0);
    return;
  }

  if (transition) {
    uint64_t m = millis();
    scale += float(m - transition_time) * (transition_start ? 0.00075f : -0.00075f);
    transition_time = m;

    if (scale > 1.0f) {
      scale = 1.0f;
      transition = false;
    }

    if (scale < 0.0f) {
      scale = 0.0f;
      transition = false;
      running = false;
    }    
  }

  bool has_line = false, all_black = true;
  float sum = 0.0f, avg = 0.0f;

  read();
  for (uint8_t i = 0; i < sensor_count; i++) {
    float white = (float)cal.white[i];
    float black = (float)cal.black[i];
    float current = (float)sensor_values[i];
    float v = (current - white) / (black - white);

    if (v < 0.23f) {
      v = 0.0f;
    }

    if (v > 0.40f) {
      has_line = true;
    } else {
      all_black = false;
    }

    if (v > 1.00f) v = 1.0f;

    sum += v;
    avg += v * (float)i;
  }

  if (all_black && !transition) {
    scale = 1.0f;
    transition = true;
    transition_start = false;
    transition_time = millis();
  }  

  last_error = error;
  if (has_line) {
    error = (float)sensor_count / 2.0f - avg / sum;
    total_error = constrain(total_error + error, -max_control, max_control);
    control = Kp * error + Ki * total_error + Kd * (error - last_error);
    analogWrite(BUZZER, 0);
  }

  uint16_t delta = millis() % (has_line ? 500 : 100);
  analogWrite(BUZZER, (delta > 50) ? 0 : 128);

  motors(control);
  
  delay(2);
}
