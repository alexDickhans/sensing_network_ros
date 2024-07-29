#include <CapacitiveSensorR4.h>

CapacitiveSensor sensor(2, 4);

struct _TouchPointRange {
  uint16_t index;
  uint16_t min;
  uint16_t max;
} typedef TouchPointRange;

TouchPointRange* touchRanges;
int numTouchPoints;
int startIndex;
int samples;

void setup() {
  Serial.begin(115200);

  Serial.setTimeout(80000);

  while (!Serial) {
    delay(10);
  }

  Serial.println("How many touch points?");

  numTouchPoints = atoi(Serial.readStringUntil('\n').c_str());

  if (numTouchPoints < 1) {
    Serial.println("Too few touch points, restart");
    ESP.restart();
  }

  Serial.printf("You selected %d touch points.\n\nWhat should the start index be?\n", numTouchPoints);

  startIndex = atoi(Serial.readStringUntil('\n').c_str());

  if (startIndex < 0) {
    Serial.println("Index must be positive, restart\n\n");
    ESP.restart();
  }

  Serial.printf("You selected %d start index.\n\nHow many samples should it take?\n", startIndex);

  samples = atoi(Serial.readStringUntil('\n').c_str());

  if (samples < 5) {
    Serial.println("Sample count must be 5 or greater, restart\n\n");
    ESP.restart();
  }

  Serial.printf("You selected %d samples.\n\n", samples);

  touchRanges = (TouchPointRange*) calloc(numTouchPoints, sizeof(TouchPointRange));

  for (size_t i = 0; i < numTouchPoints; i++) {
    Serial.printf("Touch point %d for 5 seconds\n ", startIndex +i);
    delay(3000);

    uint32_t startTime = millis();

    uint32_t sample = sensor.capacitiveSensorRaw(samples);

    uint32_t highValue = sample;
    uint32_t lowValue = sample;

    while (millis() - startTime < 3000) {
      sample = sensor.capacitiveSensorRaw(samples);

      if (sample == -2) {
        // nothing
      } else if (sample > highValue) {
        highValue = sample;
      } else if (sample < lowValue) {
        lowValue = sample;
      }
    }

    Serial.printf("High value: %d, Low value: %d\n", highValue, lowValue);

    touchRanges[i] = {i + startIndex, lowValue, highValue};
  }

  Serial.print("\n\n\n\TouchPointRange ranges[] = {\n");

  for (size_t i = 0; i < numTouchPoints; i++) {
    Serial.printf("{%d, %d, %d},\n", touchRanges[i].index, touchRanges[i].min, touchRanges[i].max);
  }

  Serial.print("}\n");

  delay(5000);
}

void loop() {
  auto reading = sensor.capacitiveSensorRaw(samples);

  Serial.println(reading);

  for (size_t i = 0; i < numTouchPoints; i++) {
    bool touchReading = reading > touchRanges[i].min && reading < touchRanges[i].max;
    Serial.printf("%d, %d\n", touchRanges[i].index, touchReading);
  }
}
