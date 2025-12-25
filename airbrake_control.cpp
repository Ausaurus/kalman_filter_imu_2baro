#include <Arduino.h>
#include <SD.h>
#include <Servo.h>

typedef struct {
    double Kp;
    double Kd;
    double setpoint;
    double prev_error;
    double control_output;
    double min_output;
    double max_output;
    double delta_t;
} PDController;

PDController brake_controller;
Servo myServo;

const double dt = 0.01;             // 10 ms sampling time
unsigned long last_compute = 0;
const float servo_speed = 500;
const float servo_pin = 9;


// CSV storage
const int MAX_ROWS = 5000;
float timeArr[MAX_ROWS];
float altitudeArr[MAX_ROWS];
float velocityArr[MAX_ROWS];
int dataCount = 0;

void PD_Init(PDController *pd, double Kp, double Kd,
             double min_out, double max_out, double dt);

double PD_Compute(PDController *pd, double actual_value);
double read_alt();

// -------------------------------------------
// Read CSV from SD card
// -------------------------------------------
void loadCSV() {
    File file = SD.open("flight.csv");
    if (!file) {
        Serial.println("Failed to open CSV!");
        return;
    }

    Serial.println("Loading CSV...");

    String line;
    bool skipHeader = true;

    while (file.available() && dataCount < MAX_ROWS) {
        line = file.readStringUntil('\n');

        if (skipHeader) {  
            skipHeader = false;   // skip first line
            continue;
        }

        // Parse CSV line: time, altitude, velocity
        int comma1 = line.indexOf(',');
        int comma2 = line.indexOf(',', comma1 + 1);

        if (comma1 < 0 || comma2 < 0) continue;

        timeArr[dataCount]     = line.substring(0, comma1).toFloat();
        altitudeArr[dataCount] = line.substring(comma1 + 1, comma2).toFloat();
        velocityArr[dataCount] = line.substring(comma2 + 1).toFloat();

        dataCount++;
    }

    file.close();

    Serial.print("CSV loaded. Rows: ");
    Serial.println(dataCount);
}


// -------------------------------------------
// Setup
// -------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(1000);

    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD Card init failed!");
        return;
    }

    loadCSV();

    PD_Init(&brake_controller,
            0.5,        // Kp
            0.8,        // Kd
            0.374,      // min output
            0.98,       // max output
            dt);
    myServo.attach(servo_pin);
}


// -------------------------------------------
// Main Loop
// -------------------------------------------
void loop() {

    unsigned long currentMillis = millis();

    // Only start reading setpoints AFTER 4 seconds
    if (currentMillis < 4000) return;

    // Stop using setpoints after 25 seconds
    if (currentMillis > 25000) return;

    // Run PD every 10ms
    if (currentMillis - last_compute >= dt * 1000) {

        float t = (currentMillis - 4000) / 1000.0;  // relative time after 4s

        // Find closest row in CSV
        int idx = (int)(t / dt);
        if (idx >= dataCount) idx = dataCount - 1;

        // Set CSV altitude as setpoint
        brake_controller.setpoint = altitudeArr[idx];

        // Read actual altitude (your function)
        double actual_alt = read_alt();

        // Compute PD
        double output = PD_Compute(&brake_controller, actual_alt);

        myServo.write(output);

        // Print for debugging
        Serial.print("t=");
        Serial.print(t, 3);
        Serial.print("  Setpoint=");
        Serial.print(brake_controller.setpoint);
        Serial.print("  Actual=");
        Serial.print(actual_alt);
        Serial.print("  Output=");
        Serial.println(output);

        last_compute = currentMillis;
    }

}


// -------------------------------------------
// PD Controller functions
// -------------------------------------------
void PD_Init(PDController *pd, double Kp, double Kd,
             double min_out, double max_out, double dt) {

    pd->Kp = Kp;
    pd->Kd = Kd;
    pd->min_output = min_out;
    pd->max_output = max_out;
    pd->delta_t = dt;

    pd->prev_error = 0;
    pd->control_output = 0;
    pd->setpoint = 0;
}

double PD_Compute(PDController *pd, double actual_value) {
    double error = pd->setpoint - actual_value;
    double P = pd->Kp * error;
    double D = pd->Kd * (error - pd->prev_error) / pd->delta_t;

    pd->control_output = P + D;

    if (pd->control_output > pd->max_output) pd->control_output = pd->max_output;
    if (pd->control_output < pd->min_output) pd->control_output = pd->min_output;

    pd->prev_error = error;

    double angle = pd->control_output * 180;
    return angle;
}

double read_alt() {
    return 0;
}
