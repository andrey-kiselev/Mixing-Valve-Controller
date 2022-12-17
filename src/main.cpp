/**
 * @file main.cpp
 * @author Andrey Kiselev (andrey.kiselev@live.com)
 * @brief
 * @version 0.1
 * @date 2022-12-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <Arduino.h>
#include <ArduinoHA.h>
#include <Arduino_JSON.h>
#include <ESP8266WiFi.h>
#include <DallasTemperature.h>
#include <OneWire.h>
#include <QuickPID.h>

#define BROKER_ADDR IPAddress(192, 168, 100, 1)
#define WIFI_SSID "AndyNet-IoT"
#define WIFI_PASSWORD "iotfetthehe"

WiFiClient client;
HADevice device;
HAMqtt mqtt(client, device);

// By default HAHVAC supports only reporting of the temperature.
// You can enable feature you need using the second argument of the constructor.
// Please check the documentation of the HAHVAC class.
HAHVAC hvac(
    "Norah-3000",
    HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature);

// Motor driver
const int pwmMotorA = D1;
const int pwmMotorB = D2;
const int dirMotorA = D3;
const int dirMotorB = D4;

const int tempSensorData = D5;
OneWire oneWire(tempSensorData);
DallasTemperature sensors(&oneWire);

const int encoderPinA = D6;
const int encoderPinB = D7;
volatile long encoderTicks = 0;

// PID controller for temperature control
float targetTemperature, currentTemperature, mixingValvePosition;
float Kp = 0.8, Ki = 1.6, Kd = 0.1;

struct {
  float min;
  float max;
  bool isCalibrated;
} mixingValveRange;

QuickPID myPID(&currentTemperature, &mixingValvePosition, &targetTemperature, Kp, Ki, Kd, QuickPID::Action::direct);

// PID for motor driver
float motorSetpoint, motorInput, motorOutput;
float mKp = 2.0, mKi = 5.0, mKd = 0.4;
float motorPIDMaxValue = 1023.0f;
QuickPID motorPID(&motorInput, &motorOutput, &motorSetpoint, mKp, mKi, mKd, QuickPID::Action::direct);

ICACHE_RAM_ATTR void EncoderCallback() {
    if (digitalRead(encoderPinB) == 1)
        encoderTicks++;
    else
        encoderTicks--;
}

/* events */

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC *sender) {
    float temperatureFloat = temperature.toFloat();

    Serial.print("Target temperature: ");
    Serial.println(temperatureFloat);

    targetTemperature = temperature.toFloat();

    sender->setTargetTemperature(temperature);  // report target temperature back to the HA panel
}

void onPowerCommand(bool state, HAHVAC *sender) {
    if (state) {
        Serial.println("Power on");
    } else {
        Serial.println("Power off");
    }
}

void onModeCommand(HAHVAC::Mode mode, HAHVAC *sender) {
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        Serial.println("off");
    } else if (mode == HAHVAC::AutoMode) {
        Serial.println("auto");
    } else if (mode == HAHVAC::CoolMode) {
        Serial.println("cool");
    } else if (mode == HAHVAC::HeatMode) {
        Serial.println("heat");
    } else if (mode == HAHVAC::DryMode) {
        Serial.println("dry");
    } else if (mode == HAHVAC::FanOnlyMode) {
        Serial.println("fan only");
    }

    sender->setMode(mode);  // report mode back to the HA panel
}

/* helpers */

// set motor target speed
void motorSetSpeed(float speed) {
    if (abs(speed) < 50) {
        // stop
        digitalWrite(dirMotorA, LOW);
        analogWrite(pwmMotorA, 0);
        return;
    }
    if (speed > 0) {
        digitalWrite(dirMotorA, HIGH);
    } else {
        digitalWrite(dirMotorA, LOW);
    }
    analogWrite(pwmMotorA, abs(int(speed)));
}

// spin the motor in different directions to find the range
// should only run after PID controller is initialized!
// motor PID has to be defined at this point.
void doValveCalibration() {
    // the total span of the valve is about 90 degrees (4-way valve).
    // the gear ratio is 16:48 (sprockets)  - the motor shaft range 270 degrees
    // TODO: the motor gear ratio 1:131? 1:100?
    // encoder is 64 CPR.
    // full valve range in CPR: 64 * (100/1) * (48/16) * (1/4) = 4800
    // so, we spin the motor 20% more than that to make sure we hit the limit

    long fullRangeTicks = 4800;

    uint32_t timeout = 25000000;  // 25 sec timeout

    uint32_t timeStart = micros();
    uint32_t timeLast = micros();

    // create an array for speed measurements and fill it with meaningfull numbers
    int windowSize = 20;
    float speedMeasurements[windowSize];
    int speedMeasurementsIdx = 0;
    for (int i = 0; i < windowSize; i++)
        speedMeasurements[i] = 2.0f;
    float speedPIDCurrentSpeed = 0.0f;

    long lastMotorTicks = encoderTicks;

    motorSetpoint = -1.0f * fullRangeTicks * 1.5f;

    while (micros() - timeStart < timeout) {
        yield();

        motorInput = encoderTicks;

        if (motorPID.Compute()) {
            motorSetSpeed(motorOutput);
            speedPIDCurrentSpeed = ((float)(encoderTicks - lastMotorTicks)) / ((float)(micros() - timeLast) / 1000);
            speedMeasurements[speedMeasurementsIdx] = speedPIDCurrentSpeed;
            if (speedMeasurementsIdx++ > windowSize)
                speedMeasurementsIdx = 0;

            // break if the average speed is close to 0
            float avgSpeed = 0.0f;
            for (int i = 0; i < windowSize; i++)
                avgSpeed += speedMeasurements[i];
            if ((abs(avgSpeed) < 0.00005f) && (abs(motorOutput) == motorPIDMaxValue)) {
                Serial.println("HIT!");
                motorSetSpeed(0.0f);
                break;
            }

            lastMotorTicks = encoderTicks;
            timeLast = micros();

            Serial.print(speedPIDCurrentSpeed, 6);
            Serial.print(" ");
            Serial.println(motorInput);
        }
    }

    // speedPIDCurrentSpeed = 0;
    // speedPIDMotorOut = 0;

    // hack!
    // encoderTicks = 0;

    mixingValveRange.min = encoderTicks;
    Serial.println(mixingValveRange.min);

    // going right
    motorSetpoint = fullRangeTicks * 1.5f;
    for (int i = 0; i < windowSize; i++)
        speedMeasurements[i] = 2.0f;

    while (micros() - timeStart < timeout) {
        yield();

        motorInput = encoderTicks;

        if (motorPID.Compute()) {
            motorSetSpeed(motorOutput);
            speedPIDCurrentSpeed = ((float)(encoderTicks - lastMotorTicks)) / ((float)(micros() - timeLast) / 1000);
            speedMeasurements[speedMeasurementsIdx] = speedPIDCurrentSpeed;
            if (speedMeasurementsIdx++ > windowSize)
                speedMeasurementsIdx = 0;

            // break if the average speed is close to 0
            float avgSpeed = 0.0f;
            for (int i = 0; i < windowSize; i++)
                avgSpeed += speedMeasurements[i];
            if ((abs(avgSpeed) < 0.00005f) && (abs(motorOutput) == motorPIDMaxValue)) {
                Serial.println("HIT!");
                motorSetSpeed(0.0f);
                break;
            }

            lastMotorTicks = encoderTicks;
            timeLast = micros();

            Serial.print(speedPIDCurrentSpeed, 6);
            Serial.print(" ");
            Serial.println(motorInput);
        }
    }

    mixingValveRange.max = encoderTicks;
    Serial.println(mixingValveRange.max);
}

void setup() {
    Serial.begin(115200);
    Serial.println("Starting...");

    delay(2000);

    mixingValveRange.min = -1000.0;
    mixingValveRange.max = 1000.0;
    mixingValveRange.isCalibrated = false;

    // Unique ID must be set!
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    device.setUniqueId(mac, sizeof(mac));

    // connect to wifi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);  // waiting for the connection
    }
    Serial.println();
    Serial.println("Connected to the network");

    // set device's details (optional)
    device.setName("NodeMCU");
    device.setSoftwareVersion("1.0.0");

    // assign callbacks (optional)
    hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    hvac.onPowerCommand(onPowerCommand);
    // hvac.onModeCommand(onModeCommand);

    // configure HVAC (optional)
    hvac.setName("Central Heating");
    hvac.setMinTemp(10);
    hvac.setMaxTemp(30);
    hvac.setTempStep(0.5);

    // You can set retain flag for the HA commands
    // hvac.setRetain(true);

    // You can choose which modes should be available in the HA panel
    // hvac.setModes(HAHVAC::OffMode | HAHVAC::CoolMode);

    mqtt.begin(BROKER_ADDR, "norrah", "norrah");

    pinMode(pwmMotorA, OUTPUT);
    pinMode(pwmMotorB, OUTPUT);
    pinMode(dirMotorA, OUTPUT);
    pinMode(dirMotorB, OUTPUT);

    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    attachInterrupt(encoderPinA, EncoderCallback, RISING);

    pinMode(tempSensorData, INPUT_PULLUP);

    sensors.begin();

    // Setpoint = 28.0f;
    myPID.SetOutputLimits(mixingValveRange.min, mixingValveRange.max);
    myPID.SetSampleTimeUs(10000000);
    myPID.SetMode(QuickPID::Control::automatic);

    motorPID.SetOutputLimits(-1.0 * motorPIDMaxValue, motorPIDMaxValue);
    motorPID.SetMode(QuickPID::Control::automatic);
    motorPID.SetProportionalMode(QuickPID::pMode::pOnMeas);

    sensors.setWaitForConversion(false);
    // this needs to be done first time
    sensors.requestTemperatures();

    doValveCalibration();
}

void loop() {
    // if (tempUpdateCounter > tempUpdateTicks) {
    //   sensors.requestTemperatures();
    //   for(int i=0; i<sensors.getDeviceCount(); i++){
    //     float temperatureC = sensors.getTempCByIndex(i);
    //     if(i==0) currentTemperature = temperatureC;
    //     Serial.print(i);
    //     Serial.print(":");
    //     Serial.print(temperatureC);
    //     Serial.print("ºC ");
    //   }

    mqtt.loop();

    if (sensors.isConversionComplete() == 1) {
        float temperatureC = sensors.getTempCByIndex(0);
        sensors.requestTemperatures();
        currentTemperature = temperatureC;

        hvac.setCurrentTemperature(temperatureC);
        // Serial.print("Temperature: ");
        // Serial.println(temperatureC);
    }

    if (myPID.Compute()) {
        motorSetpoint = mixingValvePosition;
        Serial.print("Target: ");
        Serial.print(targetTemperature);
        Serial.print(" Current: ");
        Serial.print(currentTemperature);
        Serial.print(" Control: ");
        Serial.println(mixingValvePosition);
    }

    motorInput = encoderTicks;
    if (motorPID.Compute()) {
        motorSetSpeed(motorOutput);
        //   Serial.print(" ");
        //   Serial.print(motorInput);
    }
}