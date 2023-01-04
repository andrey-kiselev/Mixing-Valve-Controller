/**
 * @file main.cpp
 * @author Andrey Kiselev (andrey.kiselev@live.com)
 * @brief
 * @version 0.1
 * @date 2022-12-17
 *
 * Underscore used for global variables naming!
 *
 * @copyright Copyright (c) 2022
 */

#include <Arduino.h>
#include <ArduinoHA.h>
#include <DallasTemperature.h>
#include <ESP8266WiFi.h>
#include <OneWire.h>
#include <QuickPID.h>

#define BROKER_ADDR IPAddress(192, 168, 100, 1)
#define WIFI_SSID "AndyNet-IoT"
#define WIFI_PASSWORD "iotfetthehe"

WiFiClient _client;
HADevice _device;
HAMqtt _mqtt(_client, _device);

/**
 * @brief Various devices that will be available in HA control panel.
 *
 */
HAHVAC _hvac("Norah-3000", HAHVAC::TargetTemperatureFeature | HAHVAC::PowerFeature | HAHVAC::ModesFeature);
HASensorNumber _temperatureSensorReturn("temperatureSensorReturn", HASensorNumber::PrecisionP1);
HANumber _valveDirectPosition("valvePosition");

/**
 * @brief Variables and objects for getting 1-Wire sensor readings.
 */
const int _tempSensorData = D5;
OneWire _oneWire(_tempSensorData);
DallasTemperature _sensors(&_oneWire);

/**
 * @brief Variables and objects for valve motor driver and PID controller.
 *
 */
const int _pwmMotorA = D1;
const int _pwmMotorB = D2;
const int _dirMotorA = D3;
const int _dirMotorB = D4;

const int _encoderPinA = D6;
const int _encoderPinB = D7;
volatile long _encoderTicks = 0;

struct {
    float min;
    float max;
    bool isCalibrated;
} _mixingValveRange;

float _valveMotorSetpoint, _valveMotorCurrentPosition, _valveMotorPWMOutput;
float _valveMotorKp = 2.0, _valveMotorKi = 5.0, _valveMotorKd = 0.4;
float _motorPIDMaxValue = 1023.0f;
QuickPID _valvePIDController(&_valveMotorCurrentPosition, &_valveMotorPWMOutput, &_valveMotorSetpoint, _valveMotorKp, _valveMotorKi, _valveMotorKd, QuickPID::Action::direct);

IRAM_ATTR /*ICACHE_RAM_ATTR*/ void EncoderCallback() {
    if (digitalRead(_encoderPinB) == 1)
        _encoderTicks++;
    else
        _encoderTicks--;
}

/**
 * @brief PID controller for temperature.
 *
 */
float _targetTemperature, _currentTemperature, _mixingValvePosition;
float _temperatureKp = 0.8, _temperatureKi = 1.6, _temperatureKd = 0.1;
const unsigned long _temperaturePIDTimeStep = 10000000;
QuickPID _temperaturePIDController(&_currentTemperature, &_mixingValvePosition, &_targetTemperature, _temperatureKp, _temperatureKi, _temperatureKd, QuickPID::Action::direct);

/* helpers */

// set motor target speed
void motorSetSpeed(float speed) {
    if (abs(speed) < 50) {
        // stop
        digitalWrite(_dirMotorA, LOW);
        analogWrite(_pwmMotorA, 0);
        return;
    }
    if (speed > 0) {
        digitalWrite(_dirMotorA, HIGH);
    } else {
        digitalWrite(_dirMotorA, LOW);
    }
    analogWrite(_pwmMotorA, abs(int(speed)));
}

/**
 * @brief Set desired valve motor position (absolute number of ticks) with timeout (millis). Returns set position.
 *
 * @param position
 * @param timeout
 * @return long
 */
long valveMotorSetPosition(long position, unsigned long timeout) {
    if (abs(_valveMotorCurrentPosition - (float)position) < 50) {
        return _valveMotorCurrentPosition;
    }

    _valveMotorSetpoint = (float)position;

    unsigned long timeStart = millis();
    unsigned long timeLast = millis();

    int windowSize = 10;
    float speedMeasurements[windowSize];
    int speedMeasurementsIdx = 0;

    long valveMotorLastPosition = _encoderTicks;

    _valvePIDController.SetMode(QuickPID::Control::automatic);

    while (millis() - timeStart < timeout) {
        yield();
        _mqtt.loop();
        _valveMotorCurrentPosition = _encoderTicks;

        // break if we are very close to the desired position
        // if (abs(_valveMotorCurrentPosition - (float)position) < 50) break;

        if (_valvePIDController.Compute()) {
            motorSetSpeed(_valveMotorPWMOutput);
            speedMeasurements[speedMeasurementsIdx % windowSize] = ((float)(_encoderTicks - valveMotorLastPosition)) / ((float)(micros() - timeLast) / 1000);

            // break if the average speed is close to 0
            float avgSpeed = 0.0f;
            if (speedMeasurementsIdx < windowSize) {
                for (int i = 0; i <= speedMeasurementsIdx; i++) avgSpeed += speedMeasurements[i] / (speedMeasurementsIdx + 1);
            } else {
                for (int i = 0; i < windowSize; i++) avgSpeed += speedMeasurements[i] / windowSize;
            }

            if ((speedMeasurementsIdx > windowSize) && (abs(avgSpeed) < 0.005f) && (abs(_valveMotorPWMOutput) == _motorPIDMaxValue)) {
                break;
            }

            valveMotorLastPosition = _encoderTicks;
            timeLast = micros();
            speedMeasurementsIdx++;
        }
    }
    _valvePIDController.SetMode(QuickPID::Control::manual);
    motorSetSpeed(0.0f);
    return _valveMotorCurrentPosition;
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

    long fullRangeTicks = 4800;  // 4800;
    uint32_t timeout = 10000;    // 10 sec timeout

    _mixingValveRange.min = valveMotorSetPosition(-fullRangeTicks, timeout);
    Serial.println(_mixingValveRange.min);

    _mixingValveRange.max = valveMotorSetPosition(fullRangeTicks, timeout);

    Serial.println(_mixingValveRange.max);
    _mixingValveRange.isCalibrated = true;
}

/* events */
void onModeCommand(HAHVAC::Mode mode, HAHVAC *sender) {
    Serial.print("Mode: ");
    if (mode == HAHVAC::OffMode) {
        _temperaturePIDController.SetMode(QuickPID::Control::manual);
        valveMotorSetPosition(_mixingValveRange.min, 5000);
        _valvePIDController.SetMode(QuickPID::Control::manual);
        Serial.println("off");
    } else if (mode == HAHVAC::AutoMode) {
        _temperaturePIDController.SetMode(QuickPID::Control::automatic);
        _valvePIDController.SetMode(QuickPID::Control::automatic);
        Serial.println("auto");
        // ESP.restart();
    } else if (mode == HAHVAC::HeatMode) {
        _temperaturePIDController.SetMode(QuickPID::Control::manual);
        valveMotorSetPosition(_mixingValveRange.max, 5000);
        _valvePIDController.SetMode(QuickPID::Control::manual);
        Serial.println("heat");
    }
    sender->setMode(mode);  // report mode back to the HA panel
}

void onNumberCommand(HANumeric number, HANumber *sender) {
    if (!number.isSet()) {
        // the reset command was send by Home Assistant
        onModeCommand(HAHVAC::Mode::AutoMode, &_hvac);
    } else {
        _temperaturePIDController.SetMode(QuickPID::Control::manual);
        _hvac.setMode(HAHVAC::Mode::OffMode);

        // you can do whatever you want with the number as follows:
        float numberFloat = number.toFloat();
        valveMotorSetPosition(numberFloat, 10000);
    }
    sender->setState(number);  // report the selected option back to the HA panel
}

void onTargetTemperatureCommand(HANumeric temperature, HAHVAC *sender) {
    float temperatureFloat = temperature.toFloat();

    onModeCommand(HAHVAC::Mode::AutoMode, &_hvac);

    Serial.print("Target temperature: ");
    Serial.println(temperatureFloat);

    _targetTemperature = temperature.toFloat();
    sender->setTargetTemperature(temperature);  // report target temperature back to the HA panel
}

void onPowerCommand(bool state, HAHVAC *sender) {
    if (state) {
        Serial.println("Power on");
    } else {
        Serial.println("Power off");
    }
}

void setup() {
    Serial.begin(115200);

    // Unique ID must be set!
    byte mac[WL_MAC_ADDR_LENGTH];
    WiFi.macAddress(mac);
    _device.setUniqueId(mac, sizeof(mac));

    // TODO: Implement timeout for WiFi connection
    // connect to wifi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED) {
        Serial.print(".");
        delay(500);  // waiting for the connection
    }
    Serial.println();
    Serial.println("Connected to the network");

    _device.enableSharedAvailability();
    _device.enableLastWill();
    _device.setName("NodeMCU-Norrah3000");
    _device.setSoftwareVersion("1.0.2");

    // assign callbacks (optional)
    _hvac.onTargetTemperatureCommand(onTargetTemperatureCommand);
    _hvac.onPowerCommand(onPowerCommand);
    _hvac.onModeCommand(onModeCommand);

    // configure HVAC (optional)
    _hvac.setName("Central Heating");
    _hvac.setMinTemp(10);
    _hvac.setMaxTemp(30);
    _hvac.setTempStep(0.5);

    _hvac.setRetain(true);
    _hvac.setModes(HAHVAC::OffMode | HAHVAC::HeatMode | HAHVAC::AutoMode);

    pinMode(_tempSensorData, INPUT_PULLUP);
    _sensors.begin();
    _sensors.setWaitForConversion(false);
    // this needs to be done manually first time
    _sensors.requestTemperatures();

    for (int i = 0; i < _sensors.getDeviceCount(); i++) {
        float temperatureC = _sensors.getTempCByIndex(i);
        uint8_t address;
        _sensors.getAddress(&address, i);
        Serial.print(i);
        Serial.print(":");
        Serial.print(address);
        Serial.print(":");
        Serial.print(temperatureC);
        Serial.println("ºC ");
    }

    _temperatureSensorReturn.setIcon("mdi:thermometer");
    _temperatureSensorReturn.setName("Heating Return Temperature");
    _temperatureSensorReturn.setUnitOfMeasurement("℃");

    pinMode(_pwmMotorA, OUTPUT);
    pinMode(_pwmMotorB, OUTPUT);
    pinMode(_dirMotorA, OUTPUT);
    pinMode(_dirMotorB, OUTPUT);

    pinMode(_encoderPinA, INPUT_PULLUP);
    pinMode(_encoderPinB, INPUT_PULLUP);

    attachInterrupt(_encoderPinA, EncoderCallback, RISING);

    _valvePIDController.SetOutputLimits(-1.0 * _motorPIDMaxValue, _motorPIDMaxValue);
    _valvePIDController.SetProportionalMode(QuickPID::pMode::pOnMeas);
    _valvePIDController.SetMode(QuickPID::Control::manual);

    // Calibrate the valve before starting temperature controller
    _mixingValveRange.min = -1000.0;
    _mixingValveRange.max = 1000.0;
    _mixingValveRange.isCalibrated = false;

    doValveCalibration();

    _valveDirectPosition.setMin(_mixingValveRange.min);
    _valveDirectPosition.setMax(_mixingValveRange.max);
    _valveDirectPosition.setStep((_mixingValveRange.max - _mixingValveRange.min) / 10.0f);
    _valveDirectPosition.setMode(HANumber::ModeSlider);
    _valveDirectPosition.setIcon("mdi:compass");
    _valveDirectPosition.setName("Direct Valve Control");
    _valveDirectPosition.setOptimistic(false);
    _valveDirectPosition.onCommand(onNumberCommand);

    _temperaturePIDController.SetOutputLimits(_mixingValveRange.min, _mixingValveRange.max);
    _temperaturePIDController.SetSampleTimeUs(_temperaturePIDTimeStep);
    _temperaturePIDController.SetMode(QuickPID::Control::automatic);

    _device.setAvailability(true);

    _mqtt.begin(BROKER_ADDR, "norrah", "norrah");
}

void loop() {
    _mqtt.loop();

    if (_sensors.isConversionComplete() == 1) {
        float temperatureC = _sensors.getTempCByIndex(0);
        _sensors.requestTemperatures();
        _currentTemperature = temperatureC;
        _hvac.setCurrentTemperature(temperatureC);
        _temperatureSensorReturn.setValue(temperatureC);
    }

    if (_temperaturePIDController.Compute()) {
        Serial.print("Temperature PID: ");
        // _valveMotorSetpoint = _mixingValvePosition;
        valveMotorSetPosition(_mixingValvePosition, 2500);
        _valveDirectPosition.setState(_mixingValvePosition);
        Serial.println(_mixingValvePosition);
    }
}