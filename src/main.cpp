#include <Arduino.h>

// #include <ESP8266WiFi.h>

// #include <DNSServer.h>
// #include <ESP8266WebServer.h>
// #include <WiFiManager.h>

// #include <ArduinoHA.h>

#include <OneWire.h>
#include <DallasTemperature.h>

#include <QuickPID.h>
// #include <PID_v1.h>


// Motor driver
const int pwmMotorA = D1;
const int pwmMotorB = D2;
const int dirMotorA = D3;
const int dirMotorB = D4;
int motorSpeed = 70;

const int tempSensorData = D5;
OneWire oneWire(tempSensorData);
DallasTemperature sensors(&oneWire);

const int encoderPinA = D6;
const int encoderPinB = D7;
volatile long encoderTicks = 0;

// PID controller for temperature control
float Setpoint, Input, Output;
float Kp = 0.8, Ki = 1.6, Kd = 0.1;

float valveLeftPos = -1000.0;
float valveRightPos = 1000.0;
bool valveCalibrated = false;

QuickPID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, QuickPID::Action::direct);

// PID for motor driver
float motorSetpoint, motorInput, motorOutput;
float mKp = 2.0, mKi = 5.0, mKd = 0.4;
float motorPIDMaxValue = 1023.0f;
QuickPID motorPID(&motorInput, &motorOutput, &motorSetpoint, mKp, mKi, mKd, QuickPID::Action::direct);


ICACHE_RAM_ATTR void EncoderCallback(){
  if(digitalRead(encoderPinB) == 1) encoderTicks++;
  else encoderTicks--;
}

// set motor target speed
void motorSetSpeed(float speed){
  if (abs(speed) < 50){
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
void doValveCalibration(){

  // the total span of the valve is about 90 degrees (4-way valve).
  // the gear ratio is 16:48 (sprockets)  - the motor shaft range 270 degrees
  // TODO: the motor gear ratio 1:131? 1:100?
  // encoder is 64 CPR.
  // full valve range in CPR: 64 * (100/1) * (48/16) * (1/4) = 4800
  // so, we spin the motor 20% more than that to make sure we hit the limit

  long fullRangeTicks = 4800;
  
  uint32_t timeout = 25000000; // 25 sec timeout

  uint32_t timeStart = micros();
  uint32_t timeLast = micros();

  // create an array for speed measurements and fill it with meaningfull numbers
  int windowSize = 20;
  float speedMeasurements[windowSize];
  int speedMeasurementsIdx = 0;
  for (int i = 0; i<windowSize; i++) speedMeasurements[i] = 2.0f;
  float speedPIDCurrentSpeed = 0.0f;

  long lastMotorTicks = encoderTicks;

  motorSetpoint = -1.0f * fullRangeTicks * 1.5f;

  while(micros() - timeStart < timeout) {

    yield();

    motorInput = encoderTicks;

    if (motorPID.Compute()) {
      motorSetSpeed(motorOutput);
      speedPIDCurrentSpeed = ((float)(encoderTicks - lastMotorTicks))/((float)(micros() - timeLast)/1000);
      speedMeasurements[speedMeasurementsIdx] = speedPIDCurrentSpeed;
      if (speedMeasurementsIdx++ > windowSize) speedMeasurementsIdx = 0;

      // break if the average speed is close to 0
      float avgSpeed = 0.0f;
      for (int i = 0; i<windowSize; i++) avgSpeed += speedMeasurements[i];
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

  valveLeftPos = encoderTicks;
  Serial.println(valveLeftPos);

  // going right
  motorSetpoint = fullRangeTicks * 1.5f;
  for (int i = 0; i<windowSize; i++) speedMeasurements[i] = 2.0f;

  while(micros() - timeStart < timeout) {

    yield();

    motorInput = encoderTicks;

    if (motorPID.Compute()) {
      motorSetSpeed(motorOutput);
      speedPIDCurrentSpeed = ((float)(encoderTicks - lastMotorTicks))/((float)(micros() - timeLast)/1000);
      speedMeasurements[speedMeasurementsIdx] = speedPIDCurrentSpeed;
      if (speedMeasurementsIdx++ > windowSize) speedMeasurementsIdx = 0;

      // break if the average speed is close to 0
      float avgSpeed = 0.0f;
      for (int i = 0; i<windowSize; i++) avgSpeed += speedMeasurements[i];
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

  valveRightPos = encoderTicks;
  Serial.println(valveRightPos);

}

void setup()
{
  // WiFi.begin("AndyNet-IoT", "andynetfetthehe");
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(500);
  //   Serial.print(".");
  // }
  // Serial.println();

  // Serial.print("Connected, IP address: ");
  // Serial.println(WiFi.localIP());


    Serial.begin(115200);
    Serial.println("Starting...");

    delay(2000);

    pinMode(pwmMotorA, OUTPUT);
    pinMode(pwmMotorB, OUTPUT);
    pinMode(dirMotorA, OUTPUT);
    pinMode(dirMotorB, OUTPUT);

    pinMode(encoderPinA, INPUT_PULLUP);
    pinMode(encoderPinB, INPUT_PULLUP);

    attachInterrupt(encoderPinA, EncoderCallback, RISING);

    pinMode(tempSensorData, INPUT_PULLUP);
    
    sensors.begin();

    Setpoint = 28.0f;
    myPID.SetOutputLimits(valveLeftPos, valveRightPos);
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
  //     if(i==0) Input = temperatureC;
  //     Serial.print(i);
  //     Serial.print(":");
  //     Serial.print(temperatureC);
  //     Serial.print("ºC ");
  //   }

  if (sensors.isConversionComplete() == 1){
    float temperatureC = sensors.getTempCByIndex(0);
    sensors.requestTemperatures();
    Input = temperatureC;
    // Serial.println(temperatureC);
  }

  if (myPID.Compute()){
    motorSetpoint = Output;
    Serial.println();
    Serial.print("Motor: ");
    Serial.print(Input);
    Serial.print(" ");
    Serial.println(Output);
  }

  motorInput = encoderTicks;
  if (motorPID.Compute()) {
    motorSetSpeed(motorOutput);
  //   Serial.print(" ");
  //   Serial.print(motorInput);
  }



}