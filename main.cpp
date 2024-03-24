// SenESP Engine Monitor
// Production Code for Apres' Engine Monitor
// Last modification was to add Bilge monitor code

#include <Adafruit_BME280.h>  //JG Added
#include <Wire.h>
#include "sensesp_onewire/onewire_temperature.h"
#include <Arduino.h>
#include "sensesp/sensors/analog_input.h"
#include "sensesp/sensors/digital_input.h"
#include "sensesp/sensors/sensor.h"
#include "sensesp/signalk/signalk_output.h"
#include "sensesp/system/lambda_consumer.h"
#include "sensesp_app_builder.h"
#include "sensesp/transforms/linear.h"
#include "sensesp/transforms/analogvoltage.h"
#include "sensesp/transforms/curveinterpolator.h"
#include "sensesp/transforms/voltagedivider.h"
#include "sensesp/transforms/frequency.h"
#include "sensesp/transforms/lambda_transform.h"

using namespace sensesp;

class FuelInterpreter : public CurveInterpolator {
 public:
  FuelInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate RPM to M^3/s
    clear_samples();
    // addSample(CurveInterpolator::Sample(RPM, M^3/s));
    add_sample(CurveInterpolator::Sample(500, 0.00000473125946250));
    add_sample(CurveInterpolator::Sample(1000, 0.00000630834595000));
    add_sample(CurveInterpolator::Sample(1500, 0.00001261669190000));
    add_sample(CurveInterpolator::Sample(1800, 0.00001577086487500));
    add_sample(CurveInterpolator::Sample(2000, 0.00001955587244500));
    add_sample(CurveInterpolator::Sample(2100, 0.00002207921082500));
    add_sample(CurveInterpolator::Sample(2500, 0.00004100424867500));
    add_sample(CurveInterpolator::Sample(2600, 0.00004731259462500));
    add_sample(CurveInterpolator::Sample(2800, 0.00005677511355000));
    add_sample(CurveInterpolator::Sample(3000, 0.00006939180545000));
    add_sample(CurveInterpolator::Sample(3200, 0.00008831684330000));
    add_sample(CurveInterpolator::Sample(3400, 0.00010724188115000));  
  }
};

  // Oil Pressure lookup
class PressureInterpreter : public CurveInterpolator {
 public:
  PressureInterpreter(String config_path = "")
      : CurveInterpolator(NULL, config_path) {
    // Populate a lookup table to translate the ohm values returned by
    // our Pressure sender to Pascal
    clear_samples();
    // addSample(CurveInterpolator::Sample(knownOhmValue, knownPascal));
    add_sample(CurveInterpolator::Sample(10, 0));
    add_sample(CurveInterpolator::Sample(21, 50000));
    add_sample(CurveInterpolator::Sample(31, 100000));
    add_sample(CurveInterpolator::Sample(42, 150000));
    add_sample(CurveInterpolator::Sample(52, 200000));
    add_sample(CurveInterpolator::Sample(71, 300000));
    add_sample(CurveInterpolator::Sample(90, 400000));
    add_sample(CurveInterpolator::Sample(107, 500000));
    add_sample(CurveInterpolator::Sample(124, 600000)); 
    add_sample(CurveInterpolator::Sample(140, 700000));
    add_sample(CurveInterpolator::Sample(156, 800000));
    add_sample(CurveInterpolator::Sample(163, 850000));
    add_sample(CurveInterpolator::Sample(170, 900000));
    add_sample(CurveInterpolator::Sample(184, 1000000)); 
  }
};

reactesp::ReactESP app;


// BME280
  
    Adafruit_BME280 bme280;

  float read_temp_callback() { return (bme280.readTemperature() + 273.15);}
  float read_pressure_callback() { return (bme280.readPressure());}
  float read_humidity_callback() { return (bme280.readHumidity());}


// The setup function performs one-time application initialization.
void setup() {
#ifndef SERIAL_DEBUG_ENABLED
  SetupSerialDebug(115200);
#endif

  Wire.begin(21,22);                // join i2c bus (address optional for master)
// 
 //Serial.begin(9600);          // start serial communication at 9600bps
  
  //Serial.println(F("BME280 Forced Mode Test."));

  //if (!bmp.begin(BMP280_ADDRESS_ALT, BMP280_CHIPID)) {
  //if (!bme280.begin()) {
    //Serial.println(F("Could not find a valid BMP280 sensor, check wiring or "
      //                "try a different address!"));
    //while (1) delay(10);// could need a delay here:
  //}   
//
 
  // Construct the global SensESPApp() object
  SensESPAppBuilder builder;
  sensesp_app = (&builder)
                    // Set a custom hostname for the app.
                    ->set_hostname("Apres-Eng-Mon")
                    // Optionally, hard-code the WiFi and Signal K server
                    // settings. This is normally not needed.
                    //->set_wifi("My WiFi SSID", "my_wifi_password")
                    //->set_sk_server("192.168.10.3", 80)
                    ->enable_uptime_sensor()
                    ->get_app();

/// 1-Wire Temp Sensors - Exhaust Temp Sensors ///

  DallasTemperatureSensors* dts = new DallasTemperatureSensors(25); //digital 2

 // Oil temp (fasten to oil P sensor) - /propulsion/engine/oilTemperature
  auto* oil_temp =
      new OneWireTemperature(dts, 1000, "/Oil Temperature/oneWire");

  oil_temp->connect_to(new Linear(1.0, 0.0, "/Oil Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.oilTemperature",
                             "/Oil Temperature/sk_path"));

// Aft Cabin temp - /environment/inside/aftCabin/temperature                             
  auto* aft_cabin_temp =
      new OneWireTemperature(dts, 1000, "/Aft Cabin Temperature/oneWire");

  aft_cabin_temp->connect_to(new Linear(1.0, 0.0, "/Aft Cabin Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("environment.inside.aftCabin.temperature",
                             "/Aft Cabin Temperature/sk_path"));

// Exhaust Elbow Temp sensor - /propulsion/engine/intakeManifoldTemperature
  auto* elbow_temp =
      new OneWireTemperature(dts, 1000, "/Elbow Temperature/oneWire");

  elbow_temp->connect_to(new Linear(1.0, 0.0, "/Elbow Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.intakeManifoldTemperature",
                             "/Elbow Temperature/sk_path"));

// Exhaust barrel sensor - /propulsion/engine/exhaustTemperature
  auto* exhaust_temp =
      new OneWireTemperature(dts, 1000, "/Exhaust Temperature/oneWire");

  exhaust_temp->connect_to(new Linear(1.0, 0.0, "/Exhaust Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("propulsion.engine.exhaustTemperature",
                             "/Exhaust Temperature/sk_path"));

// Alternator Temperature - /electrical/alternator/temperature
  auto* alternator_temp =
      new OneWireTemperature(dts, 1000, "/Alternator Temperature/oneWire");

  alternator_temp->connect_to(new Linear(1.0, 0.0, "/Alternator Temperature/linear"))
      ->connect_to(
          new SKOutputFloat("electrical.alternator.temperature",
                             "/Alternator Temperature/sk_path"));

 //RPM Application/////

  const char* config_path_calibrate = "/Engine RPM/calibrate";
  const char* config_path_skpath = "/Engine RPM/sk_path";
  const float multiplier = 1.0;

  auto* sensor = new DigitalInputCounter(16, INPUT_PULLUP, RISING, 500);

  sensor->connect_to(new Frequency(multiplier, config_path_calibrate))  
  // connect the output of sensor to the input of Frequency()
          ->connect_to(new SKOutputFloat("propulsion.engine.revolutions", config_path_skpath));  
          // connect the output of Frequency() to a Signal K Output as a number

  sensor->connect_to(new Frequency(6))
  // times by 6 to go from Hz to RPM
           ->connect_to(new FuelInterpreter("/Engine Fuel/curve"))
          ->connect_to(new SKOutputFloat("propulsion.engine.fuel.rate", "/Engine Fuel/sk_path"));                                       

 
/// BME280 SENSOR CODE - Temp/Humidity/Altitude/Pressure Sensor ////  

  // 0x77 is the default address. Some chips use 0x76, which is shown here.
  // If you need to use the TwoWire library instead of the Wire library, there
  // is a different constructor: see bmp280.h

  bme280.begin();
  // Create a RepeatSensor with float output that reads the temperature
  // using the function defined above.
  auto* bme280_temp =
      new RepeatSensor<float>(5000, read_temp_callback);

  auto* bme280_pressure = 
      new RepeatSensor<float>(60000, read_pressure_callback);

  auto* bme280_humidity = 
      new RepeatSensor<float>(60000, read_humidity_callback);     

  // Send the temperature to the Signal K server as a Float
  bme280_temp->connect_to(new SKOutputFloat("environment.inside.engineBay.temperature"));

  bme280_pressure->connect_to(new SKOutputFloat("environment.inside.engineBay.pressure"));

  bme280_humidity->connect_to(new SKOutputFloat("environment.inside.engineBay.relativeHumidity"));

//// Pressure Sender Config ////

const float Vin = 3.45;
const float R1 = 47.0;
auto* analog_input = new AnalogInput(36, 500); //- Pin 36 is Analogue 0

analog_input->connect_to(new AnalogVoltage(Vin,Vin))
      ->connect_to(new VoltageDividerR2(R1, Vin, "/Engine Pressure/sender"))
      ->connect_to(new PressureInterpreter("/Engine Pressure/curve"))
      ->connect_to(new Linear(1.0, 0.0, "/Engine Pressure/calibrate"))
      ->connect_to(new SKOutputFloat("propulsion.engine.oilPressure", "/Engine Pressure/sk_path"));

//// Bilge Monitor /////
auto* bilge = new DigitalInputState(13, INPUT_PULLUP, 5000);  //- Pin 13 is Digital 7

	auto int_to_string_function = [](int input) ->String {
		 if (input == 1) {
		   return "Water present!";
		 } 
		 else { // input == 0
		   return "bilge clear";
		 }
	};

auto int_to_string_transform = new LambdaTransform<int, String>(int_to_string_function);

bilge->connect_to(int_to_string_transform)
      ->connect_to(new SKOutputString("propulsion.engine.bilge"));

bilge->connect_to(new SKOutputFloat("propulsion.engine.bilge.raw"));

  // Start networking, SK server connections and other SensESP internals
  sensesp_app->start();

}

void loop() { app.tick(); }