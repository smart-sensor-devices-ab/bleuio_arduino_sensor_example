/*********************************************************************
 BleuIO Example part 2.
 Example of using a Adafruit Feather RP2040 Board with a BME680 sensor
 and a OTP3002 sensor then advirtising the results using a BleuIO.

 Copyright (c) 2024 Smart Sensor Devices AB
*********************************************************************/

#include <Wire.h>
#include <SPI.h>
/* Adafruid BME680 */
#include <Adafruit_Sensor.h>
#include "Adafruit_BME680.h"
Adafruit_BME680 bme; // I2C

#define SEALEVELPRESSURE_HPA (1013.25)

/* ClosedCube_OPT3002 */
#include "ClosedCube_OPT3002.h"
ClosedCube_OPT3002 opt3002;

// USBHost is defined in usbh_helper.h
#include "usbh_helper.h"
// CDC Host object
Adafruit_USBH_CDC SerialHost;

#define OPT3002_ADDRESS 0x44
#define BME680_ADDRESS 0x76

/* Starting commands for the BleuIO. 
  First it will turn of echo, then set 'BleuIO Arduino Example' as Complete Local Name in the Advertising Response data.
  Lastly it will start Advertising.
*/
#define START_CMDS "ATE0\rAT+ADVRESP=17:09:42:6C:65:75:49:4F:20:41:72:64:75:69:6E:6F:20:45:78:61:6D:70:6C:65\rAT+ADVSTART\r"

// How often we read the sensors and update the advertings message (in seconds) 
#define READ_UPDATE_FREQUENCY   5

/* Global variables */
int loop_cnt;
char dongle_cmd[120];
uint32_t ALS = 0;
uint32_t pressure = 0;
uint32_t temp = 0;
uint32_t hum = 0;
uint32_t voc = 0;


void forward_serial(void) {
  uint8_t buf[256];

  // Serial -> SerialHost
  // if (Serial.available()) {
  //   size_t count = Serial.read(buf, sizeof(buf));
  //   if (SerialHost && SerialHost.connected()) {
  //     SerialHost.write(buf, count);
  //     SerialHost.flush();
  //   }
  // }

  // SerialHost -> Serial
  if (SerialHost.connected() && SerialHost.available()) {
    size_t count = SerialHost.read(buf, sizeof(buf));
    Serial.println("BleuIO response:"); 
    Serial.write(buf, count);
    Serial.println("----"); 
    Serial.flush();
  }
}

void setup() {
  Serial.begin(9600);
  loop_cnt = 0;
  while (!Serial);
  Serial.println(F("BME680 & OPT3002 test v1.1b"));

  // init host stack on controller (rhport) 1
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600);


  if (!bme.begin(BME680_ADDRESS)) {
    Serial.println("Could not find a valid BME680 sensor, check wiring!");
    while (1);
  }

  // Set up oversampling and filter initialization
  bme.setTemperatureOversampling(BME680_OS_8X);
  bme.setHumidityOversampling(BME680_OS_2X);
  bme.setPressureOversampling(BME680_OS_4X);
  bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
  bme.setGasHeater(320, 150); // 320*C for 150 ms

  // Setup OPT3002
  opt3002.begin(OPT3002_ADDRESS);
	Serial.print("OPT3002 Manufacturer ID:");
	Serial.print(opt3002.readManufacturerID());
	Serial.print(" Device ID:");
	Serial.println(opt3002.readDeviceID());

	configureSensor();
	printResult("High-Limit", opt3002.readHighLimit());
	printResult("Low-Limit", opt3002.readLowLimit());
	Serial.println("----");  

  // Delay so we have a chance to read the sensor setup prints
  delay(5000);
}

/* Generate a BleuIO command to change the Advertising Data along with the Advertising Data we want to set.
   The Advertising Data is setup with the flag Manufacturer Specific Data with a made up Company ID 0x1234 (little endian) and the sensor values as the Data */
void generateAdvData(char * input_buffer, uint16_t lux, uint16_t pressure, uint16_t temperature, uint16_t humidity, uint16_t gas_resistance)
{
    sprintf(input_buffer, "AT+ADVDATA=0D:FF:34:12:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X:%02X\r", 
    (uint8_t) (lux >> 8), (uint8_t) (lux & 0xFF), /*big endian*/
    (uint8_t) (pressure >> 8), (uint8_t) (pressure & 0xFF), /*big endian*/
    (uint8_t) (temperature >> 8), (uint8_t) (temperature & 0xFF), /*big endian*/
    (uint8_t) (humidity >> 8), (uint8_t) (humidity & 0xFF), /*big endian*/
    (uint8_t) (gas_resistance >> 8), (uint8_t) (gas_resistance & 0xFF) /*big endian*/
    );
}

void loop() {
  // Forward the output from the dongle to the serial
  forward_serial();

  if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))
  {
      Serial.println("Reading sensors and updating Adv Msg!");

      /* BME680 */
      if (! bme.performReading()) {
        Serial.println("Failed to perform reading :(");
        return;
      }

      Serial.print("Temperature = ");
      Serial.print(bme.temperature);
      Serial.println(" *C");

      Serial.print("Pressure = ");
      Serial.print(bme.pressure / 100.0);
      Serial.println(" hPa");

      Serial.print("Humidity = ");
      Serial.print(bme.humidity);
      Serial.println(" %");

      Serial.print("Gas = ");
      Serial.print(bme.gas_resistance);
      Serial.println(" Ohms");
      // Serial.print(bme.gas_resistance / 1000.0);
      // Serial.println(" KOhms");

      Serial.print("Approx. Altitude = ");
      Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
      Serial.println(" m");
      /* End BME680 */

      /* OPT3002 */
      OPT3002 result = opt3002.readResult();

      printResult("OPT3002", result);
      Serial.println();
      Serial.flush();
      /* End OPT3002 */

      /* Setting the sensor values to fit in a uint16_t in the adverting message, no decimals*/
      ALS = (uint32_t) (result.lux/1000); /* uW/cm2 */
      pressure = (uint32_t) (bme.pressure/100); /* hPa (removing the decimals)*/
      temp = (uint32_t) (bme.temperature); /* Celcius */
      hum = (uint32_t) (bme.humidity); /* %Rh*/
      voc = (uint32_t) bme.gas_resistance / 1000; /* KOhms */

      /*Generate Advertising command to send to the BleuIO*/
      generateAdvData(dongle_cmd, ALS, pressure, temp, hum, voc);

      /* Sending generated command to BleuIO */
      if (SerialHost && SerialHost.connected()) {
        SerialHost.write((uint8_t *)dongle_cmd, strlen(dongle_cmd) +1);
        SerialHost.flush();
      }

      loop_cnt = 0;
  } // end of if(loop_cnt >= (READ_UPDATE_FREQUENCY * 1000))

  loop_cnt++;
  delay(1);
}

//------------- Core1 -------------//
void setup1() {
  // configure pio-usb: defined in usbh_helper.h
  rp2040_configure_pio_usb();

  // run host stack on controller (rhport) 1
  // Note: For rp2040 pico-pio-usb, calling USBHost.begin() on core1 will have most of the
  // host bit-banging processing works done in core1 to free up core0 for other works
  USBHost.begin(1);

  // Initialize SerialHost
  SerialHost.begin(9600);
}

void loop1() {
  USBHost.task();
}

/* ClosedCube_OPT3002 functions */
void printResult(String text, OPT3002 result) {
	if (result.error == NO_ERROR) {
		Serial.print(text);
		Serial.print(": ");
		Serial.print(result.lux);
		Serial.println(" nW/cm2");
	}
	else {
		printError(text, result.error);
	}
}

void printError(String text, OPT3002_ErrorCode error) {
	Serial.print(text);
	Serial.print(": [ERROR] Code #");
	Serial.println(error);
}

void configureSensor() {
	OPT3002_Config newConfig;

	newConfig.RangeNumber = 0b1100;
	newConfig.ConvertionTime = 0b0;
	newConfig.Latch = 0b1;
	newConfig.ModeOfConversionOperation = 0b11;

	OPT3002_ErrorCode errorConfig = opt3002.writeConfig(newConfig);
	if (errorConfig != NO_ERROR)
		printError("OPT3002 configuration", errorConfig);
	else {
		OPT3002_Config sensorConfig = opt3002.readConfig();
		Serial.println("OPT3002 Current Config:");
		Serial.println("------------------------------");

		Serial.print("Conversion ready (R):");
		Serial.println(sensorConfig.ConversionReady, HEX);

		Serial.print("Conversion time (R/W):");
		Serial.println(sensorConfig.ConvertionTime, HEX);

		Serial.print("Fault count field (R/W):");
		Serial.println(sensorConfig.FaultCount, HEX);

		Serial.print("Flag high field (R-only):");
		Serial.println(sensorConfig.FlagHigh, HEX);

		Serial.print("Flag low field (R-only):");
		Serial.println(sensorConfig.FlagLow, HEX);

		Serial.print("Latch field (R/W):");
		Serial.println(sensorConfig.Latch, HEX);

		Serial.print("Mask exponent field (R/W):");
		Serial.println(sensorConfig.MaskExponent, HEX);

		Serial.print("Mode of conversion operation (R/W):");
		Serial.println(sensorConfig.ModeOfConversionOperation, HEX);

		Serial.print("Polarity field (R/W):");
		Serial.println(sensorConfig.Polarity, HEX);

		Serial.print("Overflow flag (R-only):");
		Serial.println(sensorConfig.OverflowFlag, HEX);

		Serial.print("Range number (R/W):");
		Serial.println(sensorConfig.RangeNumber, HEX);

		Serial.println("------------------------------");
	}

}
/* end of ClosedCube_OPT3002 functions */

//--------------------------------------------------------------------+
// TinyUSB Host callbacks
//--------------------------------------------------------------------+
extern "C" {

// Invoked when a device with CDC interface is mounted
// idx is index of cdc interface in the internal pool.
void tuh_cdc_mount_cb(uint8_t idx) {
  // bind SerialHost object to this interface index
  SerialHost.mount(idx);
  Serial.print("SerialHost is connected to a new CDC device. Idx: ");
  Serial.println(idx);

  /* Send start commands to BleuIO when detecting that  */
  if (SerialHost && SerialHost.connected()) 
  {
    SerialHost.write((uint8_t *)START_CMDS, sizeof(START_CMDS));
    SerialHost.flush();
  }
}

// Invoked when a device with CDC interface is unmounted
void tuh_cdc_umount_cb(uint8_t idx) {
  SerialHost.umount(idx);
  Serial.println("SerialHost is disconnected");
}

}
