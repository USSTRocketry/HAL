# HAL

# SensorBMP280 Documentation

The `SensorBMP280` class allows interaction with the **Adafruit BMP280** sensor for measuring temperature, pressure, and altitude. It supports both **I2C** and **SPI** communication modes.

### Features

- Measures **temperature**, **pressure**, and **altitude**.
- Supports **I2C** and **SPI** interfaces.
- Configures sensor settings for optimal performance (sampling rate, filtering, etc.).

### Usage

#### 1. **I2C Mode**
To use the sensor with I2C, instantiate the class with the I2C address and wire (defaults to `I2C_WIRE0`):

```cpp
SensorBMP280 bmp280Sensor(BMP280_ADDRESS, I2C_WIRE0);
```

- `BMP280_ADDRESS`: I2C address (default: `0x76` or `0x77`).
- `I2C_WIRE0`, `I2C_WIRE1`, `I2C_WIRE2`: Select I2C bus (default: `Wire`).

#### 2. **SPI Mode**
For SPI communication, provide the chip select and SPI pins:

```cpp
SensorBMP280 bmp280Sensor(CS_PIN, MISO_PIN, MOSI_PIN, SCK_PIN);
```

- `CS_PIN`, `MISO_PIN`, `MOSI_PIN`, `SCK_PIN`: SPI pins (choose appropriate pins for your setup).

#### 3. **Initialize the Sensor**
Use the `begin()` method to initialize the sensor:

```cpp
uint8_t status = bmp280Sensor.begin();
```

#### 4. **Reading Data**
Read temperature, pressure, and altitude using the `read()` method:

```cpp
BMP280Data* data = bmp280Sensor.read();
```

This returns a pointer to `BMP280Data` containing:
- `data->altitude`: Altitude in meters.
- `data->pressure`: Atmospheric pressure in Pascals.
- `data->temperature`: Temperature in Celsius.

### Constructors

- **I2C Constructor**:
  ```cpp
  SensorBMP280(uint8_t i2c_addr = BMP280_ADDRESS, uint8_t i2c_wire = I2C_WIRE0, float ground_alt = 0.0f);
  ```
  - `i2c_addr`: Sensor I2C address (default: `BMP280_ADDRESS`).
  - `i2c_wire`: I2C wire (default: `I2C_WIRE0`).
  - `ground_alt`: Ground altitude for altitude calculation (optional).

- **SPI Constructor**:
  ```cpp
  SensorBMP280(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck, float ground_alt = 0.0f);
  ```
  - `cs`, `miso`, `mosi`, `sck`: SPI pins.
  - `ground_alt`: Ground altitude for altitude calculation (optional).

### Example Code

```cpp
#include "SensorBMP280.h"

SensorBMP280 bmp280Sensor;

void setup() {
    Serial.begin(9600);
    if (bmp280Sensor.begin()) {
        Serial.println("BMP280 initialized.");
    } else {
        Serial.println("Initialization failed.");
    }
}

void loop() {
    BMP280Data* data = bmp280Sensor.read();
    Serial.print("Temp: ");
    Serial.print(data->temperature);
    Serial.print("°C, Pressure: ");
    Serial.print(data->pressure);
    Serial.print("Pa, Altitude: ");
    Serial.print(data->altitude);
    Serial.println("m");
    delay(1000);
}
```

# SensorAccelGyro Documentation

The `SensorAccelGyro` class provides an interface for interfacing with an accelerometer and gyroscope sensor (e.g., LSM6DSOX). It supports communication via both I2C and SPI protocols and provides methods to initialize and read data from the sensor.

---

## Features

- Supports I2C and SPI communication protocols.
- Provides accelerometer, gyroscope, and temperature readings.
- Configurable I2C bus and SPI pins for flexibility.

---
## Constructor Parameters

### I2C Constructor
```cpp
SensorAccelGyro(uint8_t i2c_addr, uint8_t i2c_wire);
```
- **`i2c_addr`**: The I2C address of the accelerometer/gyroscope sensor.
- **`i2c_wire`**: The I2C bus to use (`I2C_WIRE0`, `I2C_WIRE1`, etc.).

### SPI Constructor
```cpp
SensorAccelGyro(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
```
- **`cs`**: Chip select pin for SPI.
- **`miso`**: Master In Slave Out pin for SPI.
- **`mosi`**: Master Out Slave In pin for SPI.
- **`sck`**: Clock pin for SPI.

---

## Public Methods

### `uint8_t begin()`
Initializes the sensor for communication.

- **Returns**: Status of initialization (`0` for failure, `1` for success).

### `AccelGyroData* read()`
Reads the current accelerometer, gyroscope, and temperature data.

- **Returns**: Pointer to an `AccelGyroData` structure containing:
  - `temperature`: Current temperature in degrees Celsius.
  - `accelX`, `accelY`, `accelZ`: Acceleration in m/s² for X, Y, and Z axes.
  - `gyroX`, `gyroY`, `gyroZ`: Angular velocity in rad/s for X, Y, and Z axes.

---

## Example Usage

### Using I2C
```cpp
#include "SensorAccelGyro.h"

SensorAccelGyro accelGyro(0x6A, I2C_WIRE0); // Default I2C address and bus

void setup() {
    if (!accelGyro.begin()) {
        Serial.println("Failed to initialize accelerometer/gyroscope sensor!");
    }
}

void loop() {
    AccelGyroData* data = accelGyro.read();
    Serial.print("Accel: ");
    Serial.print(data->accelX);
    Serial.print(", ");
    Serial.print(data->accelY);
    Serial.print(", ");
    Serial.println(data->accelZ);

    Serial.print("Gyro: ");
    Serial.print(data->gyroX);
    Serial.print(", ");
    Serial.print(data->gyroY);
    Serial.print(", ");
    Serial.println(data->gyroZ);

    delay(500);
}
```

### Using SPI
```cpp
#include "SensorAccelGyro.h"

SensorAccelGyro accelGyro(10, 12, 11, 13); // SPI pins

void setup() {
    if (!accelGyro.begin()) {
        Serial.println("Failed to initialize accelerometer/gyroscope sensor!");
    }
}

void loop() {
    AccelGyroData* data = accelGyro.read();
    Serial.print("Temperature: ");
    Serial.println(data->temperature);
    delay(500);
}
```

---

# SensorMagnetometer Documentation

The `SensorMagnetometer` class provides an interface for working with a magnetometer sensor (e.g., Adafruit LIS3MDL). It supports communication via both I2C and SPI protocols, allowing flexible hardware integration.

---

## Features

- Supports I2C and SPI communication protocols.
- Provides magnetic field readings for the X, Y, and Z axes.
- Configurable I2C bus and SPI pins for flexible usage.

---

## Constructor Parameters

### I2C Constructor
```cpp
SensorMagnetometer(uint8_t i2c_addr, uint8_t i2c_wire);
```
- **`i2c_addr`**: The I2C address of the magnetometer sensor (default: `LIS3MDL_I2CADDR_DEFAULT`).
- **`i2c_wire`**: The I2C bus to use (`I2C_WIRE0`, `I2C_WIRE1`, etc.).

### SPI Constructor
```cpp
SensorMagnetometer(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
```
- **`cs`**: Chip select pin for SPI.
- **`miso`**: Master In Slave Out pin for SPI.
- **`mosi`**: Master Out Slave In pin for SPI.
- **`sck`**: Clock pin for SPI.

---

## Public Methods

### `uint8_t begin()`
Initializes the magnetometer sensor for communication.

- **Returns**: Status of initialization (`0` for failure, `1` for success).

### `MagnetometerData* read()`
Reads the current magnetic field data.

- **Returns**: Pointer to a `MagnetometerData` structure containing:
  - `magX`, `magY`, `magZ`: Magnetic field values (in µT) for X, Y, and Z axes.

---

## Example Usage

### Using I2C
```cpp
#include "SensorMagnetometer.h"

SensorMagnetometer magnetometer(LIS3MDL_I2CADDR_DEFAULT, I2C_WIRE0); // Default I2C address and bus

void setup() {
    if (!magnetometer.begin()) {
        Serial.println("Failed to initialize magnetometer!");
    }
}

void loop() {
    MagnetometerData* data = magnetometer.read();
    Serial.print("Magnetic Field: ");
    Serial.print(data->magX);
    Serial.print(", ");
    Serial.print(data->magY);
    Serial.print(", ");
    Serial.println(data->magZ);

    delay(500);
}
```

### Using SPI
```cpp
#include "SensorMagnetometer.h"

SensorMagnetometer magnetometer(10, 12, 11, 13); // SPI pins

void setup() {
    if (!magnetometer.begin()) {
        Serial.println("Failed to initialize magnetometer!");
    }
}

void loop() {
    MagnetometerData* data = magnetometer.read();
    Serial.print("Magnetic Field: ");
    Serial.print(data->magX);
    Serial.print(", ");
    Serial.print(data->magY);
    Serial.print(", ");
    Serial.println(data->magZ);

    delay(500);
}
```

# SensorTemperature Documentation

The `SensorTemperature` class provides an interface to interact with the MCP9808 temperature sensor. It supports communication over the I2C protocol and provides methods to initialize and read temperature data.

---

## Features

- I2C communication support for temperature data.
- Easy initialization with configurable I2C address and bus.
- Provides high-accuracy temperature readings.

---

## Constructor Parameters

### I2C Constructor
```cpp
SensorTemperature(uint8_t i2c_addr = MCP9808_I2CADDR_DEFAULT, uint8_t i2c_wire = I2C_WIRE0);
```
- **`i2c_addr`**: I2C address of the MCP9808 sensor (default: `MCP9808_I2CADDR_DEFAULT`).
- **`i2c_wire`**: I2C bus to use (`I2C_WIRE0`, `I2C_WIRE1`, etc.).

---

## Public Methods

### `uint8_t begin()`
Initializes the temperature sensor for communication.

- **Returns**: Status of initialization (`0` for failure, `1` for success).

### `float read()`
Reads the current temperature from the sensor.

- **Returns**: Temperature in degrees Celsius.

---

## Example Usage

```cpp
#include "SensorTemperature.h"

SensorTemperature tempSensor; // Default I2C address and bus

void setup() {
    Serial.begin(115200);
    if (!tempSensor.begin()) {
        Serial.println("Failed to initialize temperature sensor!");
    }
}

void loop() {
    float temperature = tempSensor.read();
    Serial.print("Temperature: ");
    Serial.print(temperature);
    Serial.println(" °C");
    delay(1000);
}
```

# GPS Documentation

The `GPS` class provides an interface for integrating GPS modules using the Adafruit GPS library. It supports hardware serial communication and provides methods for configuration, data retrieval, and command communication.

---

## Features

- Supports hardware serial communication.
- Configurable baud rate and update rate.
- Retrieves latitude, longitude, altitude, and other GPS data.
- Allows sending custom commands to the GPS module.
- Fix detection to check satellite connectivity.

---

## Constructor Parameters

### Constructor
```cpp
GPS(uint8_t serial = GPS_HW_SERIAL, uint32_t baud_rate = 9600);
```

- **`serial`**: Specifies the hardware serial port to use. Use predefined macros (`GPS_HW_SERIAL0`, `GPS_HW_SERIAL1`, etc.) for easy configuration.
- **`baud_rate`**: Communication speed with the GPS module. Defaults to `9600`.

---

## Public Methods

### `void begin()`
Initializes the GPS module with the specified serial port and baud rate.

### `void configure(uint32_t update_rate_ms = 1000, const char* output_mode = PMTK_SET_NMEA_OUTPUT_RMCONLY)`
Configures the GPS module settings.

- **`update_rate_ms`**: Update interval for GPS data (in milliseconds). Default is `1000 ms`.
- **`output_mode`**: NMEA output mode. Default is `PMTK_SET_NMEA_OUTPUT_RMCONLY` (only RMC data).

### `GPSData* read()`
Reads the latest GPS data and updates the `GPSData` structure.

- **Returns**: Pointer to a `GPSData` structure containing:
  - `latitude` and `longitude`: Coordinates in decimal degrees.
  - `altitude`: Altitude above sea level (meters).
  - `speed`: Speed in km/h.
  - `fixQuality`: Indicates the GPS fix quality (0 for no fix, 1 for standard, 2+ for enhanced fixes).

### `bool hasFix()`
Checks if the GPS module has a satellite fix.

- **Returns**: `true` if a fix is available, `false` otherwise.

### `void sendCommand(const char* command)`
Sends a custom command string to the GPS module.

---

## Example Usage

### Basic Example
```cpp
#include "GPS.h"

GPS gps(GPS_HW_SERIAL1, 9600);

void setup() {
    Serial.begin(115200);
    gps.begin();
    gps.configure(1000, PMTK_SET_NMEA_OUTPUT_RMCONLY);
}

void loop() {
    GPSData* data = gps.read();
    if (gps.hasFix()) {
        Serial.print("Lat: "); Serial.println(data->latitude, 6);
        Serial.print("Lon: "); Serial.println(data->longitude, 6);
        Serial.print("Alt: "); Serial.println(data->altitude);
    } else {
        Serial.println("No GPS fix available.");
    }
    delay(1000);
}
```

# RYLR998Radio Documentation

The `RYLR998Radio` class provides an interface for the RYLR998 LoRa module, enabling long-range communication over LoRa networks. It supports configurable hardware serial communication on compatible microcontrollers like the Teensy.

---

## Features

- Supports hardware serial communication for Teensy and similar boards.
- Provides methods for sending and receiving data.
- Allows configuration of LoRa parameters such as frequency, transmission power, spreading factor, bandwidth, and coding rate.

---

## Constructor Parameters

```cpp
RYLR998Radio(uint8_t serialPort = RYLR998_HW_SERIAL, uint32_t baudRate = 9600);
```

- **`serialPort`**: The hardware serial port to use for communication with the RYLR998 module. Defaults to `RYLR998_HW_SERIAL`.
- **`baudRate`**: The communication baud rate. Defaults to `9600`.

---

## Public Methods

### `bool begin()`
Initializes the LoRa module for communication.

- **Returns**: `true` if initialization is successful, `false` otherwise.

---

### `bool send(const uint8_t* data, size_t length)`
Sends data over the LoRa network.

- **Parameters**:
  - `data`: Pointer to the data buffer to send.
  - `length`: Size of the data buffer.
- **Returns**: `true` if the data is successfully sent, `false` otherwise.

---

### `bool receive(uint8_t* buffer, size_t maxLength, size_t& receivedLength)`
Receives data from the LoRa network.

- **Parameters**:
  - `buffer`: Pointer to the buffer to store received data.
  - `maxLength`: Maximum size of the buffer.
  - `receivedLength`: Reference to store the size of the received data.
- **Returns**: `true` if data is successfully received, `false` otherwise.

---

### `void setFrequency(float frequency)`
Sets the operating frequency of the LoRa module.

- **Parameter**:
  - `frequency`: Frequency in MHz (e.g., `915.0` for the 915 MHz band).

---

### `void setTxPower(uint8_t power)`
Configures the transmission power.

- **Parameter**:
  - `power`: Transmission power level (range depends on the module specifications).

---

### `void configureLoRa(uint8_t spreadingFactor, uint16_t bandwidth, uint8_t codingRate)`
Sets advanced LoRa configuration parameters.

- **Parameters**:
  - `spreadingFactor`: Value from `6` to `12`, defining the chirp spreading rate.
  - `bandwidth`: Bandwidth in kHz (e.g., `125`, `250`, `500`).
  - `codingRate`: Coding rate from `1` to `4`.

---

## Example Usage

### Basic Usage

```cpp
#include "RYLR998Radio.h"

RYLR998Radio lora;

void setup() {
    Serial.begin(9600);
    if (!lora.begin()) {
        Serial.println("Failed to initialize LoRa module!");
    }
    lora.setFrequency(915.0);
    lora.setTxPower(14);
}

void loop() {
    const char* message = "Hello, LoRa!";
    if (lora.send((uint8_t*)message, strlen(message))) {
        Serial.println("Message sent successfully!");
    }
    delay(1000);
}
```

### Receiving Data

```cpp
#include "RYLR998Radio.h"

RYLR998Radio lora;
uint8_t buffer[256];
size_t receivedLength;

void setup() {
    Serial.begin(9600);
    if (!lora.begin()) {
        Serial.println("Failed to initialize LoRa module!");
    }
}

void loop() {
    if (lora.receive(buffer, sizeof(buffer), receivedLength)) {
        Serial.print("Received: ");
        Serial.write(buffer, receivedLength);
        Serial.println();
    }
    delay(100);
}
```

