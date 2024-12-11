#include <cstdint>

#define I2C_WIRE0   0 // default (SDA = 18, SCL = 19)
#define I2C_WIRE1   1 // (SDA = 17, SCL = 16)
#define I2C_WIRE2   2 // (SDA = 25, SCL = 24)

#define I2C_WIRE(w) (w == 0 ? Wire : (w == 1 ? Wire1 : Wire2))

#define SENSOR_MODE_I2C 0
#define SENSOR_MODE_SPI 1


class Sensor
{
protected:
    // I2C
    uint8_t i2c_addr;
    uint8_t i2c_wire;

    // SPI
    uint8_t spi_cs;
    uint8_t spi_miso;
    uint8_t spi_mosi;
    uint8_t spi_sck;

    uint8_t sensor_mode;
    uint8_t status;
public:
    Sensor(uint8_t i2c_addr, uint8_t i2c_wire);
    Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck);
    ~Sensor();
};

Sensor::Sensor(uint8_t i2c_addr, uint8_t i2c_wire = I2C_WIRE0)
: i2c_addr(i2c_addr)
, i2c_wire(i2c_wire)
{
    sensor_mode = SENSOR_MODE_I2C;
}

Sensor::Sensor(uint8_t cs, uint8_t miso, uint8_t mosi, uint8_t sck)
: spi_cs(cs)
, spi_miso(miso)
, spi_mosi(mosi)
, spi_sck(sck)
{
    sensor_mode = SENSOR_MODE_SPI;
}

Sensor::~Sensor()
{
}
