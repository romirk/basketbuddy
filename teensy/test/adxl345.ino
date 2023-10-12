// Add the SPI library so we can communicate with the ADXL345 sensor
#include <SPI.h>

// Assign the Chip Select signal to pin 10.
int CS = 10;

// ADXL345 Register Addresses
#define DEVID 0x00          // Device ID Register
#define THRESH_TAP 0x1D     // Tap Threshold
#define OFSX 0x1E           // X-axis offset
#define OFSY 0x1F           // Y-axis offset
#define OFSZ 0x20           // Z-axis offset
#define DURATION 0x21       // Tap Duration
#define LATENT 0x22         // Tap latency
#define WINDOW 0x23         // Tap window
#define THRESH_ACT 0x24     // Activity Threshold
#define THRESH_INACT 0x25   // Inactivity Threshold
#define TIME_INACT 0x26     // Inactivity Time
#define ACT_INACT_CTL 0x27  // Axis enable control for activity and inactivity detection
#define THRESH_FF 0x28      // free-fall threshold
#define TIME_FF 0x29        // Free-Fall Time
#define TAP_AXES 0x2A       // Axis control for tap/double tap
#define ACT_TAP_STATUS 0x2B // Source of tap/double tap
#define BW_RATE 0x2C        // Data rate and power mode control
#define POWER_CTL 0x2D      // Power Control Register
#define INT_ENABLE 0x2E     // Interrupt Enable Control
#define INT_MAP 0x2F        // Interrupt Mapping Control
#define INT_SOURCE 0x30     // Source of interrupts
#define DATA_FORMAT 0x31    // Data format control
#define DATAX0 0x32         // X-Axis Data 0
#define DATAX1 0x33         // X-Axis Data 1
#define DATAY0 0x34         // Y-Axis Data 0
#define DATAY1 0x35         // Y-Axis Data 1
#define DATAZ0 0x36         // Z-Axis Data 0
#define DATAZ1 0x37         // Z-Axis Data 1
#define FIFO_CTL 0x38       // FIFO control
#define FIFO_STATUS 0x39    // FIFO status

// This buffer will hold values read from the ADXL345 registers.
unsigned char values[10];
char output[20];
// These variables will be used to hold the x,y and z axis accelerometer values.
int x, y, z;
double xg, yg, zg;
char tapType = 0;

void setup()
{
  // Initiate an SPI communication instance.
  SPI.begin();
  // Configure the SPI connection for the ADXL345.
  SPI.setDataMode(SPI_MODE3);
  // Create a serial connection to display the data on the terminal.
  Serial.begin(9600);

  // Set up the Chip Select pin to be an output from the Arduino.
  pinMode(CS, OUTPUT);
  // Before communication starts, the Chip Select pin needs to be set high.
  digitalWrite(CS, HIGH);

  // Put the ADXL345 into +/- 2G range by writing the value 0x00 to the DATA_FORMAT register.
  writeRegister(DATA_FORMAT, 0x00);

  // disable interrupts
  writeRegister(INT_ENABLE, 0x00);

  // Put the ADXL345 into Measurement Mode by writing 0x08 to the POWER_CTL register.
  writeRegister(POWER_CTL, 0x08);      // Measurement mode
  readRegister(INT_SOURCE, 1, values); // Clear the interrupts from the INT_SOURCE register.
}

int16_t tenBitTwosComplementToDecimal(uint16_t x)
{
  boolean negative = (x & (1 << 9)) != 0;
  if (negative)
    return x | ~((1 << 10) - 1);
  return (int16_t)x;
}

void loop()
{
  // Reading 6 bytes of data starting at register DATAX0 will retrieve the x,y and z acceleration values from the ADXL345.
  // The results of the read operation will get stored to the values[] buffer.
  readRegister(DATAX0, 6, values);

  // The ADXL345 gives 10-bit acceleration values, but they are stored as bytes (8-bits). To get the full value, two bytes must be combined for each axis.
  // The X value is stored in values[0] and values[1].
  x = tenBitTwosComplementToDecimal((((uint16_t)values[1] << 8) | (uint16_t)values[0]) & 1023);
  // The Y value is stored in values[2] and values[3].
  y = tenBitTwosComplementToDecimal((((uint16_t)values[3] << 8) | (uint16_t)values[2]) & 1023);
  // The Z value is stored in values[4] and values[5].
  z = tenBitTwosComplementToDecimal((((uint16_t)values[5] << 8) | (uint16_t)values[4]) & 1023);

  // Convert the accelerometer value to G's.
  // With 10 bits measuring over a +/-4g range we can find how to convert by using the equation:
  //  Gs = Measurement Value * (G-range/(2^10)) or Gs = Measurement Value * (8/1024)
  xg = x * 0.00390625;
  yg = y * 0.00390625;
  zg = z * 0.00390625;

  Serial.print((float)xg, 2);
  Serial.print(",");
  Serial.print((float)yg, 2);
  Serial.print(",");
  Serial.print((float)zg, 2);
  Serial.println("");
  /*for(unsigned char i = 0; i < 6; i++)
  {
    Serial.print(values[i]);
    Serial.print(",");
  }
  Serial.println();*/
  delay(100);
}

// This function will write a value to a register on the ADXL345.
// Parameters:
//   char registerAddress - The register to write a value to
//   char value - The value to be written to the specified register.
void writeRegister(char registerAddress, unsigned char value)
{
  // Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(CS, LOW);
  // Transfer the register address over SPI.
  SPI.transfer(registerAddress);
  // Transfer the desired register value over SPI.
  SPI.transfer(value);
  // Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(CS, HIGH);
}

// This function will read a certain number of registers starting from a specified address and store their values in a buffer.
// Parameters:
//   char registerAddress - The register addresse to start the read sequence from.
//   int numBytes - The number of registers that should be read.
//   char * values - A pointer to a buffer where the results of the operation should be stored.
void readRegister(char registerAddress, int numBytes, unsigned char *values)
{
  // Since we're performing a read operation, the most significant bit of the register address should be set.
  char address = 0x80 | registerAddress;
  // If we're doing a multi-byte read, bit 6 needs to be set as well.
  if (numBytes > 1)
    address = address | 0x40;

  // Set the Chip select pin low to start an SPI packet.
  digitalWrite(CS, LOW);
  // Transfer the starting register address that needs to be read.
  SPI.transfer(address);
  // Continue to read registers until we've read the number specified, storing the results to the input buffer.
  for (int i = 0; i < numBytes; i++)
  {
    values[i] = SPI.transfer(0x00);
  }
  // Set the Chips Select pin high to end the SPI packet.
  digitalWrite(CS, HIGH);
}
