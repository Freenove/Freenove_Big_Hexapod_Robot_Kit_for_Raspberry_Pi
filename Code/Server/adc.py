import smbus  # Import the smbus module for I2C communication
import time  # Import the time module for sleep functionality

class ADC:
    def __init__(self):
        """Initialize the ADC class."""
        self.ADS7830_COMMAND = 0x84                                           # Set the command byte for ADS7830
        self.adc_voltage_coefficient = 3                                      # Set the ADC voltage coefficient based on the PCB version
        self.i2c_bus = smbus.SMBus(1)                                         # Initialize the I2C bus
        self.I2C_ADDRESS = 0x48                                               # Set the I2C address for the ADC

    def scan_i2c_bus(self) -> list:
        """Scan the I2C bus for connected devices."""
        print("Scanning I2C bus...")                                          # Print a message indicating the start of I2C bus scanning
        iic_addr = [None]
        for device in range(128):                                             # Iterate over possible I2C addresses (0 to 127)
            try:
                self.i2c_bus.read_byte_data(device, 0)                        # Try to read data from the current device address
                print(f"Device found at address: 0x{device:02X}")             # Print the address of the found device
                iic_addr.append(device)                                       # Return the address of the found device
            except OSError:
                pass
        return iic_addr                                                       # Return the list of found I2C addresses
    def _read_stable_byte(self) -> int:
        """Read a stable byte from the ADC."""
        while True:
            value1 = self.i2c_bus.read_byte(self.I2C_ADDRESS)                 # Read the first byte from the ADC
            value2 = self.i2c_bus.read_byte(self.I2C_ADDRESS)                 # Read the second byte from the ADC
            if value1 == value2:
                return value1                                                 # Return the value if both reads are the same

    def read_channel_voltage(self, channel: int) -> float:
        """Read the ADC value for the specified channel using ADS7830."""
        command_set = self.ADS7830_COMMAND | ((((channel << 2) | (channel >> 1)) & 0x07) << 4)  # Calculate the command set for the specified channel
        self.i2c_bus.write_byte(self.I2C_ADDRESS, command_set)                # Write the command set to the ADC
        value = self._read_stable_byte()                                      # Read a stable byte from the ADC
        voltage = value / 255.0 * 5 * self.adc_voltage_coefficient                # Convert the ADC value to voltage
        return round(voltage, 2)                                              # Return the voltage rounded to 2 decimal places

    def read_battery_voltage(self) -> float:
        """Read the battery voltage using ADS7830."""
        battery1 = self.read_channel_voltage(0)                                   # Read the battery voltage from channel 0
        battery2 = self.read_channel_voltage(4)                                   # Read the battery voltage from channel 4
        return battery1,battery2

    def close_i2c(self) -> None:
        """Close the I2C bus."""
        self.i2c_bus.close()                                                  # Close the I2C bus

if __name__ == '__main__':
    print("start .. \n")                                                      # Print a message indicating the start of the program
    adc = ADC()                                                               # Create an instance of the ADC class
    try:
        while True:
            power = adc.read_battery_voltage()
            print ("The battery voltage is "+str(power)+'\n')
            time.sleep(1)                                                     # Wait for 1 second
    except KeyboardInterrupt:                                                 # If the program is interrupted by the user
        print("\nEnd of program")
        adc.close_i2c()                                                       # Close the I2C bus when the program is interrupted