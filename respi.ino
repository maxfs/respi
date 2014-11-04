/**
 * Project: Respi - A Teensy 3.1 Arduino-powered respiration monitor.
 * Author: Max-Ferdinand Suffel
 * Creation Data: 11-02-2014
 */

#include <SdFat.h> // SdFat library is used to access the SD card.

// The SPI chip-select of the SD card reader is port 6.
static const int sd_chip_select = 6;
// Fat32 filesystem on SD card
SdFat sd;
// Log file use for writing out the sensor data.
SdFile log_file;

// Serial report output stream
ArduinoOutStream cout(Serial);

void setup () {
    // Initialize serial port with baud rate 9600bips
    Serial.begin(9600);
    // Wait until serial monitor is openend.
    while (!Serial) {}
    // Small delay to ensure the serial port is initialized.
    delay(1000);

    // DEBUG: Disable LCD SPI port temporarily.
    pinMode(10, OUTPUT);
    digitalWrite(10, HIGH);

    cout << pstr("Initialize SD card...");

    // Initialize sd card or print a detailed error message and halt
    // We run the SPI by half speed since the SD card was turned out
    // not to be accessable with full speed.
    if (!sd.begin(sd_chip_select, SPI_HALF_SPEED))
    sd.initErrorHalt();

    cout << pstr("ok") << endl;
    cout << pstr("Create log.txt file...");

    // Create a new log file with write persmissions.
    if (!log_file.open("log.txt", O_RDWR | O_CREAT))
    sd.errorHalt_P(PSTR("Cannot create log.txt file"));

    cout << pstr("ok") << endl;

    log_file.close();
}

void loop () {

}
