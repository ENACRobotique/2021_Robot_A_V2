#include <i2c_t3.h>
#include "testI2C.h"
// Function prototypes
//void print_scan_status(uint8_t target, uint8_t all);

uint8_t found, target, all = 1;

void testi2c_s()
{

    Serial.begin(115200);
    pinMode(LED_BUILTIN,OUTPUT);    // LED
    //pinMode(12,INPUT_PULLUP);       // pull pin 12 low to show ACK only results
    //pinMode(11,INPUT_PULLUP);       // pull pin 11 low for a more verbose result (shows both ACK and NACK)

    // Setup for Master mode, pins 18/19, external pullups, 400kHz, 10ms default timeout
    Wire.begin(I2C_MASTER, 0x00, I2C_PINS_18_19, I2C_PULLUP_EXT, 400000);
    //Wire.begin(I2C_MASTER, 0x00, I2C_PINS_37_38, I2C_PULLUP_EXT, 400000);
    delay(500);
    Wire.setDefaultTimeout(10000); // 10ms
    delay(500);

}

void testi2c_l()
{
    // Scan I2C addresses

    //all = (digitalRead(11) == LOW);
    found = 0;
    
    Serial.println("\n---------------------------------------------------\n");
    Serial.println("Starting scan...");
    delay(500);
    digitalWrite(LED_BUILTIN,HIGH); // LED on
    for(target = 1; target <= 0x70; target++) // sweep addr, skip general call
    {
        //Serial.printf("boucle for %d\n", target);
        Wire.beginTransmission(target);       // slave addr
        Wire.endTransmission();               // no data, just addr
        print_scan_status(target, all);
    }
    digitalWrite(LED_BUILTIN,LOW); // LED off

    if(!found) Serial.println("No devices found.");
    
    delay(500); // delay to space out tests
}

//
// print scan status
//
void print_scan_status(uint8_t target, uint8_t all)
{
    switch(Wire.status())
    {
    case I2C_WAITING:  
        Serial.printf("Addr: 0x%02X ACK\n\r", target);
        found = 1;
        break;
    case I2C_ADDR_NAK: 
        if(all) Serial.printf("Addr: 0x%02X\n\r", target); 
        break;
    default:
        Serial.printf("default case : %d\n\r",Wire.status());
        break;
    }
}