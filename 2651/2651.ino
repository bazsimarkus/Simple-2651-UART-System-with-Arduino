/**
 * @file 2651.ino
 * @author Balazs Markus
 * @brief Arduino code for interacting with the 2651 PCI UART chip and transmitting a test string with 9600 baud, 8-bit data, no parity, and 1 stop bit, once every second.
 * @details This code provides a simple test environment for a 2651 Programmable Communications Interface (PCI) UART chip. It is specifically tested with an SCN2651 chip but it is forward compatible with all serial ICs based on the 2651 architecture (For example, the 2661).
 *
 * Pinout of the 2651 PCI UART chip (DIP-28):
 *
 *         ┌───────○───────┐
 *      D2 │1            28│ D1
 *      D3 │2            27│ D0
 *     RxD │3            26│ VCC 
 *     GND │4            25│ RxCN 
 *      D4 │5            24│ DTRN  
 *      D5 │6     2651   23│ RTSN
 *      D6 │7     UART   22│ DSRN 
 *      D7 │8            21│ RESET
 *    TxCN │9            20│ BRCLK
 *      A1 │10           19│ TxD   
 *     CEN │11           18│ TxEMTN_DSCHGN
 *       A0│12           17│ CTSN  
 *     RWN │13           16│ DCDN   
 *  RxRDYN │14           15│ TxRDYN   
 *         └───────────────┘
 *
 * Pin descriptions of the 2651 PCI UART chip:
 *
 * ---------------------------------------------------------------------------------
 * | Pin Number | Name            | Type    | Description                          |
 * |------------|-----------------|---------|--------------------------------------|
 * | 1          | D2              | In/Out  | Data Bit 2                           |
 * | 2          | D3              | In/Out  | Data Bit 3                           |
 * | 3          | RxD             | In      | Receive Data                         |
 * | 4          | GND             | -       | Ground                               |
 * | 5          | D4              | In/Out  | Data Bit 4                           |
 * | 6          | D5              | In/Out  | Data Bit 5                           |
 * | 7          | D6              | In/Out  | Data Bit 6                           |
 * | 8          | D7              | In/Out  | Data Bit 7                           |
 * | 9          | TxCN            | In      | Transmit Clock  (Active Low)         |
 * | 10         | A1              | In      | Address Bit 1                        |
 * | 11         | CEN             | In      | Chip Enable  (Active Low)            |
 * | 12         | A0              | In      | Address Bit 0                        |
 * | 13         | RWN             | In      | Read/Write (Active Low)              |
 * | 14         | RxRDYN          | Out     | Receiver ready (Active Low)          |
 * | 15         | TxRDYN          | Out     | Transmitter ready (Active Low)       |
 * | 16         | DCDN            | In      | Data Carrier Detect (Active Low)     |
 * | 17         | CTSN            | In      | Clear To Send (Active Low)           |
 * | 18         | TxEMTN_DSCHGN   | Out     | Serialization complete (Active Low)  |
 * | 19         | TxD             | Out     | Transmit Data                        |
 * | 20         | BRCLK           | In      | Baud Rate Clock (for internal clock) |
 * | 21         | RESET           | In      | Reset                                |
 * | 22         | DSRN            | In      | General Purpose Input (Active Low)   |
 * | 23         | RTSN            | Out     | Request To Send (Active Low)         |
 * | 24         | DTRN            | Out     | Data Terminal Ready (Active Low)     |
 * | 25         | RxCN            | In      | Receiver Clock  (Active Low)         |
 * | 26         | Vcc             | -       | 5V Positive Supply                   |
 * | 27         | D0              | In/Out  | Data Bit 0                           |
 * | 28         | D1              | In/Out  | Data Bit 1                           |
 * ---------------------------------------------------------------------------------
 *
 * Components needed for the test circuit:
 * - 2651 PCI UART chip
 * - Arduino board (Uno, Nano, both works, the important thing is that it should operate on 5V voltage)
 * - USB-TTL Serial converter for testing (FTDI for example)
 *
 * The circuit works as follows:
 * The Arduino sends 8-bit ASCII characters to the PCI UART chip, then the chip converts it to serial data (9600 baud, 8-bit data, no parity, and 1 stop bit).
 * The parameters like the baud rate, parity bit, etc., are initialized by the Arduino at the beginning of the procedure.
 * The text to send is stored in the PROGMEM section of the microcontroller memory, allowing the Arduino to send long messages without any stack problems.
 * No external crystal is needed, as the program utilizes Timer2 of the ATmega328P microcontroller on the Arduino UNO board to generate the clock signal, and achieve the desired standard 9600 Baud rate.
 *
 * How to use:
 * 1. Connect your Arduino board to the 2651 PCI UART chip based on the included schematics.
 * 2. Upload the 2651.ino source code to the Arduino board.
 * 3. Connect the USB-TTL Serial converter to the PC and to the TxD/RxD pins of the 2651
 * 4. Open a serial terminal program (for example, RealTerm), select the COM port of your USB-Serial converter, and set up the terminal to use 9600 baud, 8-bit data, no parity, and 1 stop bit.
 * 5. You should be receiving the string "TESTUART" once every second.
 */

#include <avr/io.h>
#include <avr/pgmspace.h>
#include <string.h>

// Pin Definitions
#define BAUDCLK 11  // Baud Rate Clock

#define ADDRESS0 A0  // Address 0 for register selection
#define ADDRESS1 A1  // Address 1 for register selection
#define RESET A2     // Reset signal

#define CEN A3   // Chip enable negated
#define RWN A4   // Read-write (high when write, low when read)
#define DSRN A5  // General purpose input which can be used for data set ready or ring indicator condition. Its complement appears as status register bit SR7.

#define DCDN 10  // Data carrier detect input. Must be low in order for the receiver to operate. Its complement appears as status register bit SR6.
#define CTSN 12  // Clear to send input. Must be low in order for the transmitter to operate. If it goes high during transmission, the character in the transmit shift register will be transmitted before termination.

#define D0 2  // Data bus bit 0
#define D1 3  // Data bus bit 1
#define D2 4  // Data bus bit 2
#define D3 5  // Data bus bit 3
#define D4 6  // Data bus bit 4
#define D5 7  // Data bus bit 5
#define D6 8  // Data bus bit 6
#define D7 9  // Data bus bit 7

#define LED_OUTPUT 13  // Arduino onboard LED to indicate status

// Timing constants
#define DELAY_SHORT 1
#define DELAY_MEDIUM 50
#define DELAY_LONG 200

// Test string definition
const char testString[] PROGMEM = { "TESTABCD" };  // We store the string in the PROGMEM, this enables us to store very long strings

/**
 * @brief Arduino setup function.
 *
 * @details This function is called once when the microcontroller is powered on or reset. It is used to initialize
 * the microcontroller's hardware and peripherals, such as pins, timers, serial communication, etc.
 *
 * @note The setup function runs only once after power-up or reset. It should be used to perform
 * initialization tasks and configuration settings.
 */
void setup() {
  pinMode(LED_OUTPUT, OUTPUT);
  setUartPinsToOutput();
  initClocks();
  initUart();  // Initializing 2651 PCI UART
}

/**
 * @brief Arduino main loop function.
 *
 * @details The loop function is called repeatedly as long as the microcontroller is powered on.
 * It is used to implement the main program logic, perform tasks, and handle events.
 *
 * @note The loop function runs in an infinite loop and should be used to execute the main
 * program logic or perform tasks that need to be repeated continuously.
 */
void loop() {
  // Print the string with the 2651 PCI UART
  printStringToUart(testString);

  // Blink onboard LED
  digitalWrite(LED_OUTPUT, HIGH);
  delay(DELAY_MEDIUM);
  digitalWrite(LED_OUTPUT, LOW);

  // Wait one second
  delay(1000);
}

/**
 * @brief Sets all data bus pins and control pins to output mode.
 *
 * @details This function configures all the data bus pins and control pins to be used as outputs.
 * It sets the direction of the pins accordingly to enable data transmission and control signal handling.
 */
void setUartPinsToOutput() {
  // Arduino D pins
  pinMode(D0, OUTPUT);
  pinMode(D1, OUTPUT);
  pinMode(D2, OUTPUT);
  pinMode(D3, OUTPUT);
  pinMode(D4, OUTPUT);
  pinMode(D5, OUTPUT);
  pinMode(D6, OUTPUT);
  pinMode(D7, OUTPUT);
  pinMode(DCDN, OUTPUT);
  pinMode(BAUDCLK, OUTPUT);
  pinMode(CTSN, OUTPUT);
  // Arduino A pins
  pinMode(ADDRESS0, OUTPUT);
  pinMode(ADDRESS1, OUTPUT);
  pinMode(RESET, OUTPUT);
  pinMode(CEN, OUTPUT);
  pinMode(RWN, OUTPUT);
  pinMode(DSRN, OUTPUT);
}

/**
 * @brief Set data bus pins to high impedance (not used).
 *
 * @details This function sets the data bus pins to high impedance,
 * but it is not used in the current implementation.
 */
void setDataBusToHighZ() {
  pinMode(D0, INPUT);  // Data Bus D0 - Input (High-Z)
  pinMode(D1, INPUT);  // Data Bus D1 - Input (High-Z)
  pinMode(D2, INPUT);  // Data Bus D2 - Input (High-Z)
  pinMode(D3, INPUT);  // Data Bus D3 - Input (High-Z)
  pinMode(D4, INPUT);  // Data Bus D4 - Input (High-Z)
  pinMode(D5, INPUT);  // Data Bus D5 - Input (High-Z)
  pinMode(D6, INPUT);  // Data Bus D6 - Input (High-Z)
  pinMode(D7, INPUT);  // Data Bus D7 - Input (High-Z)
}

/**
 * @brief Write data to the data bus.
 *
 * @details This function sets the data bus pins in bulk. It uses the bitRead
 * function to retrieve each bit of the input parameter.
 *
 * @param data The data to be written to the data bus.
 */
void writeDataBus(int data) {
  digitalWrite(D0, bitRead(data, 0));
  digitalWrite(D1, bitRead(data, 1));
  digitalWrite(D2, bitRead(data, 2));
  digitalWrite(D3, bitRead(data, 3));
  digitalWrite(D4, bitRead(data, 4));
  digitalWrite(D5, bitRead(data, 5));
  digitalWrite(D6, bitRead(data, 6));
  digitalWrite(D7, bitRead(data, 7));
}

/**
 * @brief Write a character to the SCN2651 Programmable Communications Interface (PCI).
 *
 * @details This function writes an 8-bit character to the SCN2651 PCI for transmission. This function can only be called after initialization!
 *
 * @param data The character to be transmitted.
 *
 * This function uses the same sequence for register writing as the initialization function, it just writes the transmit register at address 00.
 */
void writeCharToUart(int data) {
  // Write transmit data register
  delay(DELAY_SHORT);      // Short delay
  digitalWrite(ADDRESS0, LOW);   // Set A0 line to select transmit register
  digitalWrite(ADDRESS1, LOW);   // Set A1 line to select transmit register
  delay(DELAY_SHORT);      // Short delay
  digitalWrite(RWN, HIGH);  // Set R/W line to write mode
  delay(DELAY_SHORT);      // Short delay
  digitalWrite(CEN, LOW);   // Disable the SCN2651 PCI chip (enable only after bus has been properly set)
  delay(DELAY_SHORT);      // Short delay

  // Register Setting
  writeDataBus(data);  // Write configuration data to transmit data register

  delay(DELAY_SHORT);      // Short delay
  digitalWrite(CEN, HIGH);  // Disable the SCN2651 PCI chip (enable only after bus has been properly set)

  delay(DELAY_SHORT);     // Short delay
  digitalWrite(RWN, LOW);  // Set R/W line to read mode
  delay(DELAY_SHORT);     // Short delay
}

/**
 * @brief Print a string to UART.
 * 
 * @details This function prints a string to the UART for transmission.
 * 
 * @param str The string to be transmitted. The string has to be zero terminated!
 */
void printStringToUart(const char* str) {
  char myChar;
  for (int j = 0; j < strlen(str); j++) {
    myChar = pgm_read_byte_near(str + j);
    writeCharToUart(myChar);
  }
}

/**
 * @brief Blink the onboard LED to indicate status.
 * 
 * @details This function blinks the onboard LED to indicate status.
 * It is used for visual indication in the circuit.
 */
void blinkLed() {
  for (int i = 0; i < 4; i++) {
    digitalWrite(LED_OUTPUT, HIGH);
    delay(DELAY_LONG);
    digitalWrite(LED_OUTPUT, LOW);
    delay(DELAY_LONG);
  }
}

/**
 * @brief Initializes the baud clock for UART communication.
 *
 * @details This function sets up the Timer2 registers to generate the UART baud clock.
 * It sets the timer in Fast PWM mode with a prescaler of 1, and configures the compare match
 * to toggle pin 11 at the desired baud rate (153.85kHz, which is roughly 9600*16)
 */
void initClocks() {
  // Timer2 register setup
  // Bits 7:6 toggle pin 11 on a compare match
  // Bits 1:0 and bit 3 in TCCR2B select Fast PWM mode, with OCRA for the TOP value
  TCCR2A = 0b01000011;
  // Bits 2:0 are the prescaler setting, this is a 1 prescaler
  TCCR2B = 0b00001001;
  // This is the value that, when the timer reaches it, will toggle the output pin and restart.
  // A value of 51 in this case gives a 153.85kHz output
  OCR2A = 51;
}

/**
 * @brief Initializes the SCN2651 (PCI UART) chip for UART communication.
 *
 * @details This function configures the control lines and registers of the SCN2651 chip for proper operation.
 *          It sets the two mode registers (MR1 and MR2), and the command register (CR) to establish the baud rate divisor and other parameters required for communication.
 *          The SCN2651 Programmable Communications Interface (PCI) serves as a critical intermediary between a computer
 *          and external systems, facilitating the conversion of data between serial and parallel forms. The PCI's hardware 
 *          consists of three main sections: the CPU side, the transmitter side, and the receiver side.

 * The SYN1, SYN2, and DLE registers are accessed by performing write operations with the conditions A1 = 0, A0 = 1, and R/W = 1.
 * The first operation loads the SYN1 register. The next loads the SYN2 register, and the third loads the DLE register.
 * Reading or loading the mode registers is done in a similar manner. The first write (or read) operation addresses mode register 1,
 * and a subsequent operation addresses mode register 2. If more than the required number of accesses are made,
 * the internal sequencer recycles to point at the first register. The pointers are reset to SYN1 register and mode register 1
 * by a RESET input or by performing a “read command register” operation, but are unaffected by any other read or write operation.
 *
 * The SCN2651 register formats are summarized in the following lists. Mode registers 1 and 2 define the general operational characteristics of the PCI,
 * while the command register controls the operation within this basic framework.
 * The PCI indicates its status in the status register. These registers are cleared when a RESET input is applied.
 *
 * Mode Register 1 (MR1):
 * - MR17: Number of SYN characters (0 = Double SYN, 1 = Single SYN)
 * - MR16: Transparency control (0 = Normal, 1 = Transparent)
 * - MR15: Parity type (0 = Odd, 1 = Even)
 * - MR14: Parity control (0 = Disabled, 1 = Enabled)
 * - MR13-MR12: Character length (00 = 5 Bits, 01 = 6 Bits, 10 = 7 Bits, 11 = 8 Bits)
 * - MR11-MR10: Mode and baud rate factor (00 = Synchronous 1X rate, 01 = Asynchronous 1X rate, 10 = Asynchronous 16X rate, 11 = Asynchronous 64X rate)
 *   Note: Baud rate factor in asynchronous applies only if external clock is selected. Factor is 16X if internal clock is selected. Mode must be selected (MR11, MR10) in any case
 *
 * Mode Register 2 (MR2):
 * - MR27-MR26: Not used
 * - MR25: Transmitter clock source (0 = External, 1 = Internal)
 * - MR24: Receiver clock source (0 = External, 1 = Internal)
 * - MR23-MR20: Baud rate selection (0000 = 50 Baud, ..., 1111 = 19,200 Baud)
 *
 * Command Register (CR):
 * - CR7-CR6: Operating mode (00 = Normal operation, 01 = Echo/Stripping mode, 10 = Local Loopback, 11 = Remote Loopback)
 * - CR5: Request to send (0 = High, 1 = Low)
 * - CR4: Reset error flags (0 = Normal, 1 = Reset error flags)
 * - CR3: Transmit control (Async: Force break, Sync: Send DLE)
 * - CR2: Receive control (0 = Disable, 1 = Enable)
 * - CR1: Data terminal ready (0 = High, 1 = Low)
 * - CR0: Transmit control (0 = Disable, 1 = Enable)
 *
 * This function initializes the PCI UART by configuring the mode registers and the command register to set the clock divisor, data format, 
 * and interrupt control. It sets the clock divisor to 16x, selects 8 data bits, no parity, 1 stop bit (8n1) format, and 
 * set the default settings for the transmitter and the receiver. Additionally, it configures the PCI's control lines for proper 
 * operation by setting appropriate digital signals.
 *
 */
void initUart() {
  delay(DELAY_SHORT);       // Short delay
  digitalWrite(CEN, HIGH);  // Chip enable high

  delay(DELAY_SHORT);         // Short delay
  digitalWrite(RESET, LOW);   // Reset low
  delay(DELAY_SHORT);         // Short delay
  digitalWrite(RESET, HIGH);  // Reset high
  delay(DELAY_SHORT);         // Short delay
  digitalWrite(RESET, LOW);   // Reset low

  delay(DELAY_SHORT);        // Short delay
  digitalWrite(DSRN, HIGH);  // Data set ready high

  delay(DELAY_SHORT);        // Short delay
  digitalWrite(DCDN, HIGH);  // Data carrier detect high

  delay(DELAY_SHORT);       // Short delay
  digitalWrite(CTSN, LOW);  // Clear to send low

  delay(DELAY_SHORT);            // Short delay
  digitalWrite(ADDRESS0, LOW);   // Address line 0 low
  digitalWrite(ADDRESS1, HIGH);  // Address line 1 high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(RWN, HIGH);       // Read/write high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(CEN, LOW);        // Chip enable low
  delay(DELAY_SHORT);            // Short delay

  writeDataBus(0b01001110);  // Write to data bus (Mode Register 1 configuration)

  delay(DELAY_SHORT);       // Short delay
  digitalWrite(CEN, HIGH);  // Chip enable high

  delay(DELAY_SHORT);      // Short delay
  digitalWrite(RWN, LOW);  // Read/write low

  delay(DELAY_SHORT);            // Short delay
  digitalWrite(ADDRESS0, LOW);   // Address line 0 low
  digitalWrite(ADDRESS1, HIGH);  // Address line 1 high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(RWN, HIGH);       // Read/write high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(CEN, LOW);        // Chip enable low
  delay(DELAY_SHORT);            // Short delay

  writeDataBus(0b00000000);  // Write to data bus (Mode Register 2 configuration)

  delay(DELAY_SHORT);       // Short delay
  digitalWrite(CEN, HIGH);  // Chip enable high

  delay(DELAY_SHORT);      // Short delay
  digitalWrite(RWN, LOW);  // Read/write low

  delay(DELAY_SHORT);            // Short delay
  digitalWrite(ADDRESS0, HIGH);  // Address line 0 high
  digitalWrite(ADDRESS1, HIGH);  // Address line 1 high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(RWN, HIGH);       // Read/write high
  delay(DELAY_SHORT);            // Short delay
  digitalWrite(CEN, LOW);        // Chip enable low
  delay(DELAY_SHORT);            // Short delay

  writeDataBus(0b00000101);  // Write to data bus (Command Register configuration)

  delay(DELAY_SHORT);       // Short delay
  digitalWrite(CEN, HIGH);  // Chip enable high

  delay(DELAY_SHORT);      // Short delay
  digitalWrite(RWN, LOW);  // Read/write low

  delay(DELAY_SHORT);  // Short delay

  blinkLed();  // Blink LED to indicate initialization complete
}
