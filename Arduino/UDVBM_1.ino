// Arduino "sketch" for use with UDVBM-1 VFO/BFO. Version. 6.21.2022
// (C) T.F. Carney (K7TFC). For use under the terms of the
// Creative Commons Attribution-ShareAlike 4.0 International license.
// See https://creativecommons.org/licenses/by-sa/4.0/legalcode

//************************************
// SET Si5351 CALIBRATION FACTOR ***** // To be determined for each Si5351 unit using NT7S's calibration sketch:
const uint32_t cal_factor = 149600;    // File-> Examples-> Etherkit Si5351-> si5351_calibration
//************************************ 

// *** EEPROM byte addresses for VFO and BFO memory ***
// -------------------------------------------------------------------------------------------
// | addr 0 | addr 1 | addr 2 | addr 3 | addr 4 | addr 5 | addr 6 | addr 7 | addr 8 | addr 9 |
// -------------------------------------------------------------------------------------------
// | VFO address write talley (uint32) | VFO addr (int)  |      BFO frequency (uint32)       |
// -------------------------------------------------------------------------------------------
// 
// --------------------------------------------------------------------------------------------------------
// | addr 10 | addr 11 | addr 12 | addr 13 | addr 14 | addr 15 | addr 16 | addr 17 | addr 18 |  addr 19  |
// -------------------------------------------------------------------------------------------------------
// | Initial VFO freq. data (tally <100K)  |  VFO freq. data (tally >100K <200K)   |  cont. to addr 1020 |
// -------------------------------------------------------------------------------------------------------
// 
// EEPROM bytes have a maximum number of reliable write cycles. 100,000 is a safe number. Because the VFO
// frequency data is saved each time it's incremented (by rotary encoder), it might not take long to use
// up a single block of 4 byte used to store the 32-bit integer. A few hundred cycles could be used in a
// single operating/listening session. Consequently, this sketch abandons each block of 4-byte VFO 
// addresses every 100,000 cycles, beginning with addresses 10-13. Subsequent addresses (14 - 17, 18 - 21,
// etc.) are saved as a 2-byte integer in addresses 4 & 5. On a ATMEGA328P-based Arduino, there are 1024
// bytes available allowing for 254 4-byte blocks to use for 100,000 writes before being abandoned for the
// next one. A total of 25,400,000 write cycles. This should be sufficient.
// 
// Because the BFO frequency will be changed *much* less frequently (perhaps as few as a dozen times), a
// single block of 4 bytes can be used for the life of the system. The addresses used for the write tally
// and VFO address bytes can also remain the same. 

#define NOP __asm__ __volatile__ ("nop\n\t")  // NOP needed to follow "skip" labels.
#define relayPin 6
#define PWMout 9
#define i2cSDA A4
#define i2cSCL A5
#define spiMISO 12
#define spiMOSI 11
#define spiSCK 13
#define encoderBTN 2
#define encoderA 4
#define encoderB 3
// For custom LCD characters:
#if defined(ARDUINO) && ARDUINO >= 100
#define printByte(args)  write(args);
#else
#define printByte(args)  print(args,BYTE);
#endif

//========================================
//=============  LIBRARIES ===============
//========================================
#include <EEPROM.h>  
#include <Wire.h>
#include <LiquidCrystal_I2C.h>    // Author: Schwartz
#include <si5351.h>               // Author: Jason NT7S. V.2.1.4
#include <ClickEncoder.h>         // Author: 0xPIT, but using the version from soligen2010's Github repo
#include <TimerOne.h>             // Required by ClickEncoder

//========================================
//======== GLOBAL DECLARATIONS ===========
//======================================== 
uint32_t lastUsedVFO; 
uint32_t lastUsedBFO;
uint32_t maxWriteCycles = 100000;
int steps[] = {10,100,1000,10000}; // Tuning steps to increment frequency (in Hz) each encoder detent.
int step = 1000;                   // Step on startup. THIS *MUST* REMAIN A REGULAR *SIGNED* INTEGER!
byte stepsPerNotch = 4;
byte detent = 0;                   // Tuning encoder. THIS *MUST* REMAIN A *BYTE* TYPE.
byte encoder = 0;
byte single_click;
byte btn = 0;                      // Tuning encoder button.
byte downcaret[] = {0x00,0x00,0x00,0x00,0x11,0x0A,0x04,0x00};

//========================================
//============ INSTANTIATIONS ============
//========================================
Si5351 si5351;
ClickEncoder tuningEncoder(encoderA,encoderB,encoderBTN,stepsPerNotch);
LiquidCrystal_I2C lcd(0x27, 16, 2);

////===================================== 
////******* FUNCTION: saveInt ********
////=====================================
void saveInt(int address, int number) {
 
  EEPROM.write(address, number >> 8);
  EEPROM.write(address + 1, number & 0xFF);
}

////===================================== 
////******* FUNCTION: readInt ********
////=====================================
int readInt(int address) {

  byte byte1 = EEPROM.read(address);
  byte byte2 = EEPROM.read(address + 1);
  return (byte1 << 8) + byte2;
}

////===================================== 
////******* FUNCTION: saveUint32 ********
////=====================================
void saveUint32(int address, uint32_t number) {

  EEPROM.write(address, (number >> 24) & 0xFF);      // These lines code the 32-bit variable into
  EEPROM.write(address + 1, (number >> 16) & 0xFF);  // 4 bytes (8-bits each) and writes them to
  EEPROM.write(address + 2, (number >> 8) & 0xFF);   // consecutive eeprom bytes starting with the
  EEPROM.write(address + 3, number & 0xFF);          // address byte. Using write() instead of update()
}                                                    // to save time and because update() won't help
                                                     // save write cycles for *each* byte of the 4-byte blocks.
////===================================
//// ***** FUNCTION: readUint32 *******
///===================================
uint32_t readUint32(int address) {

  return ((uint32_t)EEPROM.read(address) << 24) +      // These lines decode 4 consecutive eeprom bytes (8-
         ((uint32_t)EEPROM.read(address + 1) << 16) +  // bits each) into a 32-bit variable (starting with the
         ((uint32_t)EEPROM.read(address + 2) << 8) +   // address byte) and returns to the calling statement.
         (uint32_t)EEPROM.read(address + 3);           // Example: uint32_t myNumber = readUint32(0);
}

///========================================
////***** FUNCTION: saveBFO  **************
////========================================
void saveBFO() {

  saveUint32(6, lastUsedBFO);
}

////========================================
////***** FUNCTION: saveVFO  ***************
////========================================
void saveVFO() {

  uint32_t tally = readUint32(0);
  int vfoAddress = readInt(4);
  
  if(tally < maxWriteCycles) {
    tally++;
    goto skip;
  }
  else {
    vfoAddress = vfoAddress + 4;
    tally = 0;
  }

  skip:

  saveUint32(vfoAddress, lastUsedVFO);
  tally++;
  saveUint32(0, tally);

}

////========================================
////***** FUNCTION: lcdClearLine ***********
////========================================
int lcdClearLine(byte lineNum) {

    lcd.setCursor(0,lineNum);
    lcd.print("                ");
    lcd.setCursor(0,lineNum);
}

////========================================
////***** FUNCTION: displayFreqLine ********  
////========================================
void displayFreqLine(byte lineNum, uint32_t freqValue) {
 
  char padspace = 32;  // ASCII for blank space.
  String valueStr;
  String lineTag;
  String khzOnly;
  String decKHZ;
  
  if(lineNum == 0){
    lineTag = "VFO:";
  }
  else if(lineNum == 1) {
    lineTag = "BFO:";
  }
  else {                 // Only 2-line displays allowed. 
    lineNum = 0;         // Add more else-if statements for 4-line displays.
    lineTag = "Err!";
  }
    
  valueStr = String(freqValue);
  khzOnly = valueStr.substring(0, valueStr.length() - 3);                   // Takes all but the last 3 digits.
  decKHZ = valueStr.substring(valueStr.length() - 3, valueStr.length()-1);  // Takes the last 3 digits and cuts
                                                                            //  off the last digit (0-9Hz).
                                                                            // I.e., keeps only tenths and hundredths
                                                                            // of KHz (hundreds and tens of Hz).
  lcdClearLine(1);
  lcd.setCursor(0, lineNum); 

  if(valueStr.length() == 8) {
    lcd.print(lineTag + khzOnly + "." + decKHZ);  // For frequencies >=10,000KHz (no leading blank space).
  }
  else {
    lcd.print(lineTag + padspace + khzOnly + "." + decKHZ);  // For frequencies <=9,999KHz (adds leading blank space).
  }

  Serial.print("ValueStr length: "); Serial.println(valueStr.length());

  lcd.setCursor(13, lineNum);
  lcd.print("KHz");

} // End displayFreqLine()

////========================================
////***** FUNCTION: displayStepCursor ******  
////========================================
void displayStepCursor(int Step, byte lineNum) {
                  
  char stepCursor;

  if(lineNum == 1) {    // lineNum is the line on which the cursor is to appear.
    stepCursor = 94;    // ASCII "up" caret.
  }
  else if(lineNum == 0) {
    stepCursor = 0;     // 0 is the LCD custom-character location for a "down" caret 
  }
  else {
    stepCursor = 'X';  // To indicate error of lineNum. 
  }
  
  switch (Step) {
    case 10:
      lcdClearLine(lineNum);
      lcd.setCursor(11, lineNum);
      lcd.print(stepCursor);
      break;
    case 100:
      lcdClearLine(lineNum);
      lcd.setCursor(10, lineNum);
      lcd.print(stepCursor);
      break;
    case 1000:
      lcdClearLine(lineNum);
      lcd.setCursor(8, lineNum);
      lcd.print(stepCursor);
      break;
    case 10000:
      lcdClearLine(lineNum);
      lcd.setCursor(7, lineNum);
      lcd.print(stepCursor);
      break;  
  }
}

////========================================
////****FUNCTION: timerISR ***********
////========================================
void timerIsr() {

  tuningEncoder.service();      // Used by ClickEncoder for timer-based interrupts.     

  }

////========================================
////****** FUNCTION: bfoFreq() *******
////========================================
void bfoFreq() { 

    lastUsedBFO = readUint32(6);
    int bfoStep = steps[0];
    uint32_t bfoValue = lastUsedBFO;
    btn = 0;
    String valueStr;

    lcdClearLine(0);
    displayFreqLine(1, bfoValue);  // Parameters: LCD line (0 or 1), frequency value.
    displayStepCursor(bfoStep, 0);

  while (btn != 4) {               // Loop until a long press-and-release of encoder button
    // Reset button variables for this pass through the loop.
    int flag = 0;
    int item = 0;
    int encoder = 0;

    // Read tuning encoder and set Si5351 accordingly
    /////////////////////////////////////////////////
    btn = tuningEncoder.getButton();
    detent = tuningEncoder.getValue();    // ClickEncoder gets can only be done once. Getting them also clears them.
    if (detent == 255) {                  // A (-1) from the encoder rolls back the byte-type 
      encoder = -1;                       // 'encoder' byte from 0 to 255. Doing it this way  
    }                                     // eliminates otherwise testing for CW or CCW movement.
    else {
      encoder = detent;
    }
    // Skip to end of loop() unless there's change on either encoder or button
    // so LCD and Si5351 aren't constantly updating (and generating RFI).
    if (encoder == 0 && btn == 0) {
      goto skip;
    }
    else {
      bfoValue += (encoder * bfoStep);
      // Si5351 is set in 0.01 Hz increments.
      // "value" is in integer Hz.
      si5351.set_freq(bfoValue * 100, SI5351_CLK2);
      lastUsedBFO = bfoValue;
      saveBFO();  
    }
    // LCD display ////////////////////////////////
    ///////////////////////////////////////////////
    if (btn == 5 && bfoStep == steps[3]) {        
    bfoStep = steps[0];
    displayStepCursor(bfoStep, 0);                          
    }
   else if (btn == 5 && bfoStep == steps[0]) {
    bfoStep = steps[1];
    displayStepCursor(bfoStep, 0);
    }
   else if (btn == 5 && bfoStep == steps[1]) {
    bfoStep = steps[2];
    displayStepCursor(bfoStep, 0);
    }
   else if (btn == 5 && bfoStep == steps[2]) {
    bfoStep = steps[3];
    displayStepCursor(bfoStep, 0);
   }
   displayFreqLine(1,bfoValue);  //Parameters: LCD line (0 or 1), frequency value.
                             
  skip: 
  NOP;
  }
  
  // At this point (after long press-and-hold of encoder button),
  // restore the VFO display before returning.
  lcdClearLine(1);lcd.setCursor(0,1);
  displayFreqLine(0,lastUsedVFO);  //Parameters: LCD line (0 or 1), frequency value.
  displayStepCursor(step, 1); 
  
  return;
}    

////========================================
////******** FUNCTION: setup ***************
////========================================
void setup() {

  lcd.init();
  lcd.backlight();
  lcd.createChar(0, downcaret);

  Serial.begin(115200);
  Wire.begin();

  Timer1.initialize(1000);
  Timer1.attachInterrupt(timerIsr);

  tuningEncoder.setAccelerationEnabled(false);
    
  // Initialize the Si5351
  si5351.init(SI5351_CRYSTAL_LOAD_8PF, 0, 0);             
  si5351.set_correction(cal_factor, SI5351_PLL_INPUT_XO);
  si5351.drive_strength(SI5351_CLK0, SI5351_DRIVE_2MA);

  // Set default VFO & BFO frequencies for first-time use.
  byte vfoMem = (EEPROM.read(10));   // Get first byte of first VFO memory location.
  byte bfoMem = (EEPROM.read(6));
  if((vfoMem >> 6) != 0) {           // This would be true only if never used before.
    lastUsedVFO = 5000000;           // First startup default.
    saveUint32(10, lastUsedVFO);  
    saveInt(4, 10);
  }
  if((bfoMem >> 6) != 0) {
    lastUsedBFO = 11000000;           // First startup default.
    saveUint32(6, lastUsedBFO);
  }
  lastUsedVFO = readUint32(10);
  lastUsedBFO = readUint32(6);
  si5351.set_freq(lastUsedVFO * 100, SI5351_CLK0);
  si5351.set_freq(lastUsedBFO * 100, SI5351_CLK2);

  // LCD display 
  displayFreqLine(0,lastUsedVFO);  //Parameters: LCD line (0 or 1), frequency value.
  displayStepCursor(step, 1);      //Parameters: displayStepCursor(int Step, byte lineNum)
}

//========================================
//********* FUNCTION: (main)loop *********
//========================================
void loop() {

  lastUsedVFO = readUint32(10);

  //Serial.println("lastUsedVFO: "); Serial.println(lastUsedVFO);
  //delay(1000);
  uint32_t vfoValue = lastUsedVFO;
  
  // Reset button variables for this pass through the loop.
  int flag = 0;
  int item = 0;
  int encoder = 0;

  // Read tuning encoder and set Si5351 accordingly
  /////////////////////////////////////////////////
  btn = tuningEncoder.getButton();
  detent = tuningEncoder.getValue();  // ClickEncoder "gets" can only be done once. 
                                    // Getting them also clears them.

  if (detent == 255) {                // A (-1) from the encoder rolls back the byte-type 
    encoder = -1;                     // 'encoder' byte from 0 to 255. Doing it this way  
  }                                   // eliminates otherwise testing for CW or CCW movement.
  else {
    encoder = detent;
   }

  // Skip to end of loop() unless there's change on either encoder or button
  // so LCD and Si5351 aren't constantly updating (and generating RFI).
  if (encoder == 0 && btn == 0) {
    goto skip;
  }
  else {
    vfoValue += (encoder * step);
    Serial.print("vfoValue: "); Serial.println(vfoValue);
    // Si5351 is set in 0.01 Hz increments.
    // "vfoValue" is in integer Hz.
    si5351.set_freq(vfoValue * 100, SI5351_CLK0);
    lastUsedVFO = vfoValue;
    Serial.print("lastUsedVFO: "); Serial.println(lastUsedVFO);
    saveVFO();
  }
  // LCD display ///////////////////
  displayFreqLine(0,lastUsedVFO);             //Parameters: LCD line (0 or 1), frequency value.

  // Button activity on tuning encoder        // ClickEncoder button returns: 4==released (after long press), 5==clicked, 6==double-clicked.   
   if (btn == 4) {                            // I'm sure some would say I should use switch-case here. I don't care.
    bfoFreq();                                // Long press-and-release will call BFO-setting function.
   }
   else if(btn == 5 && step == steps[3]) {    // These else-if statements respond to single (short click) button pushes to step-through   
    step = steps[0];                          // the tuning increments (10Hz, 100Hz, 1KHz, 10KHz) for each detent of the tuning encoder
    displayStepCursor(step, 1);               // and moves the cursor caret to the corresponding digit. A short click on 10KHz loops back
   }                                          //  to 10Hz. The default step is 1KHz. The bfoFreq() function uses the same structure.
   else if (btn == 5 && step == steps[0]) {
    step = steps[1];
    displayStepCursor(step, 1);
    }
   else if (btn == 5 && step == steps[1]) {
    step = steps[2];
    displayStepCursor(step, 1);
    }
   else if (btn == 5 && step == steps[2]) {
    step = steps[3];
    displayStepCursor(step, 1);
   }

skip:   // This label is where the loop goes if there are no inputs.
NOP;    // C/C++ rules say a label must be followed by something. This "something" does nothing.

}       // closes main loop() 
   
          
