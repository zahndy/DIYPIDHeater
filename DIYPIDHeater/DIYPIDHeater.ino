// PID Heater v0.2 
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <FastPID.h>

const int MinTemp = 0;
const int MaxTemp = 128;
int TargetTemperature;     // changed to 8bit value
int EepRomTempAdress = 0;
//inputs
int UpBtnState;   //buttons
int UplastButtonState = LOW;
int DownBtnState;
int DownlastButtonState = LOW;
int PwrSwBtnState;
int PwrSwlastButtonState = LOW;
unsigned long UplastDebounceTime = 0;
unsigned long DownlastDebounceTime = 0;
unsigned long PwrSwlastDebounceTime = 0;
unsigned long debounceDelay = 5;
unsigned long previousMillis = 0;
// A4 A5 = lcd SDA SCL
const int ThermistorPin = A1;
const int PwrSwPin = 12;
const int BtnUpPin = 4;
const int BtnDownPin = 6;
// outputs
const int MosFetPin = 9;
const int PwrRelayPin = 2;

bool preHeat = false;
bool isPreHeating = false;

bool on = false;

int animationIndex =0;

//Thermistor
int Vo;
float R1 = 10000; // change to 100K?
float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

//pid
float Kp=1.89, Ki=0.07, Kd=0.01, Hz=2; 
// Manual Kp=2.89, Ki=0.67, Kd=0.01, Hz=2;
// Autotune Kp=1.89, Ki=0.07, Kd=0.01, Hz=2;
int output_bits = 8;
bool output_signed = false;
FastPID myPID(Kp, Ki, Kd, Hz, output_bits, output_signed);

//LCD
LiquidCrystal_I2C lcd(0x38,16,2);
byte cornerChar[8] = {
  0b00100,
	0b00100,
	0b00100,
	0b00111,
	0b00000,
	0b00000,
	0b00000,
	0b00000
};
byte backslashChar[8] = {
	0b00000,
	0b10000,
	0b01000,
	0b00100,
	0b00010,
	0b00001,
	0b00000,
	0b00000
};

void setup() {
  if (myPID.err()) {
    //Serial.println("There is a configuration error!");
    for (;;) {}
  }
  pinMode(MosFetPin, OUTPUT);
  pinMode(PwrRelayPin, OUTPUT);
  lcd.init();
  lcd.backlight();
  PrepareLCD();
  lcd.createChar(0, cornerChar);
  lcd.createChar(1, backslashChar);
  //TargetTemperature = 450; // default off state 18C for initial flash, use eeprom after
  ReadTempFromRom();
  preHeat = true;
  isPreHeating = false;
  on = digitalRead(PwrSwPin);
  delay(200);
}

void loop() {
  unsigned long currentMillis = millis();
  Buttons(currentMillis);
  // update every 500ms
  if (currentMillis - previousMillis >= 500) {
    previousMillis = currentMillis;
    PidStep();
  }
}

void Buttons(unsigned long currentMillis)
{
  int UPreading = digitalRead(BtnUpPin);
  int DOWNreading = digitalRead(BtnDownPin);
  int PwrSwReading = digitalRead(PwrSwPin);
  // Button Detection with debounce
  if (UPreading != UplastButtonState) {
    // reset the debouncing timer
    UplastDebounceTime = currentMillis;
  }
   if (DOWNreading != DownlastButtonState) {
    DownlastDebounceTime = currentMillis;
  }
  if (PwrSwReading != PwrSwlastButtonState) {
    PwrSwlastDebounceTime = currentMillis;
  }
  if ((currentMillis - UplastDebounceTime) > debounceDelay) {
    if (UPreading != UpBtnState) {
      UpBtnState = UPreading;
      if (UpBtnState == HIGH) {
        if(AnalogToTmp(TargetTemperature)<(MaxTemp-5)){
          TargetTemperature+=5;
          UpdateTempRom();}
        
      }
    }
  }
  UplastButtonState = UPreading;
  if ((currentMillis - DownlastDebounceTime) > debounceDelay) {
    if (DOWNreading != DownBtnState) {
      DownBtnState = DOWNreading;
      if (DownBtnState == HIGH) {
        if(AnalogToTmp(TargetTemperature)>(MinTemp+5)){
          TargetTemperature-=5;
          UpdateTempRom();}
        
      }
    }
  }
  DownlastButtonState = DOWNreading;
  if ((currentMillis - PwrSwlastDebounceTime) > debounceDelay) { //flip switch
    if (PwrSwReading != PwrSwBtnState) {
      PwrSwBtnState = PwrSwReading;
      if (PwrSwBtnState == HIGH) {
        // power on
        on =1;
        digitalWrite(PwrRelayPin, HIGH);
        ReadTempFromRom();
        myPID.configure(Kp, Ki, Kd, Hz, output_bits, output_signed);
        preHeat = true;
        isPreHeating = false;
        PidStep();
        
      }
      else{
        // power off
        on =0;
        UpdateTempRom();
        digitalWrite(PwrRelayPin, LOW);
        //TargetTemperature = 450; // default off state 18C
        analogWrite(MosFetPin, 0);
        myPID.clear();
      }
    }
  }
  PwrSwlastButtonState = PwrSwReading;
}

void PrepareLCD()
{
  lcd.clear();
  lcd.setCursor(0,1);
  lcd.write(byte(0));
  lcd.setCursor(1,1);
  //lcd.write(byte(1));
  lcd.print((char)126);
  lcd.setCursor(8,1);
  lcd.print(" PWM: ");
}

int CurrentTemp()
{
    return analogRead(ThermistorPin);
}

float AnalogToTmp(int EightBit)
{
  float R2 = R1 * (1023.0 / (float)EightBit - 1.0);
  float logR2 = log(R2);
  float T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  T = T - 273.15;
  return T;
}

void PidStep()
{
  int T = CurrentTemp();
  int setpoint = TargetTemperature;
  int feedback = T;
  
  lcd.setCursor(2,1);//Display
  lcd.print(String(AnalogToTmp(setpoint),0)+(char)223+"C");
  lcd.setCursor(0,0); 
  if(on)
  {
    if(preHeat)
    {
      lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "  Warm up ");
      if(!isPreHeating)
      {
        SetTemp(255);
        isPreHeating = true;
      }
      lcd.setCursor(13,1);
      switch (animationIndex) {
        case 0:
          lcd.print(".");
          break;
        case 1:
          lcd.print("..");
          break;
        case 2:
          lcd.print("...");
          break;
        default:
          lcd.print("   ");
          break;
      }
      if(T>setpoint)
      {
        preHeat = false;
        isPreHeating = false;
      }
    }
    else
    {
      
      switch (animationIndex) {
        case 0:
          lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "  Ready   ");
          lcd.setCursor(15,0); 
          lcd.write(byte(1));
          break;
        case 1:
          lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "  Ready  |");
          break;
        case 2:
          lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "  Ready  /");
          break;
        default:
          lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "  Ready  -");
          break;
      }
      uint8_t output = myPID.step(setpoint, feedback);
      lcd.setCursor(13,1);
      if(output<100){
        if(output<10){
          lcd.print("  "+String(output));
          }
        else{
          lcd.print(" "+String(output));
          }
      }
      else{
        lcd.print(String(output));
      }
      SetTemp(output); //mosfet
    }
    animationIndex++;
    if(animationIndex>3)
    {
      animationIndex=0;
    }
  }
  else
  {
    lcd.print(String(AnalogToTmp(T),1)+(char)223+"C" + "   OFF    "); 
  }
}

void SetTemp(uint8_t PidOutput)  // arduino output = 0-5v, needed = >5
{
  if (PwrSwBtnState == HIGH) {
  analogWrite(MosFetPin, PidOutput);
  }
}

void ReadTempFromRom()
{
  TargetTemperature = EEPROMReadlong(EepRomTempAdress);
}

void UpdateTempRom()
{
  EEPROMWritelong(EepRomTempAdress, TargetTemperature);   
}

void EEPROMWritelong(int address, long value)
{
  //Decomposition from a long to 4 bytes by using bitshift.
  //One = Most significant -> Four = Least significant byte
  byte four = (value & 0xFF);
  byte three = ((value >> 8) & 0xFF);
  byte two = ((value >> 16) & 0xFF);
  byte one = ((value >> 24) & 0xFF);

  //Write the 4 bytes into the eeprom memory.
  EEPROM.write(address, four);
  EEPROM.write(address + 1, three);
  EEPROM.write(address + 2, two);
  EEPROM.write(address + 3, one);
}
long EEPROMReadlong(long address)
{
  //Read the 4 bytes from the eeprom memory.
  long four = EEPROM.read(address);
  long three = EEPROM.read(address + 1);
  long two = EEPROM.read(address + 2);
  long one = EEPROM.read(address + 3);

  //Return the recomposed long by using bitshift.
  return ((four << 0) & 0xFF) + ((three << 8) & 0xFFFF) + ((two << 16) & 0xFFFFFF) + ((one << 24) & 0xFFFFFFFF);
}
