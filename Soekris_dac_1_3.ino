/***************************************************************************************************************
 * Control the Soekris dam1021 by an Arduino pro with serial line and relay 
 *
 * Project Page: 
 * v1.3   30/08/2026 : power control by gemini
 * v1.2   04/04/2026 : Filter change with rotary switch code added
 *                     rotary switch operation : polling --> interrupt mode
 * v1.1   28/03/2026 : LCD display logic changed
 * v1.0   28/03/2026 : major bug (volume control) removed
 * v0.5a  13/09/2021 : minor revisions
 * v0.5   05/09/2021 : rewrite codes
 * v0.2   26/08/2021 : IR remote control changed to B&W A5 remote
 * v0.1   10/08/2021 : modified source from dimdim's blog
 *
 *  1. Arduino and Soekris DAC connected by serial port
 * 
 *  2. Arduino has 
 *      IR receiver
 *          Volume control
 *          Input selection
 *          Filter selection
 *      Rotary encoder            
 *          rotary : volume control
 *          button : input switch
 *      LCD display (16*2 dot matrix)
 *      1 * 2channel relay switch for input selection
 *      
 *  3. Operation
 *     - body control pannel
 *      > Source selection : short preesing push button 
 *      > Volume control : rotary switch up/down
 *      > Filter control : rotary switch up/down after push button Long press
 *      > Power control : dedicated tact switch
 *     - remote control (b&w A5 remote control only)
 *      > power control with power key
 *      > volume control with up/down key
 *      > source control with push button
 *      > filter control with left/right button

 *  4. Further improvement
 *      remote control power switch handling (just for utilize power key on the remote control ^^)
 *      
 **************************************************************************************************************/


#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <stdlib.h>
#include <IRremote.h>

//
// pin definition
// rotary switch pins
#define CLK     3    // pin 3 is CLK 
#define DT      4    // pin 4 is DT
#define SW      5    // pin5 is SW

#define POWER_SW 6   // Power Tact Switch pin (GND연결, 내부 풀업 사용)

//source selection pins
#define COAXSEL 8    //select COAXIAL if GND
#define OPTSEL  9    //select OPT if GND

#define POWERPIN 11  // Power Relay pin.

#define NUMSRC  3     //USB, COAXIAL, OPTICAL
#define NUMFILTER 5   //Number of Filters :  NULL, Linear, Mixed, Min, Soft
#define NUMDISP   5
#define NUM_SR  11  //number of predefined sampling frequencies

// Remote control codes. They correspond to an old remote that I use for testing - change to match your remote's.
// following key sequences are for B&W A5 remote control
// remote key sequence : 1st touch will generate 3bytes code, and following touch will generate 2bytes code (lower 2 bytes code)

#define POWER_CODE    0xD20C    //power key : 0x01d20c -> 0xd20c -> 0x01d20c -> 0xd20c ...
#define VOLDOWN_CODE  0xD211    //volume up
#define VOLUP_CODE    0xD210    //volume down
#define FILTUP_CODE   0xD220    //filter up
#define FILTDOWN_CODE 0xD221    //filter down
#define SOURCE_CODE   0xD286    //source key

enum Sources { USB, COAX, OPT, AUTO };  //input label
enum SignalTypes { PCM, DSD };          //input signal type
enum RotaryMode { VolumeControl, FilterControl };      //rotary switch operation mode

//
// --------------------------------------- Return codes from DAC
typedef struct {
  char  code[2];      //return code
  void  (*func)();    //pointers to return code processing function
} returnCode;

// --------------------------------------- IR code from remote control
typedef struct {
  long  irCode;       //IR received command
  void  (*func)();    //pointers to IR command processing function
} irCommand;

// --------------------------------------- Sampling rates
typedef struct {
  int rxSR;       //sampling rates from soekris DAC
  float orgSR;    //original value
  SignalTypes signalType; //pcm or dsd
} SamplingRates;

typedef struct {      //v1.1 
  bool *flag;         //need to display
  void  (*func)();    //pointers to display functions
} DisplayControl;


//process codes from DAC
//
void rcodeRelCode();
void rcodeSRate();
void rcodeSource();
void rcodeVolume();
void rcodeFilter();
void rcodePower();

//codes from Remote control
//
void powerCode();
void volUpCode();
void volDnCode();
void sourceCode();
void filtUpCode();
void filtDnCode();

//process codes from DAC
returnCode  rcode[] = {
  { "R", rcodeRelCode },    //firmware release information  R2.34 r2.34
  { "I", rcodeSource  },    //current input source    I0
  { "L", rcodeSRate   },    //sampling rate     L044
  { "F", rcodeFilter  },    //filter bank       F4..F7
  { "P", rcodePower   },    //power on          PN
  { "V", rcodeVolume  }     //gain (volume)     V+00
};

//codes from Remote control
irCommand ircmd[] = {
  { POWER_CODE,   powerCode },  //power button
  { VOLUP_CODE,   volUpCode },  //volume up
  { VOLDOWN_CODE, volDnCode },  //volume down
  { SOURCE_CODE,  sourceCode }, //source change
  { FILTUP_CODE,  filtUpCode }, //filter change - up
  { FILTDOWN_CODE,filtDnCode }  //filter change - down
};

//display functions
//
void disp_SR();
void disp_type();
void disp_volume();
void disp_filter();
void disp_source();

bool display_SR     = true;         //need refresh display
bool display_type   = true;         //display current signal type (PCM, DSD) 
bool display_Vol    = true;         //display current Volume  
bool display_Filt   = true;         //display current filter
bool display_Source = true;

DisplayControl dispControl[NUMDISP] = { //display control data
  { &display_SR,   disp_SR},
  { &display_type, disp_type},
  { &display_Vol,  disp_volume},
  { &display_Filt, disp_filter},
  { &display_Source,disp_source}
};

SamplingRates SRTable[NUM_SR] = {
  {44,  44.1, PCM},
  {48,  48,   PCM},
  {88,  88.2, PCM},
  {96,  96,   PCM},
  {176, 176.4,PCM},
  {192, 192,  PCM},
  {352, 352.8,PCM},
  {384, 384,  PCM},
  {2,   64,   DSD},
  {5,   128,  DSD},
  {11,  256,  DSD}
};

String sources[NUMSRC] = { 
  "USB ",        // source names
  "COAX", 
  "OPT "
//  "AUTO" 
}; 

String filters[NUMFILTER] = { 
  "NULL  ",      //filter names
  "Linear",      // The name of the filter F4
  "Mixed ",      // The name of the filter F5
  "Min   ",      // The name of the filter F6
  "Soft  "       // The name of the filter F7
};

int Volume = 0;   //local volume -80 .. 10, initially 0dB
int source     = USB;              // Variable to hold the Input number. 1,2 for s/pdif, 0 for USB.
int sourceold  = 2;                // Variable to hold the old Input number. 1,2 for s/pdif, 0 for USB.
int signalType = PCM;              // Variable to hold the signal type. 0 for PCM, 1 for DSD.
int signalTypeold = 3;             // Variable to hold the old signal type. 0 for PCM, 1 for DSD.
int SRint;                          // Variable to hold the detected sampling rate as int.
float SR    = 0;                    // Variable to hold the detected sampling rate as float.
float SRold = 1.1;                  // Variable to hold the old detected sampling rate as float.
bool powerState = true;             // Variable to hold current power status (true: ON, false: SLEEP)

String filter;                      //current filter name
int currentFilter = 1;                   // Variable to hold the number of the selected filter.
int currentFilter_old;                  // Variable for detecting filter change.

String serialin;                    // Variable to hold the data read from the serial port.
int RECV_PIN = 10;                  // IR Receiver input pin.
long prevCmd=0;

//rotary encoder handling
volatile int encoderMoved = 0;
volatile int rotaryOffset = 0;

// rotary switch mode control
int rotaryMode = VolumeControl;     //initial operation Mode
int currentStateCLK;
volatile int lastStateCLK;
bool isPressing = false;            //still  pressing push button?
bool longPressTriggered = false;    //Entered filter control mode
unsigned long lastButtonPress=0;
unsigned long pressStartTime = 0;   //

// power switch handling
bool lastPwrSwState = HIGH;
unsigned long lastPwrSwDebounce = 0;

LiquidCrystal_I2C lcd(0x27, 16,2);    //16*2 LCD pannel at address 0x27

// Function Declarations
void togglePower();

// ---------------------------- Process DAC return codes
String Release; 

void rcodeRelCode()
{
  Release = serialin.substring(1);
}

void rcodeSRate()
{
  String s;

  s = serialin.substring(1);

  SRint = s.toInt();
  signalType = PCM;   //Default is PCM
  SR = 0;
  
  for (int i=0; i<NUM_SR; i++){
    if (SRTable[i].rxSR == SRint) {
      SR = SRTable[i].orgSR;
      signalType = SRTable[i].signalType;
      break;
    }
  }

  display_SR = true;
  display_type = true;
}

void rcodeSource()
{
  String s;
  
  sourceold = source;
  s = serialin.substring(1);
  source = s.toInt();

  if(source != sourceold) {
    display_Source = true;
    sourceold = source;
  }
}

void rcodeVolume()
{
  String s;

  s = serialin.substring(1);
  Volume = s.toInt();       //volume range is -80 to 10
  
  display_Vol = true; 
}

void rcodeFilter()
{
  String s;
  
  currentFilter_old = currentFilter;
  s = serialin.substring(1);
  currentFilter = s.toInt() - 3;
  filter = filters[currentFilter];

  if(currentFilter != currentFilter_old) {
    display_Filt = true;
    currentFilter_old = currentFilter;
  }
}

void rcodePower()
{
  String powerStateStr;
  
  powerStateStr = serialin.substring(1);
}

// ---------------------------- Read DAC response --------------------------------------------------
void read_input() 
{
  String s;
  
  signalType = PCM;
  if(Serial.available()) {      
    serialin = Serial.readStringUntil('\n');

    for(int i=0; i<6; i++) {
      if (serialin.charAt(0) == rcode[i].code[0]) {
        rcode[i].func();
        break;
      }
    }
    
    serialin = "";
  }
}

//
// ---------------------------- Power Switch / Remote Control Toggle logic 
void togglePower()
{
  powerState = !powerState;
  
  if (powerState) { // 동작 상태로 전환 (ON)
    digitalWrite(POWERPIN, HIGH);
    lcd.backlight();
    display_SR = true;
    display_type = true;
    display_Vol = true;
    display_Filt = true;
    display_Source = true;
  } else {          // 수면 상태로 전환 (SLEEP)
    digitalWrite(POWERPIN, LOW);
    lcd.noBlink();
    lcd.clear();
    lcd.noBacklight();
  }
}

void powerCode()
{
  togglePower();
}

void volUpCode()
{
  if (!powerState) return;

  int step;

  if(Volume <= -50) {
    step = 5;
  } else if(Volume <= -20) {
    step = 2;
  } else
    step = 1;

  Volume += step;
  
  if (Volume > 10)    // volume range -80 .. 10
    Volume = 10;
    
  doVolume();

  display_Vol = true;
}

void volDnCode()
{
  if (!powerState) return;

  int step;

  if(Volume <= -50) {
    step = 5;
  } else if(Volume <= -20) {
    step = 2;
  } else
    step = 1;

  Volume -= step;
  if (Volume < -80)     //volume rane 1..20
    Volume = -80;
    
  doVolume();
  display_Vol = true;
}

void sourceCode()
{
  if (!powerState) return;

  source++;

  if (source >= NUMSRC) 
    source = 0; 

  selectSource();
}

void filtUpCode()
{
  if (!powerState) return;

  currentFilter = (currentFilter == 4) ? 1 : currentFilter+1; //last filter?

  doFilter();
}

void filtDnCode()
{
  if (!powerState) return;

  currentFilter = (currentFilter == 1) ? 4 : currentFilter-1; //first filter?

  doFilter();
}

void doFilter()
{
  Serial.print("F");
  Serial.println(currentFilter+3);
  filter = filters[currentFilter];
  display_Filt = true;
}

//
//  control DAC board
//
// ------------------------ volume control command to DAC

void doVolume()
{
  Serial.print("V"); // Volume control start

  if (Volume > 0)
    Serial.print("+");    // send '+' when volume is positive
  else if (Volume == 0)
    Serial.println("+00"); // send "+00" when volume is normal position 

  if (Volume != 0)    Serial.println(Volume); //send volume digits
}

// ------------------------select source device - relay version
//

void selectSource()
{
  char coax, opt, y;     // 0 = usb, 1 = coax, 2 = opt

  y = ~source;
  
  coax = y & 0x01;  //LSB
  opt  = y & 0x02;  //LSB+1

  digitalWrite(COAXSEL, coax);    //relay version
  digitalWrite(OPTSEL, opt);

  display_Source = true;
}

//
//  display informations on LCD display 
//

// ------------------------ Display Sampling Rate ----------------------------------------------------
//
void disp_SR(void) {
  if (!powerState) return;
  
  lcd.setCursor(6, 0);
  lcd.print("       ");  //clear position
    
  if(SR == 0) {       //frequency unlocked - floating
    lcd.setCursor(6, 0);
    lcd.print("UnLock ");
  } else {
    if ((SR-(int)SR) > 0){         // SR is multiple of 44.1K or DSD
      lcd.setCursor(6, 0);
      lcd.print(SR,1);
    } else if ((SR-(int)SR) == 0) {  // SR is multiple of 48K
      SRint = (int)SR;
  
      lcd.setCursor(6, 0);
      lcd.print(SRint);
    }

    if(signalType == PCM)         //PCM
      lcd.print("K");
    else if(signalType ==DSD)     //DSD
      lcd.print("M");
  }
  display_SR = false;
}

// ------------------------ Display Source -------------------------------------------------------------
//
void disp_source(void)
{
  if (!powerState) return;
  lcd.setCursor(0, 0);
  lcd.print(sources[source]);
}

// ------------------------ Display Signal Type ---------------------------------------------------------
//
void disp_type(void)
{
  if (!powerState) return;
  if (signalType == PCM){
    lcd.setCursor(13, 0);
    lcd.print("PCM");
  } else {
    lcd.setCursor(13, 0);
    lcd.print("DSD");
  }
}

// ------------------------ Display Filter -------------------------------------------------------------
//
void disp_filter(void)
{
  if (!powerState) return;
  lcd.setCursor(0, 1);
  lcd.print(filter);
  if(rotaryMode == FilterControl) {
    lcd.setCursor(0,1);
    lcd.blink();
  } else 
    lcd.noBlink();
}

// ------------------------ Display Volume ------------------------------------------------------------
//
void disp_volume(void)
{
  if (!powerState) return;
  lcd.setCursor(7, 1); 
  lcd.print("Vol:");

  if(Volume > -10 && Volume < 10) {
    lcd.print(" ");
    if(Volume == 0)
      lcd.print(" ");
    if(Volume > 0)
      lcd.print("+"); 
  }
  lcd.print(Volume);
  lcd.print("dB ");
}

//*****************************************************************************************************************
// interrupt service code
void rotaryISR()
{
  int clkState = digitalRead(CLK);
  int dtState = digitalRead(DT);
  
  if (clkState != lastStateCLK) {
    if (dtState != clkState) {
      rotaryOffset++;
    } else {
      rotaryOffset--;
    }
    encoderMoved = true;
    lastStateCLK = clkState;
  }
}


// Arduino board setup
//
void setup(void) 
{
  pinMode(POWERPIN, OUTPUT);
  digitalWrite(POWERPIN, HIGH); // 기본 상태: ON (동작)
  
  pinMode(POWER_SW, INPUT_PULLUP); // 추가된 택트스위치 핀
  
  pinMode(COAXSEL, OUTPUT);
  pinMode(OPTSEL, OUTPUT);
  digitalWrite(COAXSEL, LOW);
  digitalWrite(OPTSEL, LOW);
  
  pinMode(CLK,INPUT_PULLUP);
  pinMode(DT,INPUT_PULLUP);
  pinMode(SW, INPUT_PULLUP);
  pinMode(RECV_PIN, INPUT);
  
  attachInterrupt(digitalPinToInterrupt(CLK), rotaryISR, FALLING);

  Serial.begin(115200);     //start Serial port 
  IrReceiver.begin(RECV_PIN, DISABLE_LED_FEEDBACK);      // Start the IR receiver
  
  lcd.init();       //start LCD
  lcd.backlight();

  lastStateCLK = digitalRead(CLK);
                                      //init DAC board
  selectSource();   //default is auto
  Volume = 0;      //set volume to normal 
  doVolume();
}

//
// main loop
//
void loop()
{
  unsigned long currentMillis = millis();

  // ---------------- Check Power Switch (Tact Switch) ----------------
  int currentPwrSwState = digitalRead(POWER_SW);
  if (currentPwrSwState != lastPwrSwState) {
    if ((currentMillis - lastPwrSwDebounce) > 50) { // 50ms 디바운스
      if (currentPwrSwState == LOW) { // 스위치가 눌렸을 때 (GND)
        togglePower();
      }
      lastPwrSwDebounce = currentMillis;
      lastPwrSwState = currentPwrSwState;
    }
  }

  // ---------------- Check IR Remote ----------------
  if(IrReceiver.decode()) {   //library updated
    long  irCmd;
    irCmd = IrReceiver.decodedIRData.decodedRawData & 0xFFFF;
    if(prevCmd != irCmd) {
      for (int i=0; i<6; i++) {
        if (irCmd == ircmd[i].irCode) {
          ircmd[i].func();
          break;
        }
      }
    }
    prevCmd = irCmd;
    IrReceiver.resume();                       // Resume decoding (necessary!)
  }

  // 수면 상태(Power OFF) 시 나머지 입력 및 디스플레이 제어 스킵
  if (!powerState) {
    read_input(); // 수신 큐 비우기 용도
    return;
  }

  // ---------------- Rotary Encoder Handling ----------------
  if(encoderMoved) {
    noInterrupts();
    int moveStep = rotaryOffset;
    rotaryOffset = 0;
    encoderMoved = false;
    interrupts();

    if(rotaryMode == VolumeControl) {
      int step  = 1;

      if(Volume <= -50)           //log scale volume offset
        step = 5;
      else if(Volume <= -20)
        step = 2;

      Volume = constrain(Volume + (step * moveStep), -80, 10);

      doVolume();
      display_Vol = true; 
    } else {   //if(rotaryMode == FilterControl)
      currentFilter = currentFilter + moveStep;   //rotate filter
      if (currentFilter < 1) currentFilter = 4;
      if (currentFilter > 4) currentFilter = 1;
      doFilter();
    } 
  }

  // ---------------- Check Rotary Button (Source Selection) ----------------
  int btnState = digitalRead(SW);

  if (btnState == LOW && !isPressing) {      // button switch (rotary) pressed
    if ((currentMillis - lastButtonPress) > 100) {  // more than 100ms
      pressStartTime = millis(); //store current time
      isPressing = true;
      longPressTriggered = false;
    }
  }

  if(isPressing) {    //button in press
    unsigned long Duration = millis() - pressStartTime;
    if(Duration >= 1000 && !longPressTriggered) {
      longPressTriggered = true;
    }
  }

  if(btnState == HIGH && isPressing) {    //button released
    unsigned long finalDuration = millis() - pressStartTime;

    if(finalDuration < 1000 ) {
      if (rotaryMode == VolumeControl) {
        sourceold = source;
        source++;
        if (source >= NUMSRC)
          source = 0;
        selectSource(); 
      } else {
        rotaryMode = VolumeControl; //exit rotary mode change
        display_Filt = true;        //draw again
      }
    } else {   //finalDuration > 1000 -- long press
        rotaryMode = FilterControl; //rotary mode change to filter control
        longPressTriggered = false;
        display_Filt = true;        //cursor blink
    }

    isPressing = false;
    lastButtonPress = millis();
  }

  read_input();                              // get DAC response code

  for(int i=0; i<NUMDISP; i++) {  //check display items has changed
    if( *dispControl[i].flag == true ) {   //flag has set to true
      dispControl[i].func();               //call corresponding diplay function
      *dispControl[i].flag = false;        //done
    }
  }
}