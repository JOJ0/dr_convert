#include <MIDI.h>
#include <advancedSerial.h>
#include <AltSoftSerial.h>
#include <RotaryEncoder.h>

#define ELEMENTCOUNT(x)  (sizeof(x) / sizeof(x[0]))
#define PRESSED LOW
#define RELEASED HIGH
//give pins a name
#define MODE_SWITCH_1_PIN 5
#define MODE_SWITCH_2_PIN 6
#define MODE_SWITCH_3_PIN 7
#define MODE_SWITCH_1_STATE digitalRead(MODE_SWITCH_1_PIN)
#define MODE_SWITCH_2_STATE digitalRead(MODE_SWITCH_2_PIN)
#define MODE_SWITCH_3_STATE digitalRead(MODE_SWITCH_3_PIN)
uint8_t mode_pins[3] = {MODE_SWITCH_1_PIN, MODE_SWITCH_2_PIN, MODE_SWITCH_3_PIN};
#define ROTARY_SWITCH_1_PIN 12
#define ROTARYMIN 0
#define ROTARYMAX 5

//*** GLOBAL COMPILE AND RUNTIME SETTINGS START ***
#define USBserial Serial
#define VERBOSITY Level::vvv
#define SOFT_SERIAL_DEBUG // comment out this line for mode 0
// mode 0 -> native MIDI, NO debugging! (Serial can't be used twice)
// mode 1 -> SoftwareSerial, debugging via serial monitor

#ifdef SOFT_SERIAL_DEBUG
    const uint8_t mode = 1; // mode 1 needs SoftwareSerial, comment out above!
    AltSoftSerial SoftSerial;
    MIDI_CREATE_INSTANCE(AltSoftSerial, SoftSerial, MIDI);
#else
    const uint8_t mode = 0;
    MIDI_CREATE_INSTANCE(HardwareSerial, USBserial, MIDI);
#endif
// *** GLOBAL COMPILE AND RUNTIME SETTINGS END ***

const uint8_t midi_ch = 10;
const uint8_t midi_ch_drbeat = 11;
const uint8_t midi_ch_volca = 10;
const uint8_t ledPin = 13;      // LED pin on most Arduinos
long prevMillisLedBuiltin = 0;
//const uint8_t ledPins[5] = {9,10,11,12,13};      // LED pins for "we are ready" flashing
enum conv_modes { BYPASS, DR202, DR202_ROLLS, DR202_ROLLS_HATS, DR202_ROLLS_PERC, VOLCA };
const char* convModeNames[] = {"BYPASS", "DR202", "DR202_ROLLS", "DR202_ROLLS_HATS", "DR202_ROLLS_PERC",
                                "VOLCA" };
conv_modes conv_mode = BYPASS; // initially we want bypass mode (if switch broken or something)
conv_modes last_conv_mode = BYPASS;
//printf("%s", convModeNames[conv_mode]);

uint8_t note_mapping[16][8] = {
    // first 8 pads   DR202         DR_ROLLS  DR_ROLLS_2   DR_ROLLS_3    VOLCA   
    {36, 36,100,36}, // KICK 1       (ALL)     (HATS)       (PERC)        KICK    
    {37, 35,101,75}, // KICK 2                                            CLAVES  
    {38, 38,102,38}, // SNARE 1                                           SNARE   
    {39,102,102,39}, // ROLL SN 1              ROLL SN 1    ROLL PERC 1   CLAP    
    {40, 40,103,67}, // SNARE 2                                           AGOGO   
    {41,103,103,41}, // ROLL SN 2              ROLL SN 2    ROLL PERC 2           
    {42, 42,104,42}, // CLSD HH                                           CLSD HAT 
    {43, 43,110,43}, // HIT 3                                             LO TOM  
    // second 8 pads                                                              
    {44, 60,111,44}, // PERC 1                 ROLL HIT 3                         
    {45, 61,112,45}, // PERC 2                 ROLL OP HAT                        
    {46, 46,105,46}, // OPEN HAT                                          OP HAT
    {47, 47,108,47}, // HIT 2                                                     
    {48,104,104,48}, // ROLL CLSD HH           ROLL CLSD HH ROLL HIT 2            
    {49, 49,109,49}, // CRASH                                                     
    {50, 50,106,50}, // HIT 1                  ROLL RIDE                  HI TOM  
    {51, 51,107,51}, // RIDE                                ROLL HIT 3            
};

static int buttonState = 0;
static int lastButtonState[4] = {0,0,0,0};

RotaryEncoder encoder[4] = {
    RotaryEncoder(A0, A1),
    RotaryEncoder(A2, A3),
    RotaryEncoder(A4, A5),
    RotaryEncoder(A6, A7)
};
//RotaryEncoder encoder1(A2, A3); // Setup a RoraryEncoder for pins A2 and A3:
static int currentEncoderPos[4] = {0,0,0,0};
static int lastEncoderPos[4] = {0,0,0,0};

void setup()
{
    pinMode(ledPin, OUTPUT);
    pinMode(ROTARY_SWITCH_1_PIN, INPUT_PULLUP);
    //for (uint8_t i = 0; i < 3; i++) {pinMode(mode_pins[i], INPUT);}
    // test MODE_SWITCH_1
    pinMode(MODE_SWITCH_1_PIN, INPUT_PULLUP);
    // You may have to modify the next 2 lines if using other pins than A2 and A3
    PCICR |= (1 << PCIE1); // This enables Pin Change Interrupt 1 that covers the Analog input
                           // pins or Port C.
    PCMSK1 |= (1 << PCINT8) | (1 << PCINT9);   // enables interrupt for pin 0 and 1 of Port C.
    PCMSK1 |= (1 << PCINT10) | (1 << PCINT11); // enables interrupt for pin 2 and 3 of Port C.
    // enabling Pin Change Interrupts might be easier to understand in binary:
    //PCICR |= 0b00000001;    // turn on port b
    //PCICR |= 0b00000010;    // turn on port c
    //PCICR |= 0b00000100;    // turn on port d
    //PCICR |= 0b00000111;    // turn on all ports

    MIDI.begin(midi_ch);  // Listen to incoming messages on given channel
    //MIDI.begin(MIDI_CHANNEL_OMNI);  // Listen to incoming messages on all channels
    //MIDI.turnThruOff();  // Listen to incoming messages on given channel
    MIDI.setThruFilterMode(midi::Thru::Full); // Full: all msg from all channels sent thru
    while(!USBserial); // wait until USBserial is accessible
    if (mode == 0)
        aSerial.off();  // disable debug output
    else
    {
        USBserial.begin(9600);  // common serial rate -> debugging
        aSerial.setPrinter(USBserial);  // debugging settings
        aSerial.setFilter(VERBOSITY); // debugging settings
        aSerial.v().pln("Dr. Convert is ready...");
        aSerial.on();  // enable debug output
    }
    digitalWrite(ledPin, LOW); // DEBUG LED off

}

// The Interrupt Service Routine for Pin Change Interrupt 1
// This routine will only be called on any signal change on A2 and A3: exactly where we need to check.
ISR(PCINT1_vect) {
  encoder[0].tick(); // just call tick() to check the state.
  encoder[1].tick();
}

void sendNoteONandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch)
{
    aSerial.vvv().p("sendNoteONandLog: ").pln(note_num);
    aSerial.vvv().pln();
    MIDI.sendNoteOn(note_num, note_vel, _midi_ch);
}

void sendNoteOFFandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch)
{
    aSerial.vvv().p("sendNoteOFFandLog: ").pln(note_num);
    aSerial.vvv().pln();
    MIDI.sendNoteOff(note_num, note_vel, _midi_ch);
}

void handleNoteOn(byte Channel, byte PitchMidi, byte Velocity) {
    digitalWrite(ledPin, HIGH); // DEBUG LED on
    aSerial.vvv().p("handleNoteOn: Channel, PitchMidi, Velocity: "); // DEBUG
    aSerial.vvv().p(Channel).p(" ").pln(PitchMidi); // DEBUG

    for (uint8_t note_pos = 0; note_pos < ELEMENTCOUNT(note_mapping); note_pos++)
    {
        if (PitchMidi == note_mapping[note_pos][0]) // check if rcvd note (PitchMidi) 
        {                                           // is one of the 16 notes of our drumpad 
            if (conv_mode == BYPASS) {
                aSerial.vvv().p("BYPASS MODE - all thru: ").pln(note_mapping[note_pos][0]).pln();}
            else if (conv_mode == DR202) {
                aSerial.vvv().p("DR202 NoteON: ").pln(note_mapping[note_pos][1]);
                sendNoteONandLog(note_mapping[note_pos][1], Velocity, Channel);}
            else if (conv_mode == DR202_ROLLS) {
                aSerial.vvv().p("DR202_ROLLS NoteON: ").pln(note_mapping[note_pos][2]);
                sendNoteONandLog(note_mapping[note_pos][2], Velocity, Channel);}
            else if (conv_mode == VOLCA) {
                aSerial.vvv().p("VOLCA NoteON: ").pln(note_mapping[note_pos][3]);
                sendNoteONandLog(note_mapping[note_pos][3], Velocity, Channel);}
        }
    }
}

void handleNoteOff(byte Channel, byte PitchMidi, byte Velocity) { // NoteOn with 0 velo is NoteOff. 
    digitalWrite(ledPin, LOW); // DEBUG LED off
    aSerial.vvv().p("handleNoteOFF: Channel, PitchMidi: "); // DEBUG
    aSerial.vvv().p(Channel).p(" ").pln(PitchMidi); // DEBUG

    for (uint8_t note_pos = 0; note_pos < ELEMENTCOUNT(note_mapping); note_pos++)
    {
        if (PitchMidi == note_mapping[note_pos][0]) // check if rcvd note (PitchMidi) 
        {                                           // is one of the 16 notes of our drumpad 
            if (conv_mode == BYPASS) {
                aSerial.vvv().p("BYPASS MODE - all thru: ").pln(note_mapping[note_pos][0]).pln();}
            else if (conv_mode == DR202) {
                aSerial.vvv().p("DR202 NoteOFF: ").pln(note_mapping[note_pos][1]);
                sendNoteOFFandLog(note_mapping[note_pos][1], Velocity, Channel);}
            else if (conv_mode == DR202_ROLLS) {
                aSerial.vvv().p("DR202_ROLLS NoteOFF: ").pln(note_mapping[note_pos][2]);
                sendNoteOFFandLog(note_mapping[note_pos][2], Velocity, Channel);}
            else if (conv_mode == VOLCA) {
                aSerial.vvv().p("VOLCA NoteOFF: ").pln(note_mapping[note_pos][3]);
                sendNoteOFFandLog(note_mapping[note_pos][3], Velocity, Channel);}
        }
    }
}

void sendCCandLog(uint8_t cc_num, uint8_t cc_value, uint8_t _midi_ch)
{
    aSerial.vvv().p("sending CC: ").pln(cc_num);
    aSerial.vvv().pln();
    MIDI.sendControlChange(cc_num, cc_value, _midi_ch);
}

void handleControlChange(byte inChannel, byte inNumber, byte inValue) {
    if (conv_mode == BYPASS) { // dont send anything in bypass mode - we dont want doubles!
        aSerial.vvv().p("BYPASS MODE - all thru").pln();}
    else{
        sendCCandLog(inNumber, inValue, inChannel);} // just log and forward CCs without manipulation
}

void handleProgramChange(byte inChannel, byte inNumber) {
    if (conv_mode == BYPASS) { // dont send anything in bypass mode - we dont want doubles!
        aSerial.vvv().p("BYPASS MODE - all thru").pln();}
    else{
        MIDI.sendProgramChange(inNumber, inChannel);} // just forward PCs without manipulation
}

void handleStart() {
}

void handleStop() {
}

void handleContinue() {
}

void handleClock() {
}

void setMessageHandles()
{
    MIDI.setHandleNoteOn(handleNoteOn);
    MIDI.setHandleNoteOff(handleNoteOff);
    MIDI.setHandleControlChange(handleControlChange);
    MIDI.setHandleProgramChange(handleProgramChange);
    MIDI.setHandleStart(handleStart);
    MIDI.setHandleStop(handleStop);
    MIDI.setHandleContinue(handleContinue);
    MIDI.setHandleClock(handleClock);
}

uint8_t getButtonState(uint8_t buttonPin) {
    uint8_t changed = 0;
    buttonState = digitalRead(buttonPin);
    if (buttonState != lastButtonState[buttonPin]) {
        aSerial.vvv().p("Button on pin ").p(buttonPin).p(" changed to ").pln(buttonState);
        changed = 1;
    }
    lastButtonState[buttonPin] = buttonState;
    if (changed == 1) {
	return buttonState;
    }
    else {
        return lastButtonState[buttonPin];
    }
}

uint8_t getEncoderPos(uint8_t encNum, uint8_t rotaryMin, uint8_t rotaryMax) {
    currentEncoderPos[encNum] = encoder[encNum].getPosition();

    if (currentEncoderPos[encNum] < rotaryMin) {
        currentEncoderPos[encNum] = rotaryMin;
        encoder[encNum].setPosition(rotaryMin);
    }
    else if (currentEncoderPos[encNum] > rotaryMax) {
        currentEncoderPos[encNum] = rotaryMax;
        encoder[encNum].setPosition(rotaryMax);
    }

    if (lastEncoderPos[encNum] != currentEncoderPos[encNum]) {
        aSerial.vvv().p("Rotary encoder ").p(encNum).p(" changed to ").pln(currentEncoderPos[encNum]);
        lastEncoderPos[encNum] = currentEncoderPos[encNum];
    }
    return currentEncoderPos[encNum];
}

long blinkLed(uint8_t ledPin, uint16_t interval, long previousMillis) {
    unsigned long currentMillis = millis();

    if (currentMillis - previousMillis >= interval) {
        //aSerial.vvv().pln("interval reached ");
        // save the last time you blinked the LED
        previousMillis = currentMillis;
        bool ledState = digitalRead(ledPin);
        // if the LED is off turn it on and vice-versa:
        if (ledState == LOW) {
            ledState = HIGH;
        } else {
            ledState = LOW;
        }
        digitalWrite(ledPin, ledState);
    }
    return previousMillis;
}

void loop()
{
    if (getButtonState(ROTARY_SWITCH_1_PIN) == PRESSED) {
    //if (getButtonState(MODE_SWITCH_3_PIN) == PRESSED) {
        // aSerial.vvv().pln("Rotary 1 Button is pressed");
    }
    else {
        // aSerial.vvv().pln("Rotary 1 Button is released");
    }

    //uint8_t mode_bitmask = B000;
    //for (uint8_t i = 0; i < 3; i++) {
    //    mode_bitmask = mode_bitmask << 1;
    //    mode_bitmask |= digitalRead(mode_pins[i]);
    //}
    //switch (mode_bitmask) {
    //    case B000:
    //        conv_mode = BYPASS;
    //        break;
    //    case B001:
    //        conv_mode = VOLCA;
    //        break;
    //    case B010:
    //        conv_mode = DR202;
    //        break;
    //    case B011:
    //        conv_mode = DR202_ROLLS;
    //        break;
    //    case B111:
    //        conv_mode = DR202_ROLLS_HATS;
    //        break;
    //    case B101:
    //        conv_mode = DR202_ROLLS_PERC;
    //        break;
    //}

    uint8_t modeEncoderPos = getEncoderPos(0, ROTARYMIN, ROTARYMAX);
    switch (modeEncoderPos) {
        case 0:
            conv_mode = BYPASS;
            digitalWrite(ledPin, LOW);
            break;
        case 1:
            conv_mode = VOLCA;
            digitalWrite(ledPin, LOW);
            break;
        case 2:
            conv_mode = DR202;
            prevMillisLedBuiltin = blinkLed(ledPin, 1000, prevMillisLedBuiltin);
            break;
        case 3:
            conv_mode = DR202_ROLLS;
            digitalWrite(ledPin, LOW);
            break;
        case 4:
            conv_mode = DR202_ROLLS_HATS;
            digitalWrite(ledPin, LOW);
            break;
        case 5:
            conv_mode = DR202_ROLLS_PERC;
            digitalWrite(ledPin, LOW);
            break;
    }
    if (conv_mode != last_conv_mode) {
        aSerial.vvv().p("Conversion mode set to ").pln(convModeNames[conv_mode]);
    }
    last_conv_mode = conv_mode;
    // blink if favorite mode is set
    if (conv_mode == DR202) {
    }

   // done with switch reading, main program
    if (conv_mode == BYPASS)
    {
        MIDI.setThruFilterMode(midi::Thru::Full); // all msg from all channels sent thru
        setMessageHandles(); // just for now, later this should not be here -> THRU without hassle
    }
    else // MODES: DR202, DR202_ROLLS*, VOLCA
    {
        MIDI.setThruFilterMode(midi::Thru::DifferentChannel); // all msg except from in-channel go thru
        setMessageHandles(); // NoteOn NoteOff CC etc. handles are defined, we wanna manipulate
    }
    MIDI.read();

}
