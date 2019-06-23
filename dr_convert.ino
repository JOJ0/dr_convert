#include <MIDI.h>
#include <advancedSerial.h>
#include <SoftwareSerial.h>
#define ELEMENTCOUNT(x)  (sizeof(x) / sizeof(x[0]))

//*** GLOBAL COMPILE AND RUNTIME SETTINGS START ***
#define USBserial Serial
#define VERBOSITY Level::vvvv
SoftwareSerial MIDIserial(4, 2); // RX, TX
// mode 0 -> native MIDI, NO debugging! (Serial can't be used twice)
// mode 1 -> SoftwareSerial, debugging via serial monitor
// mode 2 -> No Serial, just debugging via serial monitor
const uint8_t mode = 0; // mode 1 needs SoftwareSerial, comment out below!
//MIDI_CREATE_INSTANCE(SoftwareSerial, MIDIserial, MIDI);
MIDI_CREATE_INSTANCE(HardwareSerial, USBserial, MIDI);
// *** GLOBAL COMPILE AND RUNTIME SETTINGS END ***

const uint8_t midi_ch = 10;
const uint8_t midi_ch_drbeat = 11;
const uint8_t midi_ch_volca = 10;
const uint8_t ledPin = 13;      // LED pin on most Arduinos
//const uint8_t ledPins[5] = {9,10,11,12,13};      // LED pins for "we are ready" flashing
enum conv_modes { BYPASS, DRBEAT, DRBEAT_ROLLS, VOLCA };
conv_modes conv_mode = BYPASS; // initially we want bypass mode (if switch broken or something)
const uint8_t conv_mode_sw1 = 5;
const uint8_t conv_mode_sw2 = 6;

bool conv_mode_switch[2][1] = { // 2 switches/bits saving the wanted convert mode
    {LOW},
    {LOW},
};

struct convert_settings
{
    char* conv_mode;
    int note_map;
};

//struct key_value
//{
//   int key;
//   char* value;
//};
//
//struct key_value kv;
//
//kv.key = 1;
//kv.value = "foo";

uint8_t note_mapping[16][4] = {
    // first 8 pads   DRBEAT             VOLCA
    {36, 36,100,36}, // KICK 1           KICK
    {37, 35,101,75}, // KICK 2           CLAVES
    {38, 38,102,38}, // SNARE 1          SNARE
    {39,102,102,39}, // ROLL SNARE 1     CLAP
    {40, 40,103,67}, // SNARE 2          AGOGO
    {41,103,103,41}, // ROLL SNARE 2     
    {42, 42,104,42}, // CLOSED HH        CLOSED HH
    {43, 43,110,43}, // HIT 3            LO TOM
    // second 8 pads
    {44, 60,111,44}, // PERC 1
    {45, 61,112,45}, // PERC 2
    {46, 46,105,46}, // OPEN HH          OPEN HH
    {47, 47,108,47}, // HIT 2
    {48,104,104,48}, // ROLL CLOSED HH
    {49, 49,109,49}, // CRASH
    {50, 50,106,50}, // HIT 1            HI TOM
    {51, 51,107,51}, // RIDE            
};


void setup()
{
    pinMode(ledPin, OUTPUT);
    pinMode(conv_mode_sw1, INPUT);
    pinMode(conv_mode_sw2, INPUT);
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

void sendNoteONandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch)
{
    aSerial.vvv().p("sendNoteONandLog: ").pln(note_num);
    aSerial.vvv().pln();
    if (mode != 2)
    {
      MIDI.sendNoteOn(note_num, note_vel, _midi_ch);
    }
}

void sendNoteOFFandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch)
{
    aSerial.vvv().p("sendNoteOFFandLog: ").pln(note_num);
    aSerial.vvv().pln();
    if (mode != 2)
    {
        MIDI.sendNoteOff(note_num, note_vel, _midi_ch);
    }
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
            else if (conv_mode == DRBEAT) {
                aSerial.vvv().p("DRBEAT NoteON: ").pln(note_mapping[note_pos][1]);
                sendNoteONandLog(note_mapping[note_pos][1], Velocity, Channel);}
            else if (conv_mode == DRBEAT_ROLLS) {
                aSerial.vvv().p("DRBEAT_ROLLS NoteON: ").pln(note_mapping[note_pos][2]);
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
            else if (conv_mode == DRBEAT) {
                aSerial.vvv().p("DRBEAT NoteOFF: ").pln(note_mapping[note_pos][1]);
                sendNoteOFFandLog(note_mapping[note_pos][1], Velocity, Channel);}
            else if (conv_mode == DRBEAT_ROLLS) {
                aSerial.vvv().p("DRBEAT_ROLLS NoteOFF: ").pln(note_mapping[note_pos][2]);
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
    if (mode != 2)
    {
        MIDI.sendControlChange(cc_num, cc_value, _midi_ch);
    }
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

void loop()
{
    //struct convert_settings settings
    //settings.conv_mode = "DRBEAT_ROLLS"
    //settings.note_map = 2
    // read in 2 bit conversion mode switches
    if (digitalRead(conv_mode_sw1) == LOW && digitalRead(conv_mode_sw2) == LOW)
    {
        conv_mode = BYPASS;
    }
    else if (digitalRead(conv_mode_sw1) == LOW && digitalRead(conv_mode_sw2) == HIGH)
    {
        conv_mode = VOLCA;
    }
    else if (digitalRead(conv_mode_sw1) == HIGH && digitalRead(conv_mode_sw2) == LOW)
    {
        conv_mode = DRBEAT;
    }
    else if (digitalRead(conv_mode_sw1) == HIGH && digitalRead(conv_mode_sw2) == HIGH)
    {
        conv_mode = DRBEAT_ROLLS;
    }

    // done with switch reading, main program
    if (conv_mode == BYPASS)
    {
        MIDI.setThruFilterMode(midi::Thru::Full); // all msg from all channels sent thru
        setMessageHandles(); // just for now, later this should not be here -> THRU without hassle
    }
    else // MODES: DRBEAT, DRBEAT_ROLLS, VOLCA
    {
        MIDI.setThruFilterMode(midi::Thru::DifferentChannel); // all msg except from in-channel go thru
        setMessageHandles(); // NoteOn NoteOff CC etc. handles are defined, we wanna manipulate
    }
    MIDI.read();
}
