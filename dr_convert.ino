#include <MIDI.h>
#include <advancedSerial.h>
#define USBserial Serial
#define VERBOSITY Level::vvvv

// mode 0 -> native MIDI bitrate
// mode 1 -> serial bitrate, MIDI over hairless bridge
// mode 2 -> serial bitrate, debugging via serial monitor
const uint8_t mode = 2;
const uint8_t midi_ch = 10;
const uint8_t midi_ch_drbeat = 11;
const uint8_t midi_ch_volca = 10;
const uint8_t ledPin = 13;      // LED pin on most Arduinos
const uint8_t ledPins[5] = {9,10,11,12,13};      // LED pins for "we are ready" flashing
// convert mode 0 -> DRBEAT
// convert mode 1 -> DRBEAT ALL ROLLS
// convert mode 2 -> VOLCA
uint8_t convert_mode = 0;
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

MIDI_CREATE_INSTANCE(HardwareSerial, USBserial, MIDI); // with this method port is selectable here

void setup()
{
    pinMode(ledPin, OUTPUT);
    for (uint8_t ledset = 1; ledset <= sizeof(ledPins); ledset++)
    {
        pinMode(ledPins[ledset], OUTPUT);
    }
    while(!USBserial); // wait until USBserial is accessible
    if (mode == 0)
    {
        USBserial.begin(31250); // MIDI serial rate -> cheap USB MIDI cable hack
        aSerial.off();  // disable debug output
    }
    else if (mode == 1)
    {
        USBserial.begin(9600); // common serial rate -> hairless midi bridge
        aSerial.off();  // disable debug output
    }
    else if (mode == 2)
    {
        USBserial.begin(9600);  // common serial rate -> debugging
        aSerial.setPrinter(USBserial);  // debugging settings
        aSerial.setFilter(VERBOSITY); // debugging settings
        aSerial.v().pln("Dr. Convert is ready...");
        aSerial.on();  // enable debug output
    }
}

void sendCCandLog(uint8_t cc_num, uint8_t cc_value, uint8_t _midi_ch, uint8_t _mode)
{
    digitalWrite(ledPin, HIGH);
    //for (uint8_t led = 1; led <= sizeof(ledPins); led++)
    //{
    //    digitalWrite(ledPins[led], HIGH);
    //}
    aSerial.vvv().p("sending CC: ").pln(cc_num);
    aSerial.vvv().pln();
    if (_mode != 2)
    {
        delay(1);
        MIDI.sendControlChange(cc_num, cc_value, _midi_ch);
        delay(5);
    }
}

void sendNoteONandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch, uint8_t _mode)
{
    digitalWrite(ledPin, HIGH);
    //for (uint8_t led = 1; led <= sizeof(ledPins); led++)
    //{
    //    digitalWrite(ledPins[led], HIGH);
    //}
    aSerial.vvv().p("sending NoteOn: ").pln(note_num);
    aSerial.vvv().pln();
    if (_mode != 2)
    {
        delay(1);
        MIDI.sendNoteOn(note_num, note_vel, _midi_ch);
        delay(5);
    }
}

void sendNoteOFFandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch, uint8_t _mode)
{
    digitalWrite(ledPin, HIGH);
    //for (uint8_t led = 1; led <= sizeof(ledPins); led++)
    //{
    //    digitalWrite(ledPins[led], LOW);
    //}
    aSerial.vvv().p("sending NoteOFF: ").pln(note_num);
    aSerial.vvv().pln();
    if (_mode != 2)
    {
        delay(1);
        MIDI.sendNoteOff(note_num, note_vel, _midi_ch);
        delay(5);
    }
}

void handleNoteOn(byte Channel, byte PitchMidi, byte Velocity) {
    digitalWrite(ledPin, HIGH); // DEBUG LED on
    aSerial.vvv().p("handleNoteOn: Channel, PitchMidi: "); // DEBUG
    aSerial.vvv().p(Channel).p(" ").p(PitchMidi).p(" "); // DEBUG

    for (uint8_t note_pos = 0; note_pos <= sizeof(note_mapping); note_pos++)
    {
        if (PitchMidi == note_mapping[note_pos][0]) // check if rcvd note (PitchMidi) 
        {                                           // is one of the 16 notes of our drumpad 
            aSerial.vvv().p("rcvd NoteON: ").p(note_mapping[note_pos][0]).p(
                               " velo: ").p(Velocity).p(" ");
            //aSerial.vvv().p("conv NoteON: ").p(note_mapping[note_pos][1]).p(
            //                   " velo: ").p(Velocity).p(" ");
            //aSerial.vvv().p("roll NoteON: ").p(note_mapping[note_pos][2]).p(
            //                 " velo: ").p(Velocity).p(" ");
            //MIDI.sendNoteOn(note_mapping[note_pos][1], Velocity, midi_ch);
            sendNoteONandLog(note_mapping[note_pos][1], Velocity, midi_ch, mode);
        }
        else
        {
            aSerial.vvv().p("blocking unknown NoteON: ").p(note_mapping[note_pos][0]);
        }
    }
}

void handleNoteOff(byte Channel, byte PitchMidi, byte Velocity) { // NoteOn with 0 velo is NoteOff. 
    digitalWrite(ledPin, LOW); // DEBUG LED off
    aSerial.vvv().p("handleNoteOFF: Channel, PitchMidi: "); // DEBUG
    aSerial.vvv().p(Channel).p(" ").p(PitchMidi).p(" "); // DEBUG

    for (uint8_t note_pos = 0; note_pos <= sizeof(note_mapping); note_pos++)
    {
        if (PitchMidi == note_mapping[note_pos][0]) // check if rcvd note (PitchMidi) 
        {                                           // is one of the 16 notes of our drumpad 
            aSerial.vvv().p("rcvd NoteOFF: ").p(note_mapping[note_pos][0]).p(
                               " velo: ").p(Velocity).p(" ");
            //aSerial.vvv().p("conv NoteOFF: ").p(note_mapping[note_pos][1]).p(
            //                   " velo: ").p(Velocity).p(" ");
            //aSerial.vvv().p("roll NoteOFF: ").p(note_mapping[note_pos][2]).p(
            //                 " velo: ").p(Velocity).p(" ");
            //MIDI.sendNoteOn(note_mapping[note_pos][1], Velocity, midi_ch);
            sendNoteOFFandLog(note_mapping[note_pos][1], Velocity, midi_ch, mode);
        }
        else
        {
            aSerial.vvv().p("blocking unknown NoteOFF: ").p(note_mapping[note_pos][0]);
        }
    }
}

void handleControlChange(byte inChannel, byte inNumber, byte inValue) {
}

void handleStart() {
}

void handleStop() {
}

void handleContinue() {
}

void handleClock() {
}

void loop()
{

}
