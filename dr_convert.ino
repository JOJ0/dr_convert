#include <MIDI.h>
#include <advancedSerial.h>
#define USBserial Serial
#define VERBOSITY Level::vvvv

// mode 0 -> native MIDI bitrate
// mode 1 -> serial bitrate, MIDI over hairless bridge
// mode 2 -> serial bitrate, debugging via serial monitor
const uint8_t mode = 0;
const uint8_t midi_ch = 1;

const uint8_t ledPin = 13;      // LED pin on most Arduinos
const uint8_t ledPins[5] = {9,10,11,12,13};      // LED pins for "we are ready" flashing

uint8_t chrom_to_dr202[16][3] = {
    {36,36,100},
    {37,35,101},
    {38,38,102},
    {39,00,000},
    {40,40,103},
    {41,00,000},
    {42,42,104},
    {43,43,110},
    {44,60,111},
    {45,61,112},
    {46,46,105},
    {47,47,108},
    {48,00,000},
    {49,49,109},
    {50,50,106},
    {51,51,107},
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
    if (mode != 2)
    {
        delay(1);
        MIDI.sendControlChange(cc_num, cc_value, _midi_ch);
        delay(5);
    }
}

/*
void sendNoteONandLog(uint8_t note_num, uint8_t note_vel, uint8_t _midi_ch, uint8_t _mode)
{
    digitalWrite(ledPin, HIGH);
    //for (uint8_t led = 1; led <= sizeof(ledPins); led++)
    //{
    //    digitalWrite(ledPins[led], HIGH);
    //}
    aSerial.vvv().p("sending NoteOn: ").pln(note_num);
    aSerial.vvv().pln();
    if (mode != 2)
    {
        delay(1);
        MIDI.sendNoteOn(note_num, note_vel, _midi_ch);
        delay(5);
    }
}
*/

void handleNoteOn(byte Channel, byte PitchMidi, byte Velocity) {
    digitalWrite(ledPin, HIGH); // DEBUG LED on
    aSerial.vvvv().p("channel PitchMidi: "); // DEBUG
    aSerial.vvvv().p(Channel).p(" ").p(PitchMidi).p(" "); // DEBUG

    for (uint8_t note_pos = 0; note_pos <= sizeof(chrom_to_dr202); note_pos++)
    {
        if (PitchMidi == chrom_to_dr202[note_pos][0]) // check if rcvd note (PitchMidi) 
        {                                           // is one of the 16 chrom notes starting at 36 
            aSerial.v().p("rcvd note: ").p(chrom_to_dr202[note_pos][0]).p(
                             " velo: ").p(Velocity).p(" ");
            aSerial.v().p("conv note: ").p(chrom_to_dr202[note_pos][1]).p(
                             " velo: ").p(Velocity).p(" ");
            aSerial.v().p("roll note: ").p(chrom_to_dr202[note_pos][2]).p(
                             " velo: ").p(Velocity).p(" ");
            MIDI.sendNoteOn(chrom_to_dr202[note_pos][1], Velocity, midi_ch);
        }
    }
}

void handleNoteOff(byte Channel, byte PitchMidi, byte Velocity) { // NoteOn with 0 velo is NoteOff. 
    digitalWrite(ledPin, LOW);
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
