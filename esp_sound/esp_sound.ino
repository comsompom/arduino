/*
  Simple Melody Player for ESP32
  Plays "Twinkle, Twinkle, Little Star" on a small dynamic speaker.
  Hardware:
  - ESP32 Development Board
  - Small Speaker connected to GPIO 25 and GND
  Author: AI Assistant
*/

// Include a helper file that defines the musical note frequencies
#include "pitches.h"

// Define the pin where the speaker is connected
const int speakerPin = 25;

// Define the melody notes.
// A '0' represents a rest (silence).
int melody[] = {
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4,
  NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
  NOTE_G4, NOTE_G4, NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4,
  NOTE_C4, NOTE_C4, NOTE_G4, NOTE_G4, NOTE_A4, NOTE_A4, NOTE_G4,
  NOTE_F4, NOTE_F4, NOTE_E4, NOTE_E4, NOTE_D4, NOTE_D4, NOTE_C4
};

// Define the duration of each note.
// 4 = quarter note, 8 = eighth note, etc.
// A negative value makes it a dotted note (1.5x duration)
int noteDurations[] = {
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2,
  4, 4, 4, 4, 4, 4, 2
};

// --- You shouldn't need to change anything below this line ---

// The ESP32's tone generation uses "channels". We'll use channel 0.
const int ledcChannel = 0;
// We also set a resolution and a base frequency. These values are standard.
const int ledcResolution = 8;
const int ledcBaseFreq = 5000;

void setup() {
  // Setup the LEDC PWM channel
  // ledcSetup(channel, frequency, resolution)
  ledcSetup(ledcChannel, ledcBaseFreq, ledcResolution);

  // Attach the speaker pin to the LEDC channel
  ledcAttachPin(speakerPin, ledcChannel);
  
  Serial.begin(115200);
  Serial.println("Simple Melody Player - Starting now!");
}

void loop() {
  // Iterate through the notes of the melody:
  int totalNotes = sizeof(melody) / sizeof(melody[0]);
  for (int thisNote = 0; thisNote < totalNotes; thisNote++) {

    // Calculate the note duration in milliseconds.
    // 1000 divided by the note type (e.g., 1000 / 4 for a quarter note).
    int noteDuration = 1000 / noteDurations[thisNote];

    // Play the note on the specified channel
    // ledcWriteTone(channel, frequency)
    ledcWriteTone(ledcChannel, melody[thisNote]);
    
    // Hold the note for its duration
    delay(noteDuration);

    // Stop the sound between notes to make them distinct
    // We do this by setting the tone to 0 Hz, or silence.
    ledcWriteTone(ledcChannel, 0);

    // Add a brief pause between notes
    delay(50);
  }

  // Add a longer pause after the melody has finished playing
  Serial.println("Melody finished. Restarting in 3 seconds...");
  delay(3000);
}