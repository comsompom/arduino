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

void setup() {
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

    // Play the note on the speaker pin
    // tone(pin, frequency)
    tone(speakerPin, melody[thisNote]);

    // Hold the note for its duration
    delay(noteDuration);

    // Stop the sound between notes to make them distinct
    // We do this by calling noTone() to stop the sound.
    noTone(speakerPin);

    // Add a brief pause between notes
    delay(50);
  }

  // Add a longer pause after the melody has finished playing
  Serial.println("Melody finished. Restarting in 3 seconds...");
  delay(3000);
}