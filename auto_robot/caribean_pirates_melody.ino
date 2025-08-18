/*
  Pirates of the Caribbean - "He's a Pirate" Theme
  
  Plays the melody on a piezo buzzer connected to pin 13.
  Uses the tone() function.
*/

// First, we need to define the frequencies of the musical notes.
// This section is like a mini library of pitches.
#define NOTE_B0  31
#define NOTE_C1  33
#define NOTE_CS1 35
#define NOTE_D1  37
#define NOTE_DS1 39
#define NOTE_E1  41
#define NOTE_F1  44
#define NOTE_FS1 46
#define NOTE_G1  49
#define NOTE_GS1 52
#define NOTE_A1  55
#define NOTE_AS1 58
#define NOTE_B1  62
#define NOTE_C2  65
#define NOTE_CS2 69
#define NOTE_D2  73
#define NOTE_DS2 78
#define NOTE_E2  82
#define NOTE_F2  87
#define NOTE_FS2 93
#define NOTE_G2  98
#define NOTE_GS2 104
#define NOTE_A2  110
#define NOTE_AS2 117
#define NOTE_B2  123
#define NOTE_C3  131
#define NOTE_CS3 139
#define NOTE_D3  147
#define NOTE_DS3 156
#define NOTE_E3  165
#define NOTE_F3  175
#define NOTE_FS3 185
#define NOTE_G3  196
#define NOTE_GS3 208
#define NOTE_A3  220
#define NOTE_AS3 233
#define NOTE_B3  247
#define NOTE_C4  262
#define NOTE_CS4 277
#define NOTE_D4  294
#define NOTE_DS4 311
#define NOTE_E4  330
#define NOTE_F4  349
#define NOTE_FS4 370
#define NOTE_G4  392
#define NOTE_GS4 415
#define NOTE_A4  440
#define NOTE_AS4 466
#define NOTE_B4  494
#define NOTE_C5  523
#define NOTE_CS5 554
#define NOTE_D5  587
#define NOTE_DS5 622
#define NOTE_E5  659
#define NOTE_F5  698
#define NOTE_FS5 740
#define NOTE_G5  784
#define NOTE_GS5 831
#define NOTE_A5  880
#define NOTE_AS5 932
#define NOTE_B5  988
#define NOTE_C6  1047
#define NOTE_CS6 1109
#define NOTE_D6  1175
#define NOTE_DS6 1245
#define NOTE_E6  1319
#define NOTE_F6  1397
#define NOTE_FS6 1480
#define NOTE_G6  1568
#define NOTE_GS6 1661
#define NOTE_A6  1760
#define NOTE_AS6 1865
#define NOTE_B6  1976
#define NOTE_C7  2093
#define NOTE_CS7 2217
#define NOTE_D7  2349
#define NOTE_DS7 2489
#define NOTE_E7  2637
#define NOTE_F7  2794
#define NOTE_FS7 2960
#define NOTE_G7  3136
#define NOTE_GS7 3322
#define NOTE_A7  3520
#define NOTE_AS7 3729
#define NOTE_B7  3951
#define NOTE_C8  4186
#define NOTE_CS8 4435
#define NOTE_D8  4699
#define NOTE_DS8 4978
#define REST      0 // A rest/pause has a frequency of 0

// The pin our buzzer is connected to.
const int buzzerPin = 13;

// The tempo of the song in beats per minute (BPM).
// The original is quite fast!
int tempo = 150; 

// The melody array: stores the sequence of notes.
// Using the note definitions from above.
int melody[] = {
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 
  NOTE_F5, NOTE_G5, NOTE_E5, NOTE_E5, 
  NOTE_D5, NOTE_C5, NOTE_C5, NOTE_D5, 
  
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 
  NOTE_F5, NOTE_G5, NOTE_E5, NOTE_E5, 
  NOTE_D5, NOTE_C5, NOTE_D5, REST, NOTE_A4,
  
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 
  NOTE_D5, NOTE_F5, NOTE_G5, NOTE_G5,
  NOTE_G5, NOTE_A5, NOTE_B5, NOTE_B5,
  NOTE_A5, NOTE_G5, NOTE_F5, NOTE_D5,
  
  NOTE_A4, NOTE_C5, NOTE_D5, NOTE_D5, 
  NOTE_D5, NOTE_E5, NOTE_F5, NOTE_F5, 
  NOTE_F5, NOTE_G5, NOTE_E5, NOTE_E5, 
  NOTE_D5, NOTE_C5, NOTE_D5, REST, NOTE_A4,
};

// The note duration array: stores the duration of each note.
// 4 = quarter note, 8 = eighth note, 16 = sixteenth note, etc.
// A negative number like -4 indicates a dotted note (1.5x duration).
int noteDurations[] = {
  8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 4,
  
  8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8,
  
  8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 8,
  4, 8, 8, 4, 8, 4
};

void setup() {
  // Set the buzzer pin as an output.
  pinMode(buzzerPin, OUTPUT);
}

void loop() {
  // Get the number of notes in the melody.
  int songLength = sizeof(melody) / sizeof(melody[0]);

  // Iterate through the notes of the melody:
  for (int thisNote = 0; thisNote < songLength; thisNote++) {

    // Calculate the note duration in milliseconds.
    // A quarter note is one beat. 
    // So, 60,000 ms (1 minute) / tempo (beats per minute) gives the duration of one beat.
    int quarterNote = 60000 / tempo;
    int noteDuration = quarterNote; // Default to quarter note
    
    // The noteDurations array tells us how to scale this.
    // e.g., 8 means it's an eighth note, so duration is quarterNote / 2.
    // 4 means it's a quarter note, so duration is quarterNote.
    if (noteDurations[thisNote] > 0) {
      noteDuration = (quarterNote * 4) / noteDurations[thisNote];
    } else if (noteDurations[thisNote] < 0) {
      // Dotted note, 1.5 times the duration
      noteDuration = (quarterNote * 4) / abs(noteDurations[thisNote]);
      noteDuration *= 1.5; 
    }

    // Play the note on the buzzer pin for the calculated duration.
    // The tone() function takes three arguments: pin, frequency, duration.
    tone(buzzerPin, melody[thisNote], noteDuration * 0.9); // 0.9 for a slight staccato feel

    // To distinguish the notes, we'll add a brief pause after each one.
    // The delay should be slightly longer than the note's duration to
    // create a small silent gap between notes.
    int pauseBetweenNotes = noteDuration;
    delay(pauseBetweenNotes);

    // Stop the tone generation.
    noTone(buzzerPin);
  }

  // Add a longer pause after the song finishes before it repeats.
  delay(2000);
}
