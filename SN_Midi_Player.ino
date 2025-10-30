// SN76489AN MIDI Player
// By Tim Johansen

#define SN_WE 8
#define SN_CLK 9
#define SN_READY 2                      // Currently unused

#define CLOCK_FREQUENCY 4000000

#define BUFFER_SIZE 20

#define BEAT_TOGGLE_KEY 21
#define BPM_DOWN_KEY 22
#define BPM_UP_KEY 23
#define NOISE_KEY_0 24
#define NOISE_KEY_1 26
#define NOISE_KEY_2 28

int outpins[] = {4,5,6,7,10,11,12,13};  // Arduino pins connected to data pins 0-7 on the SN chip
uint16_t note_table[128];               // Stores the calculated frequency for each MIDI note
uint8_t note_playing[3];                // Stores the note currently playing on a channel
uint8_t noise_key_playing = 0xFF;       // Stores the most recently pressed noise key.
                                        // The noise channel will be silenced only if this key is released.

uint8_t incoming_msg[3];                // Temp storage for the current incoming MIDI message
uint8_t msg_index = 0;                 
uint8_t msg_bytes_expected = 0;

uint8_t byte_buffer[BUFFER_SIZE];         // Storage for bytes waiting to be sent to the SN chip
uint8_t buffer_start = 0;
uint8_t buffer_length = 0;

bool beat_track_enabled = false;
uint8_t bpm = 165;                    // Beats per minute
uint16_t sixteenth_count = 0;         // The beat track timing is stored as sixteenth notes. In other words,
                                      // when the count reaches 16, one full beat has been played.
long sixteenth_interval = 0;          // How many milliseconds per sixteenth note.
long sixteenth_start_time = 0;        // The time (in ms) the current sixteenth note started.


struct Beat {
  uint8_t time;   // Time in sixteenth notes
  uint8_t data;   // 0-2 for noise frequency, or 128 for silence
};

struct Beat beat_sequence[] = { {.time=0, .data=1}, {.time=4, .data=128}, 
                                {.time=16, .data=0}, {.time=20, .data=128}, 
                                {.time=24, .data=0}, {.time=28, .data=128}, 
                                {.time=32, .data=1}, {.time=36, .data=128}, 
                                {.time=48, .data=0}, {.time=52, .data=128}, 
                                {.time=63, .data=128}
                                };
int beat_sequence_length = 8;   // How many beats until the sequence repeats.
int beat_index = 0;


void setup() {
  Serial.begin(9600);
  pinMode(SN_READY, INPUT);  
  pinMode(SN_WE, OUTPUT);

  digitalWrite(SN_WE, HIGH);
  
  for (int i = 0; i < 8; i++) {
    pinMode(outpins[i], OUTPUT);
  }

  // Clock signal setup
  pinMode(SN_CLK, OUTPUT);
  TCNT1 = 0;
  TCCR1B = 0b00001001;
  TCCR1A = 0b01000000;
  OCR1A = 1; // 4mhz

  delay(100);
  
  silence_all();

  generate_note_table();
  set_bpm(120);  
}

void loop() {  
  read_midi_from_serial();
  process_buffer();

  if (beat_track_enabled){
    process_beat_track();
  }
}

void read_midi_from_serial(){
  if (!Serial.available()){    
    return;
  }
  uint8_t byte = Serial.read();

    if ((byte & 0b10000000) == 0b10000000) {
      // Incoming status byte (the start of a MIDI message)
      uint8_t msg_type = byte & 0b11110000;
      if (msg_type == 144 || msg_type == 128){  // We only care about "key on" and "key off" messages
        msg_bytes_expected = 3;
        incoming_msg[0] = byte;
        msg_index = 1;
      }
      else {
        // Unhandled type, so we ignore it and any following data bytes.
        msg_bytes_expected = 0;
      }
    }
    else {
      // Incoming data byte
      if (msg_bytes_expected == 0){
        return;
      }
      incoming_msg[msg_index] = byte;
      msg_index++;       
    }
    
    if (msg_bytes_expected > 0 && msg_index == msg_bytes_expected) {
      process_midi_message(&incoming_msg[0]);
  }    
}

void process_midi_message(uint8_t* msg){
    uint8_t type = *msg & 0b11110000;              
    bool is_key_on_msg = (type == 144);
    uint8_t note = *(msg + 1);
    
    // Check first if the key is assigned to beat controls
    if (is_key_on_msg){
      if (note == BEAT_TOGGLE_KEY){
        toggle_beat_track();
        return;
      }
      if (note == BPM_DOWN_KEY){
        set_bpm(bpm - 5);
        return;
      }
      if (note == BPM_UP_KEY){
        set_bpm(bpm + 5);
        return;
      }

      // Check for noise keys next
      if (note == NOISE_KEY_0){
        play_noise(1, 0);
        noise_key_playing = note;
        return;
        }
      if (note == NOISE_KEY_1){
        play_noise(1, 1);
        noise_key_playing = note;
        return;
        }
      if (note == NOISE_KEY_2){
        play_noise(1, 2);
        noise_key_playing = note;
        return;
      }
      
      // Are there any available channels for the note?
      bool open_channel_found = false;
      for (int i = 0; i < 3; i++) {
        if (note_playing[i] == 0xFF){
          play_note(i, note);          
          open_channel_found = true;
          break;
        }
      }
      // If all channels are currently playing notes, overwrite the channel that is least likely to sound disruptive.
      if (!open_channel_found){
        int highest_note = 0x00;
        int lowest_note = 0xFF;
        int highest_channel = 0;
        int lowest_channel = 0;
        for (int i = 0; i < 3; i++) {             // Find the channels playing the highest and lowest notes
          if (note_playing[i] > highest_note){     
            highest_note = note_playing[i];
            highest_channel = i;
          }
          if (note_playing[i] < lowest_note){
            lowest_note = note_playing[i];
            lowest_channel = i;
          }
        }
        if (note > highest_note){                 
          play_note(highest_channel, note);
        }
        else if (note < lowest_note){             
          play_note(lowest_channel, note);
        }
        else {                                    
          // If the new note is neither the new highest or lowest, 
          // replace the first channel that's also neither highest or lowest
          for (int i = 0; i < 3; i++) {
            if (i != highest_channel && i != lowest_channel){
                play_note(i, note);
                break;
            }
          }
        }
      }
    }
    else {    // Key off
      if (noise_key_playing == note){
        noise_key_playing = 0xFF;        
        silence_channel(3);
        return;
      }
      for (int i = 0; i < 3; i++) {
        if (note == note_playing[i]){
          silence_channel(i);
        }
      }
    }  
}

bool send_byte(uint8_t byte){
  if (buffer_length >= BUFFER_SIZE){
    Serial.println("Buffer full");
    return false;
  }
  byte_buffer[(buffer_start + buffer_length) % BUFFER_SIZE] = byte;
  buffer_length++;
  return true;
}

void process_buffer(){
  if (buffer_length == 0){
    return;
  }
  
  // In theory, this should wait for the SN to signal that it's ready for more data,
  // but a very brief delay seems to get the job done fine.

  // if (digitalRead(SN_READY == LOW)){
  //   // The chip isn't ready to accept more data yet
  //   return;
  // }

  digitalWrite(SN_WE, HIGH); 
  
  for (int i = 0; i < 8; i++) {    
    uint8_t bit = (uint8_t)(byte_buffer[buffer_start] >> (7 - i)) & 1;
    digitalWrite(outpins[i], bit);
  }
  delay(1);
  digitalWrite(SN_WE, LOW);   // Signal the chip that data is ready to be read
  buffer_length--;
  buffer_start = (buffer_start + 1) % BUFFER_SIZE;
  return;
}

void process_beat_track(){
  long beat_progress = millis() - sixteenth_start_time;
  if (beat_progress >= sixteenth_interval){
    int overshoot = beat_progress - sixteenth_interval;
    sixteenth_start_time = millis() + overshoot;
    sixteenth_count++;

    if (sixteenth_count == beat_sequence_length * 16){
      sixteenth_count = 0;
      beat_index = 0;
    }
  }
  
  if (sixteenth_count >= beat_sequence[beat_index].time){ 
    if (beat_sequence[beat_index].data != 0b10000000){
      play_noise(1, beat_sequence[beat_index].data);     
    }
    else {
      silence_channel(3);
    }
    beat_index++;
  }
}

void play_note(uint8_t channel, uint8_t note){

  if (note_table[note] > 0x3ff) {
    // Out of frequency range for the SN chip  
    return;
  }
  set_frequency(channel, note_table[note]);
  set_attenuation(channel, 0b00000000);
  note_playing[channel] = note;
}

void play_noise(uint8_t noise_type, uint8_t shift_rate){
  uint8_t byte = 0b11100000;
  if (noise_type == 1){
    byte |= 0b00000100;
  }
  byte |= shift_rate;
  send_byte(byte);
  set_attenuation(3, 0b00000000);    
}

void silence_channel(uint8_t channel){
  set_attenuation(channel, 0b00001111);
  note_playing[channel] = 0xFF;
}

void set_frequency(uint8_t channel, uint16_t freq){
  // The chip expects a 10-bit frequency value split across two bytes.
  // F0 is the most significant bit.

  // Command byte:  1,R0,R1,R2,F6,F7,F8,F9
  // Data byte:     0,xx,F0,F1,F2,F3,F4,F5
  uint8_t ms_byte = (uint8_t)(freq >> 8);
  uint8_t ls_byte = (uint8_t)(freq);
  uint8_t command_byte = 0b10000000;
  uint8_t data_byte = 0;

  command_byte |= (channel << 5);
  
  uint8_t lsb_masked = ls_byte & 0b00001111;
  command_byte |= lsb_masked;

  uint8_t msb_masked = (ms_byte << 4) & 0b00110000;
  lsb_masked = (ls_byte >> 4) & 0b00001111;         
  data_byte = msb_masked | lsb_masked;

  bool status = send_byte(command_byte);
  if (status)
    send_byte(data_byte);
}

void set_attenuation(uint8_t channel, uint8_t att){
  uint8_t data = 0b10010000;
  data |= channel << 5;
  data |= att & 0b00001111; 

  send_byte(data);
}

void generate_note_table(){
  // Calculates the frequency for each MIDI note on startup
  float note_frequency;
  for (int i = 0; i < 128; i++) {
    note_frequency = (float)440 * pow(2, (float)(i - 69)/ (float)12);
    // The SN chip expects the frequency as a ten-bit value according to the following formula
    uint16_t freqSN = (float)CLOCK_FREQUENCY / (32.0 * note_frequency);
    note_table[i] = freqSN;
  }
}

void silence_all(){
  silence_channel(0);
  silence_channel(1);
  silence_channel(2);
  silence_channel(3);
}

void toggle_beat_track(){
  beat_track_enabled = !beat_track_enabled;
  sixteenth_start_time = millis();
  silence_channel(3);
}

void set_bpm(uint8_t new_bpm){
  if (new_bpm <= 0){
    return;
  }
  bpm = new_bpm;
  sixteenth_interval = 60000 / bpm / 16;  
}
