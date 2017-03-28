
volatile long leftEncoder = 0;
volatile long rightEncoder = 0;

void setup()
{
  Serial.begin(115200);

  pinMode(2, INPUT);
  pinMode(3, INPUT);
  pinMode(4, INPUT);
  pinMode(5, INPUT);
  attachInterrupt(0, encoder_one, CHANGE);
  attachInterrupt(1, encoder_two, CHANGE);
}


void loop()
{
}

void encoder_one()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b00010100) >> 2; // Interrupt on 2, regular on 4 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 1) & 0b10);
  enc_val = enc_val | pins;

  leftEncoder += lookup_table[enc_val & 0b1111];
  //Serial.print("Left position: ");
  //Serial.println(leftEncoder);
}

void encoder_two()
{
  // 0000, 0001, 0010, 0011, 0100, 0101, 0110, 0111,
  // 1000, 1001, 1010, 1011, 1100, 1101, 1110, 1111
  static int8_t lookup_table[] = {0,1,0,-1, -1,0,1,0, 0,1,0,-1, -1,0,1,0};
  // Negative lookups correspond to the sensors moving with 2 leading
  static uint8_t enc_val = 0;
  
  enc_val = enc_val << 2;
  // PIND reads the first 8 digital pins (0-7)
  uint8_t pins = (PIND & 0b00101000) >> 3; // Interrupt on 3, regular on 5 (interrupts available only on 2,3)
  pins = (pins & 0b1) | ((pins >> 1) & 0b10);
  enc_val = enc_val | pins;

  rightEncoder += lookup_table[enc_val & 0b1111];
  //Serial.print("Right position: ");
  //Serial.println(rightEncoder);
}

// For debugging
void print_PIND()
{
  Serial.print((PIND & 0b10000000) >> 7);
  Serial.print((PIND & 0b01000000) >> 6);
  Serial.print((PIND & 0b00100000) >> 5);
  Serial.print((PIND & 0b00010000) >> 4);
  Serial.print((PIND & 0b00001000) >> 3);
  Serial.print((PIND & 0b00000100) >> 2);
  Serial.print((PIND & 0b00000010) >> 1);
  Serial.print((PIND & 0b00000001));
  Serial.println();
}

