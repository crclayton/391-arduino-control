#define encoder0PinA 2
#define encoder0PinB 3

volatile unsigned int encoder0Pos = 0;
boolean direction = true; // true= cw, false = ccw

void setup() {

  pinMode(encoder0PinA, INPUT); 
  pinMode(encoder0PinB, INPUT); 

// encoder pin on interrupt 0 (pin 2)

  attachInterrupt(0, doEncoderA, CHANGE);

// encoder pin on interrupt 1 (pin 3)

  attachInterrupt(1, doEncoderB, CHANGE);  

  Serial.begin (9600);
  Serial.println("Start"); 

}

void loop(){ 
  //Do stuff here 
  }

void doEncoderA(){

  // look for a low-to-high on channel A
  if (digitalRead(encoder0PinA) == HIGH) { 

    // check channel B to see which way encoder is turning
    if (digitalRead(encoder0PinB) == LOW) {  
      encoder0Pos = encoder0Pos + 1;         // CW
      direction = true; 
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
      direction = false; 
    }
  }

  else   // must be a high-to-low edge on channel A                                       
  { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinB) == HIGH) {   
      encoder0Pos = encoder0Pos + 1;          // CW
      direction = true; 
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
      direction = false; 
    }
  }
  Serial.println (encoder0Pos, DEC);          
  // decimal value of the encoder position 

}

void doEncoderB(){

  // look for a low-to-high on channel B
  if (digitalRead(encoder0PinB) == HIGH) {   

   // check channel A to see which way encoder is turning
    if (digitalRead(encoder0PinA) == HIGH) {  
      encoder0Pos = encoder0Pos + 1;         // CW
      direction = true; 
    } 
    else {
      encoder0Pos = encoder0Pos - 1;         // CCW
      direction = false; 
    }
  }

  // Look for a high-to-low on channel B

  else { 
    // check channel B to see which way encoder is turning  
    if (digitalRead(encoder0PinA) == LOW) {   
      encoder0Pos = encoder0Pos + 1;          // CW
      direction = true; 
    } 
    else {
      encoder0Pos = encoder0Pos - 1;          // CCW
      direction = false; 
    }
  }

} 



