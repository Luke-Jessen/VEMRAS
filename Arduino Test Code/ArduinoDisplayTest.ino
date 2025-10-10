#include <SoftwareSerial.h>

#define SER     2
#define NOE     3
#define RCLK    4
#define SRCLK   5
#define NSRCLR  6


void dispWord(uint16_t word);
void intToArr16(uint8_t num, uint8_t *arr);
uint16_t numToWord(uint8_t num);
uint16_t moveBits(uint8_t *arr);
void clearDisp();

String sendMessage;
String receivedMessage;
SoftwareSerial Serial1(10, 11); // RX, TX

uint8_t recNum;






//https://docs.arduino.cc/learn/communication/uart/
void setup() {
  
  Serial1.begin(9600);
  Serial.begin(9600);   // Initialize Serial1 for sending data

  pinMode(SER, OUTPUT);
  pinMode(NOE, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SRCLK, OUTPUT);
  pinMode(NSRCLR, OUTPUT);

  digitalWrite(NOE, LOW);
  digitalWrite(NSRCLR, HIGH);
  digitalWrite(RCLK, LOW);
  digitalWrite(SRCLK, LOW);


  
}

void loop() {
  while (Serial1.available() > 0) {
    char receivedChar = Serial1.read();
    if (receivedChar == '\n') {
      Serial.println(receivedMessage);  // Print the received message in the Serial monitor
      recNum = ((receivedMessage[0]) *10) + (receivedMessage[1]);
      receivedMessage = "";  // Reset the received message
    } else {
      receivedMessage += receivedChar;  // Append characters to the received message
    }
  }

  recNum = recNum < 100 ? recNum : 99;

  dispWord(numToWord(recNum));
  
  // for(uint8_t i = 0; i < 100; i++){
  //   dispWord(numToWord(i));
    
  //   //dispWord(0b0000000010001000);
  //   delay(500);
  // }
    //clearDisp();
 

}

void dispWord(uint16_t word){
  
  Serial.println("Wrote:");

    for(uint8_t i = 0; i < 16; i++){
      digitalWrite(SER, (word >> i & 1));
      Serial.print(word >> i & 1);
      delayMicroseconds(100);
      digitalWrite(SRCLK, HIGH);
      delayMicroseconds(100);
      digitalWrite(SRCLK, LOW);
      delayMicroseconds(100);

      
       
    }
   digitalWrite(RCLK, HIGH);
   delayMicroseconds(100);
   digitalWrite(RCLK, LOW);
   delayMicroseconds(100);
    
  Serial.println();

}

uint16_t numToWord(uint8_t num){
  Serial.println("num:");
  Serial.println(num);

  static uint8_t lookup[] = {   0b0111111, 
                                0b0000110,
                                0b1011011,
                                0b1001111,
                                0b1100110,
                                0b1101101,
                                0b1111101,
                                0b0000111,
                                0b1111111,
                                0b1101111
                                };


  uint16_t word = 0;
  uint8_t dig1 = 0;
  uint8_t dig2 = 0;

  if(num < 100){
    dig1 = lookup[num%10];
    dig2 = lookup[num/10];

  }else{
    dig1 = lookup[9];
    dig2 = lookup[9];
    }
  Serial.println("Digits:");
  Serial.println(dig1, BIN);
  Serial.println(dig2, BIN);

  word = (uint16_t)dig1 << 8 | (uint16_t)dig2;

  Serial.println("Appended Digits:");
  Serial.println(word, BIN);

  uint8_t arr16[16];

  intToArr16(word, arr16);
  word = moveBits(arr16);

  Serial.println("Moved Bits:");
  Serial.println(word, BIN);

  return word;


}

void intToArr16(uint16_t num, uint8_t *arr){
  Serial.println("Num to beconverted to array:");
  Serial.println(num, BIN);
  

  for(uint8_t i = 0; i < 16; i++){
      arr[i] = (num >> i) & 1;
           
    }

  Serial.println("First int array");
    for(uint8_t i = 0; i < 16; i++){      
      Serial.print(arr[15-i]);       
    }
  Serial.println();
    
}

uint16_t moveBits(uint8_t *arr){
  uint8_t wordArr[16];

  wordArr[13] = arr[0];
  wordArr[12] = arr[1];
  wordArr[2] = arr[2];
  wordArr[1] = arr[3];
  
  wordArr[0] = arr[4];
  wordArr[14] = arr[5];
  wordArr[15] = arr[6];
  wordArr[3] = arr[7];
  wordArr[9] = arr[8];
  wordArr[8] = arr[9];
  wordArr[6] = arr[10];
  wordArr[5] = arr[11];
  wordArr[4] = arr[12];
  wordArr[10] = arr[13];
  wordArr[11] = arr[14];
  wordArr[7] = arr[15];

  uint16_t word = 0;

  for(uint8_t i = 0; i < 16; i++){
      word |= wordArr[i] << i;       
    }

    return word;

}


void clearDisp(){
  digitalWrite(NOE, HIGH);
  digitalWrite(NSRCLR, LOW);

  delay(1);

  digitalWrite(NOE, LOW);
  digitalWrite(NSRCLR, HIGH);
}
