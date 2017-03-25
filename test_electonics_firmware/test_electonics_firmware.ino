
/*
This is the simplest possible firmware. All it does is run all for motors forward for 1 second, pause for 1 second, then 
go in reverse for one second. 

It also prints "Connection Good!" every 500ms to test the serial connection.

It is a good idea to remove the chains from the motors before running this test.
*/

#define IN1 9
#define IN2 8
#define IN3 11
#define IN4 10
#define IN5 12
#define IN6 13

#define ENA 6
#define ENB 7
#define ENC 5

void setup(){
    Serial.begin(19200);
    
    //set enable pins to outputs
    pinMode(ENA,   OUTPUT); 
    pinMode(ENB,   OUTPUT); 
    pinMode(ENC,   OUTPUT);
    
    //set direction pins to outputs
    pinMode(IN1,   OUTPUT); 
    pinMode(IN2,   OUTPUT); 
    pinMode(IN3,   OUTPUT);
    pinMode(IN4,   OUTPUT); 
    pinMode(IN5,   OUTPUT); 
    pinMode(IN6,   OUTPUT);
    
}


void loop(){
    //enable all motors
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    analogWrite(ENC, 255);
    
    //Command motors to go one way
    digitalWrite(IN1,   LOW); 
    digitalWrite(IN2,   HIGH); 
    digitalWrite(IN3,   LOW);
    digitalWrite(IN4,   HIGH); 
    digitalWrite(IN5,   LOW); 
    digitalWrite(IN6,   HIGH);
    
    
    Serial.println("Connection Good!");
    delay(500);
    Serial.println("Connection Good!");
    delay(500);
    
    //stop all motors
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    analogWrite(ENC, 0);
    
    Serial.println("Connection Good!");
    delay(500);
    Serial.println("Connection Good!");
    delay(500);
    
    //enable all motors
    analogWrite(ENA, 255);
    analogWrite(ENB, 255);
    analogWrite(ENC, 255);
    
    //Command motors to go the other way
    digitalWrite(IN1,   HIGH); 
    digitalWrite(IN2,   LOW); 
    digitalWrite(IN3,   HIGH);
    digitalWrite(IN4,   LOW); 
    digitalWrite(IN5,   HIGH); 
    digitalWrite(IN6,   LOW);
    
    Serial.println("Connection Good!");
    delay(500);
    Serial.println("Connection Good!");
    delay(500);
    
    //stop all motors
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
    analogWrite(ENC, 0);
    
    Serial.println("Connection Good!");
    delay(500);
    Serial.println("Connection Good!");
    delay(500);

}
