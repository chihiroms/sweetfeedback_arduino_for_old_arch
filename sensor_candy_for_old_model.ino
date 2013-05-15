// These constants won't change.  They're used to give names
// to the pins used:

const String MachineID = "0118";

// Sensors' pin
const int MicInPin = A0;  // Analog input pin that the microphone is attached to
const int PhotodlnInPin = A1;  // Analog input pin that the photodarlington is attached to
const int ThermtrInPin = A2;  // Analog input pin that the thermistor is attached to
const int DistanceInPin = A3; // Analog input pin that the distance sensor is attached to

//Actuators' pin
const int MotorOutPin = 13; // Motor connected from digital pin 13 to ground


const int sensorNum = 5;
int sensorValue[sensorNum] = {0};        // initialize value read from the pot
int outputValue[sensorNum] = {0};        // initialize value output to the PWM (analog out)
int print_mask[sensorNum] = {0};       // 0 if the data is not going to show, 1 otherwise


const int MAX_DISTANCE = 1000;
int ledState = LOW;


boolean humanState = false;
int smoothedDistance = 0;
int balanceMicVal = -1;
enum tempUnit{Kelvin,Celcius,Fahrenheit};

const int numReadings = 500;
int micReadings[numReadings] = {0};
long total = 0;
int index = 0;
float outputNoiseLevel = -1;

//for group collaboration experiment
int count = 0;
void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  pinMode(MotorOutPin, OUTPUT);

  digitalWrite(MotorOutPin, HIGH);
  establishContact();  // send a byte to establish contact until receiver responds 
}

void loop() {
  getSensorData();
  serialCallResponse();
}

// Main Loop Tasks
void getSensorData() {
  // read the analog in value:
  sensorValue[0] = analogRead(MicInPin);
  sensorValue[1] = analogRead(PhotodlnInPin);  
  sensorValue[2] = analogRead(ThermtrInPin);  

  // Sensor calibration

  float newNoiseLevel = noiseLevel(sensorValue[0]);
  if(outputNoiseLevel == -1){
    outputNoiseLevel = newNoiseLevel;
  }else{
    outputNoiseLevel = lowpassFilter(newNoiseLevel,outputNoiseLevel,0.25);
  }
  outputValue[0] = outputNoiseLevel;
  outputValue[1] = map(sensorValue[1],  0, 1023, 0, 255);  
  outputValue[2] = thermistorCalibration(sensorValue[2], Celcius);  
}

void calcBalanceMicVal(int micVal){
  if(balanceMicVal == -1){
      balanceMicVal = micVal;
      for(int i = 0; i < numReadings; i++){
        micReadings[i] = micVal;
        total += micVal;
      }
  }
  else{
    // subtract the last reading:
    total= total - micReadings[index];         
    // read from the sensor:  
    micReadings[index] =  micVal; 
    // add the reading to the total:
    total= total + micReadings[index];       
    // advance to the next position in the array:  
    index = index + 1;                    
    balanceMicVal = total/numReadings; 
    // if we're at the end of the array...
    if (index >= numReadings)              
      // ...wrap around to the beginning: 
      index = 0;             
  }
}

void serialCallResponse(){
  if(Serial.available() > 0) {
    int inByte = Serial.read();
    int i;
    if (inByte == 'A') {
        giveCandies();
    } else if (inByte == 'B') {
        for(i = 0; i < sensorNum -1; i++){
            Serial.print(outputValue[i]);
              Serial.print(",");  
        }
        Serial.println(outputValue[i]);
    } else if (inByte == 'E') {
      Serial.println(MachineID);
    }
   
  }
}

int noiseLevel(int micVal){  
  calcBalanceMicVal(micVal);
  int difference = abs(balanceMicVal - micVal);
  if(difference< 3) 
      return 0;
  return 14.9620*(log(difference))-14.2486;
}

// This is the calibration function for thermistor P/N:NTCLE413E2103H400
// parameter RawADC is the analogReading from the thermistor
// parameter Unit is the temperature unit of the return value
double thermistorCalibration(int RawADC, int Unit) {
 long double temp;
 long double A,B,C,D;

// this is the coefficient for the thermistor P/N:NTCLE413E2103H400
  A = 0.0012;
  B = 2.2614e-004;
  C = 7.0822e-007;
  D = 6.7885e-008;
  double R = 1000;
  double RT = (1024*R/RawADC) - R;
  
// Steinhart–Hart equation
// {1 \over T} = A + B \ln(R) + C (\ln(R))^3 \, 
// check wiki for more info http://en.wikipedia.org/wiki/Steinhart%E2%80%93Hart_equation
   temp = log(RT);
   long double divisor = (A + (B + (C + D * temp)* temp )* temp);
   temp = 1/ divisor;
   temp += 4;
  if(Unit == Kelvin)
    return temp;
  else if(Unit == Celcius)
    return temp = temp - 273.15;            // Convert Kelvin to Celcius
  else if(Unit == Fahrenheit)
    return temp = (temp * 9.0)/ 5.0 + 32.0; // Convert Celcius to Fahrenheit
  
}

void giveCandies(){
  Serial.print('O');
  digitalWrite(MotorOutPin, LOW);
  digitalWrite(MotorOutPin, HIGH);   // candy coming
  delay(500); // THIS 500 IS IMPORTANT. WHEN IT'S 300, DOESN'T WORK.
  digitalWrite(MotorOutPin, LOW);
  delay(500);
}


void establishContact() {
  while (Serial.available() <= 0) {
    int i;
    Serial.print(MachineID);
    Serial.print(",");
    for(i = 1; i < sensorNum -1; ++i){ Serial.print("0,");}
    Serial.println("0");
    delay(500);
  }
}

float lowpassFilter(float newValue, float oldValue, float alpha) {
  return alpha * newValue + (1 - alpha) * oldValue;
}

