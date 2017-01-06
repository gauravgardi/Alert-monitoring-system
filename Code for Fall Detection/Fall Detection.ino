////////////////////////////headers///////////////////////////////////////////////////////////
#include<Wire.h>
#include<SoftwareSerial.h>

const int MPU=0x68; // I2C address of the MPU-6050
int16_t AcX[50],AcY[50],AcZ[50],GyX[50],GyY[50],GyZ[50]; //these are the reading updated using the below reading for algorithm. In the fucntion circular queue is implemented.
int16_t Ac_X,Ac_Y,Ac_Z,Gy_X,Gy_Y,Gy_Z,Tmp; //these are the fresh readings which are cont. updated in the function read_raw_values().
unsigned int k=0;
unsigned int l=0;
int16_t i=0;
int16_t j=0;
const int16_t tmin=1000;
const int16_t tmax=2000;
const int16_t Ath=15*15; //sqauaring the threshold value.
int16_t Gmax;
String st_to_display;
SoftwareSerial mySerial(10, 11); //Rx | Tx

//////////////////////////setup//////////////////////////////////////////////////////////////
void setup(){
Wire.begin();
Wire.beginTransmission(MPU);
Wire.write(0x6B); // PWR_MGMT_1 register
Wire.write(0); // set to zero (wakes up the MPU-6050)
Wire.endTransmission(true);
Serial.begin(9600);
//BLUETOOTH MODULE POWER SUPPLIES Vcc
pinMode(9, OUTPUT);      
digitalWrite(9, HIGH);
}

//////////////////////////algorithm function/////////////////////////////////////////////////
bool fall_algo(){
  //ALGORITHM FOR FALL DETECTION
  // j is for accelerometer array index and i is for gyroscope array index.
  if(j<50){
    AcX[j]=Ac_X; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
    AcY[j]=Ac_Y; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ[j]=Ac_Z; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    //Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    j++; k++;
  }
  else {
    j=0;
    AcX[j]=Ac_X; // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
    AcY[j]=Ac_Y; // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ[j]=Ac_Z; // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    //Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    j++; k++;
  }
  if(i<50){
    GyX[i]=Gy_X; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY[i]=Gy_Y; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ[i]=Gy_Z; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    i++; l++;
  }
  else {
    i=0;
    GyX[i]=Gy_X; // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY[i]=Gy_Y; // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ[i]=Gy_Z; // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
    i++; l++;
  }
  //taking the norm square.
  int16_t Acc=(AcX[j]*AcX[j]+AcZ[j]*AcZ[j]+AcY[j]*AcY[j]);
  int16_t Gy=(GyX[i]*GyX[i]+GyZ[i]*GyZ[i]+GyY[i]*GyY[i]);
  if(Acc>Ath){
    int k1=(k-1);
    while(1){
      if(j==49){
        j=0; i=0;
        GyX[i]=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
        GyY[i]=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
        GyZ[i]=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
        AcX[j]=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
        AcY[j]=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
        AcZ[j]=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
        Acc=sqrt(AcX[j]*AcX[j]+AcZ[j]*AcZ[j]+AcY[j]*AcY[j]);
        if(Acc<Ath) {
          k++; l++;
          break;
      }
    }
    else{
      AcX[j]=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
      AcY[j]=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
      AcZ[j]=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
      Acc=sqrt(AcX[j]*AcX[j]+AcZ[j]*AcZ[j]+AcY[j]*AcY[j]);
      GyX[i]=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
      GyY[i]=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
      GyZ[i]=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
      if(Acc<Ath){
        k++; l++;
        break;
      }
    } 
    j++; i++; k++; l++;
  }
  int16_t l1,l2,t3;
  if(k<k1)
  {
   t3=(k+255-k1)*10; //IN MILLISECONDS
  //older code
   l1=(k+255)%50+k1%50;
   l2=l1/2;
  //NEW CODE, CHECKING THE TIME RANGE FOR THE PEAK VALUES.
  }
  
 //SQUARING GMAX
 else{ t3=(k-k1)*10; //IN MILLISECONDS
  //older code
   l1=(k)%50+k1%50;
   l2=l1/2;
  //NEW CODE, CHECKING THE TIME RANGE FOR THE PEAK VALUES.
  }
  
  Gmax=GyX[l2]*GyX[l2]+GyZ[l2]*GyZ[l2]+GyY[l2]*GyY[l2];
  //final condititon for detection of fall.
    if(t3>tmin & t3<tmax & Gmax>(45*45)){
      Serial.print("FALL");
      return true;
    }
  }
}

//////////////////////////function for reading raw values////////////////////////////////////
void read_raw_values(){
  //CODE FOR READING RAW VALUES
  Ac_X=Wire.read()<<8|Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
  Ac_Y=Wire.read()<<8|Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Ac_Z=Wire.read()<<8|Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Gy_X=Wire.read()<<8|Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy_Y=Wire.read()<<8|Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gy_Z=Wire.read()<<8|Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("Ac_X = "); Serial.print(Ac_X);
  Serial.print(" | Ac_Y = "); Serial.print(Ac_Y);
  Serial.print(" | Ac_Z = "); Serial.print(Ac_Z);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53); //equation for temperature in degrees C from datasheet
  Serial.print(" | Gy_X = "); Serial.print(Gy_X);
  Serial.print(" | Gy_Y = "); Serial.print(Gy_Y);
  Serial.print(" | Gy_Z = "); Serial.println(Gy_Z);
  delay(100);
}







///////////////////////////MAIN LOOP///////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////
void loop(){
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU,14,true); // request a total of 14 registers
  
  read_raw_values(); //taking the fresh values from registers.
  
  //sending via bluetooth on true fall conditiion to phone
  if (fall_algo() == true){
    mySerial.println("emergency is true");
  }
  else{
    mySerial.println("No fall condition is detected");
  }
}









////////////////////////////////////////////manual trigger///////////////////////////////////////////////////////
void serialEvent() {
  while (Serial.available()) {
    // get the new byte:
    char inChar = (char)Serial.read();
    if( inChar == 'e')
    // trigger emergency situation.
    mySerial.println("emergency is true"); // sending through bluetooth module
    Serial.print("emergency is true");
   }
  }

