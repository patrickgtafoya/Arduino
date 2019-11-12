/**************************************************************
 * Program: UDC_HyperLynx_sensorData                          *
 * Author:  Patrick Tafoya                                    *
 * Purpose: Utilize I2C protocol to read and communicate      *
 *          data from the Pod low voltage control system.     *
 *          Sensors include:                                  *
 *            MLX90614 IR Thermometer (2)                     *
 *            BNO055 9-axis Abs. Orientation (2)              *
 *            BME280 Pressure Sensor (2)                      *
 *            AttoPilot Voltage and Current sensor (1)        *
 *************************************************************/
#include <Wire.h> //Arduino I2C
#include <Adafruit_Sensor.h>  //Adafruit Unified sensor library
#include <SparkFunMLX90614.h> //IR THERM
#include <Adafruit_BNO055.h>  //BNO055-IMU
#include <Adafruit_BME280.h>
#include <utility/imumaths.h>

#define IR_ID 0x5B
#define BME280_ADDRESS_A 0x77
#define BME280_ADDRESS_B 0x76

const int dataArraySize = 5;
const int lostConnectionAbort = 1000;
const int checkConnection = 10;

IRTherm IR_temp;  //Object for IR therm
Adafruit_BNO055 IMU_nose = Adafruit_BNO055(55, BNO055_ADDRESS_A); //Object for IMU_nose
Adafruit_BNO055 IMU_tail = Adafruit_BNO055(55, BNO055_ADDRESS_B); //Object for IMU_tail
//Adafruit_BME280 PV_1 = Adafruit_BME280(BME280_ADDRESS_A); //Object for pressure sensor in PV_1
//Adafruit_BME280 PV_2 = Adafruit_BME280(BME280_ADDRESS_B); //Object for pressure sensor in PV_2

//IR Therm
float rawTemp[dataArraySize] = {0};
float filteredTemp;
//BNO055 Nose
float noseX[dataArraySize] = {0};
float filteredNoseX;
float noseY[dataArraySize] = {0};
float filteredNoseY;
float noseZ[dataArraySize] = {0};
float filteredNoseZ;
//BNO055 Tail
float tailX[dataArraySize] = {0};
float filteredTailX;
float tailY[dataArraySize] = {0};
float filteredTailY;
float tailZ[dataArraySize] = {0};
float filteredTailZ;

//FUNCTION HEADER
float medianFilter(float unfilteredData[], int dataArraySize);
void sort(float a[], unsigned int arraySize);
void connectionLost(void);

void setup() {
  Serial.begin(115200);
  sensorSetup();
}

void loop() {
  int currentIndex = 0; //Set/reset index for median filter
    
  do{
    //Check connection to IR Thermometer
    if(IR_temp.read())
    {
      rawTemp[currentIndex] = IR_temp.object();
      filteredTemp = medianFilter(rawTemp, dataArraySize);
    }
    else{
      //If connection is lost, abort after 1s
      connectionLost();
    }
    sensors_event_t event;
    if(IMU_nose.getEvent(&event)){
      noseX[currentIndex] = (float)event.orientation.x;
      filteredNoseX = medianFilter(noseX, dataArraySize);
      noseY[currentIndex] = (float)event.orientation.y;
      filteredNoseY = medianFilter(noseY, dataArraySize);
      noseZ[currentIndex] = (float)event.orientation.z;
      filteredNoseZ = medianFilter(noseZ, dataArraySize);
    }
    if(IMU_tail.getEvent(&event)){
      tailX[currentIndex] = (float)event.orientation.x;
      filteredTailX = medianFilter(tailX, dataArraySize);
      tailY[currentIndex] = (float)event.orientation.y;
      filteredTailY = medianFilter(tailY, dataArraySize);
      tailZ[currentIndex] = (float)event.orientation.z;
      filteredTailZ = medianFilter(tailZ, dataArraySize);
    }
    currentIndex++;
    delay(10);
  }
  while(currentIndex < dataArraySize);
  
  if(filteredTemp > 101.0){
    Serial.print("CRITICAL TEMP");
  }
  else{
    Serial.print(filteredTemp, 1);
    Serial.print("F");
    }
  Serial.print("\t");
  Serial.print("!");//Processing Flag
  Serial.print(filteredNoseX, 1);
  Serial.print("\t");
  Serial.print("@");//Processing Flag
  Serial.print(filteredNoseY, 1);
  Serial.print("\t");
  Serial.print("#");//Processing Flag
  Serial.print(filteredNoseZ, 1);
  Serial.print("\t");
  Serial.print("$");//Processing Flag
  Serial.print(filteredTailX, 1);
  Serial.print("\t");
  Serial.print("%");//Processing Flag
  Serial.print(filteredTailY, 1);
  Serial.print("\t");
  Serial.print("^");//Processing Flag
  Serial.print(filteredTailZ, 1);
  Serial.println("*");//Processing Flag

  delay(200);
}

void sensorSetup(void)
 {
  if(!IMU_nose.begin()){
    Serial.println("Connection ERROR with IMU_nose");
    while(1);
    }
  else if(!IMU_tail.begin()){
    Serial.println("Connection ERROR with IMU_tail");
    while(1);
  }
  else if(!IR_temp.begin(IR_ID)){
    Serial.println("Connection ERROR with IR Therm");
    while(1);  
  }
  /*
  else if(!PV_1.begin()){
    Serial.println("Connection ERROR with PV_1");
    while(1);
  }
  else if (!PV_2.begin()){
    Serial.println("Connection ERROR with PV_2");
    while(1);
  }
  /*
  else if(!analogRead(A0)){
    Serial.println("Connection ERROR with Voltage sensor");
  }
  else if(!analogRead(A1)){
    Serial.println("Connection ERROR with Current sensor");
  }*/
  else{
    IR_temp.setUnit(TEMP_F);
    Serial.println("All sensors Ready");
    delay(500);
  }
 }

 void sort(float a[], unsigned int arraySize){
  // implement sort

{
    //sort array in increasing numerical order
    //use to find median value
    for(int outer = 0; outer < arraySize; outer++)
    {
        for(int inner = 1; inner < arraySize; inner++)
        {
            if(a[outer] < a[inner])
            {
                int temp = a[outer];
                a[outer] = a[inner];
                a[inner] = temp;
            }
        }
    }
  }
}

float medianFilter(float unfilteredData[], int dataArraySize){  
  // locally declare array to store filtered data
  float filteredData[dataArraySize];
  // copy the contents of rawData to filtered data
  // do not want to sort the unfiltered Data, it is read only
  for(int i = 0; i < dataArraySize; i++){
    filteredData[i] = unfilteredData[i];
  }
  // sort the filterData array elements into ascending order
  sort(filteredData, dataArraySize);
  // The median element is stored in the middle position
  return filteredData[2];
 }

void connectionLost(void){
  //Begin timer upon lost connection
  unsigned long timeStart = millis();
  sensors_event_t event;
  //Continue timer while checking for connection
  while(!IR_temp.read() || !IMU_nose.getEvent(&event) || !IMU_tail.getEvent(&event)){
    unsigned long timeEnd = millis();
    delay(checkConnection);
    //Check duration of lost connection
    if((timeEnd - timeStart) > lostConnectionAbort){
      Serial.println("Sensor connection lost");
      while(1);   
    }
  }
 }

