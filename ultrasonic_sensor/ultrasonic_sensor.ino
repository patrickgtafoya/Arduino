/*********************************************************************
 *  Class: ELEC 1520 EMBEDDED SYSTEMS ENGINEERING I                  *
 *  Author: Patrick Tafoya                                           *
 *  Instructor: Diane Williams                                       *
 *  Date: 05/01/2018                                                 *
 *  Assignment: Arduino Ultra Sonic Sensor Lab                       *
 *                                                                   *
 *  Written and Tested for:                                          *
 *    Software: Arduino v1.8.5                                       *
 *    Hardware: Arduino Uno                                          *
 *    OS: Windows 10                                                 *
 *                                                                   *
 *  Description:                                                     *
 *    Create a program that utilizes both the Ultrasonic sensor      *
 *    and DHT11 Temp and Humidity module to calculate the distance   *
 *    of an object. LED indicator will light when object             *
 *    is within the range of 30cm and increase in brightness the     *
 *    closer it gets to the sensor. A median filter will be used     *
 *    to filter out spikes in the raw data. Temp will be used to     *
 *    update speed of sound for calculations                         *
 ********************************************************************/


#include <SimpleDHT.h>

//Sample temp every 5 sec, 1/5Hz
#define TEMP_SAMPLE 5000
//defaults to be displayed if DHT11 fails
#define DEFAULT_TEMP 20
#define DEFAULT_HUMIDITY 15

const int ledPin = 3;
const int tempPin = 2;
const int trigPin = 12;
const int echoPin = 11;
int distanceArray[5];
static unsigned long tempTime;
static unsigned long ledTime;
//Initialize DHT11
SimpleDHT11 therm;
static int distance;
static int ledValue;
//Initialize temp and humidity to default
byte currentTemp = DEFAULT_TEMP;
byte currentHum = DEFAULT_HUMIDITY;

//FUNCTION HEADER//
unsigned long get_ultrasonic_measurement(void);
unsigned long calcDistance(void);
void fillArray(int a[], int aSize);
void bubbleSort(int a[], int aSize);
int filterMedian(int a[], int aSize);
void updateLed(void);
bool readTemperature(byte *temperature, byte *humidity, byte data[40]);
void updateTemp(void);


void setup() {
  //initialize serial communication
  Serial.begin(9600);
  //Pin 12 to output mode
  pinMode(trigPin, OUTPUT);
  //Pin 11 to input mode
  pinMode(echoPin, INPUT);
  //Initialize time for Temp read
  tempTime = millis();
  
}

void loop(){
  //update temp to use in distance calculations
  updateTemp();
  //fill array with distance values for filtering
  fillArray(distanceArray, 5);
  //distance is Median filter value
  distance = filterMedian(distanceArray, 5);
  //update LED brightness
  updateLed();
  //Print results to serial monitor
  if(distance > 30){
    Serial.println("");
    Serial.println("Maximum Distance Exceeded");
    Serial.println("");
    delay(1000);
  }
  else{
    Serial.println("");
    Serial.print("Filtered Distance: ");
    Serial.print(distance);
    Serial.println("cm");
    Serial.print("Temperature: ");
    Serial.print(currentTemp);
    Serial.println("°C");
    Serial.print("Humidity: ");
    Serial.print(currentHum);
    Serial.println("%");
    Serial.println("");
    delay(1000);
  }
}





/**********
 * Name: get_ultrasonic_measurement
 * Description: uses ultrasonic sensor to return an echo pulse length
 * Parameters: void
 * Return: unsigned long echo pulse length
 **********/
unsigned long get_ultrasonic_measurement(void)
{
  unsigned long startTime;
  unsigned long endTime;
  unsigned long elapsedTime;
  //Set trigger line low for a brief interval, 2 microseconds
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  //Change the trigger line from low to high
  digitalWrite(trigPin, HIGH);
  //Hold the trigger line high for at least 10 microseconds
  delayMicroseconds(10);
  //Pull trigger line low
  digitalWrite(trigPin, LOW);
  //Start timer when the echo line rises from low to high
  while(digitalRead(echoPin) == LOW){}
  startTime = micros(); 
  //Continue timing until echo line falls from high to low
  while(digitalRead(echoPin) == HIGH){}
  //Stop timer
  endTime = micros();
  //Measure elapsed time
  elapsedTime = endTime - startTime;
  return elapsedTime;
  //delay(100);
}

/**********
 * Name: calcDistance
 * Description: Uses echo pulse measurment to calculate distance
 * Parameters: void
 * Return: Unsigned long object distance in CM
 **********/
unsigned long calcDistance(void)
{
  static unsigned long pulseDuration;
  static unsigned long distanceCm;
  unsigned long echoWidth = pulseWidth();
  pulseDuration = get_ultrasonic_measurement();
  distanceCm = pulseDuration / echoWidth;
  return distanceCm;
  //10Hz = 10 times per second = 100 milliseconds
  delay(100);
}

/**********
 * Name: fillArray
 * Description: Converts distance measurements to ints and stores in array for filtering
 * Parameters: int a[], a one dimensional array
 *             int aSize, the size of the array
 * Return: void
 **********/
void fillArray(int a[], int aSize)
{
  Serial.print("Unfiltered Data: ");
  for(int i = 0; i < aSize; i++)
  {
    a[i] = int(calcDistance());
    Serial.print(a[i]);
    Serial.print(", ");
  }
  
}

/**********
 * Name: bubbleSort
 * Description: Sorts array elements in increasing numerical value
 * Parameters: int a[], a one dimensional array
 *             int aSize, the size of the array
 * Return: void
 **********/
void bubbleSort(int a[], int aSize)
{
    //sort array in increasing numerical order
    //use to find median value
    for(int outer = 0; outer < aSize; outer++)
    {
        for(int inner = 1; inner < aSize; inner++)
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

/**********
 * Name: filterMedian
 * Description: finds median of sorted 5 element 1D array
 * Parameters: int a[], a one dimensional array
 *             int aSize, the size of the array
 * Return: int median value
 **********/
int filterMedian(int a[], int aSize)
{
  int medianValue;
  bubbleSort(a, aSize);
  //array will have aSize 5
  //median will be in a[2] after bubble sort
  medianValue = (a[2]);

  return medianValue;
}

/**********
 * Name: updateLed
 * Description: Updates the brightness of LED indicator based off object distance
 *              converts distance to appropriate analog read value from 0-255
 * Parameters: void
 * Return: void
 ***********/
void updateLed(void)
{
  //LED off if max distance passed
  if(int(distance) > 30)
  {
    ledValue = 0;
  }
  else
  {
    //LED brighter the closer the object
    //Distance Range is 2cm - 30cm
    //255 / 30 = 8.5
    ledValue = (255 - (int(distance * 8.5)));
  }
  //Write value to LED pin
  analogWrite(ledPin, ledValue);
}

/**********
 * Name: readTemperature
 * Description: Tests functionality of DHT11
 * Parameters: byte *temperature, pointer to temp read by unit
 *             byte *humidity, pointer to humidity read by unit
 *             byte data[40],
 * Return: Boolean, true or false
 **********/
bool readTemperature(byte *temperature, byte *humidity, byte data[40])
{
  //Test if thermometer is functioning
  if (therm.read(tempPin, temperature, humidity, data)) {
    return false;
  }
  else return true;
}

/**********
 * Name: updateTemp
 * Description: Updates temp and humidity ONLY if readTemperature returns true
 * Parameters: void
 * Return: void
 **********/
void updateTemp(void)
{
  static byte temperature, humidity, data[40];
  
  if( millis() - tempTime > TEMP_SAMPLE){
    //update values ONLY when read is successful
    if(readTemperature(&temperature, &humidity, data) == true){
      currentTemp = temperature;
      currentHum = humidity;
    }
    //update time for next reading
    tempTime = millis();
  }
}

/**********
 * Name: pulseWidth
 * Description: Calculates the echo pulse width based off adjusted speed of
 *              sound determined by current temperature reading.
 * Parameters: void
 * Return: unsigned long, pulse width.
 **********/
unsigned long pulseWidth(void)
{
  double velocityMps;
  double velocityCmpus;
  double deltaTus;
  //account for temperature
  velocityMps = (331.4 + (0.06 * double(currentTemp)));
  velocityCmpus = (velocityMps / 10000.0);
  deltaTus = (1 / velocityCmpus);
  return (long(2.0 * deltaTus));  
}

