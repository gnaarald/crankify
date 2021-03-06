#include <HX711.h> //load cell amp library (bogde)
#include <Adafruit_Sensor.h> // 10DOF sensor libraries with i2c support
#include <Wire.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_L3GD20_U.h>
#include <Adafruit_LSM303_U.h>

#define UCHAR unsigned char
#define MESG_TX_SYNC                      ((UCHAR)0xA4)
#define MESG_SYSTEM_RESET_ID              ((UCHAR)0x4A)
#define MESG_NETWORK_KEY_ID               ((UCHAR)0x46)
#define MESG_ASSIGN_CHANNEL_ID            ((UCHAR)0x42)
#define MESG_CHANNEL_ID_ID                ((UCHAR)0x51)
#define MESG_CHANNEL_RADIO_FREQ_ID        ((UCHAR)0x45)
#define MESG_CHANNEL_MESG_PERIOD_ID       ((UCHAR)0x43) // Set channel period 0x43
#define MESG_RADIO_TX_POWER_ID            ((UCHAR)0x47) // Set Tx Power 0x47
#define MESG_CHANNEL_SEARCH_TIMEOUT_ID    ((UCHAR)0x44) // Set Channel Search Timeout 0x44
#define MESG_OPEN_CHANNEL_ID              ((UCHAR)0x4B) // ID Byte 0x4B
#define MESG_BROADCAST_DATA_ID            ((UCHAR)0x4E)

// include libraries:
#include <TimerOne.h>
#include <SoftwareSerial.h>

const int RTS_PIN = 2; // set pin 7 as the ready to send (RTS) for ANT

//message processor
boolean msgsync = 0;
uint8_t sync = 0;
uint8_t msglength = 0;
uint8_t msgbuf[10];
uint8_t sendcount = 0;

// ANT+ requirements
uint16_t ANT_power_total = 0; 

//--------Temp., pressure, accelerometer, compass, gyro SCL & SDA-----------
const int SCL_PIN = 19;
const int SDA_PIN = 18;
//-------ANT+ module pins-----------
const int SUSPEND_PIN = 6;
const int SLEEP_PIN   = 5;
const int RESET_PIN   = 8;
const byte rxPin = 12; // set pin 12 as the software serial rx
const byte txPin = 11; // set pin 11 as the software serial tx
//---------load cell pins---------
const byte DOUT = 3;
const byte CLK = 7;

//Change this calibration factor as per your load cell once it is found you many need to vary it in thousands
float calibration_factor = -106600;
 //-106600 worked for my 40Kg max scale setup. Use EEPROM to set the factor according to temp

Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);
Adafruit_L3GD20_Unified       gyro  = Adafruit_L3GD20_Unified(20);
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 42.19F, 29.69F, 4.71F};

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { { 1.478, 0.701, -0.299 },
                                    { 0.701, 1.260, -0.080 },
                                    { -0.299, -0.080, 0.794 } }; 

float mag_field_strength        = 41.23F;

HX711 scale(DOUT, CLK);
uint16_t HX711_instPower = 0;

SoftwareSerial ANTSerial(rxPin, txPin); // RX, TX

volatile int state = 0;

int tilt;
unsigned long angle_time = 0;
int prev_angle = 0;
uint8_t RPM;
uint16_t ANT_Torque = 0;
uint16_t ANT_Period = 0;
uint16_t ANT_Power = 0;
double Torque_sum = 0;
uint8_t ANT_ticks = 0;

void isr_ant()
{
  state = 1;
}
//debug only
int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

void setup() {
  //Serial.begin(19200);
  Serial.begin(115200);
  while (!Serial)
  {
    ;
  }
  pinMode(SUSPEND_PIN, OUTPUT);
  pinMode(SLEEP_PIN,   OUTPUT);
  pinMode(RESET_PIN,   OUTPUT); 
  // set the RTSPin as an input:
  pinMode (RTS_PIN, INPUT_PULLUP);  
  //We setup an interrupt to detect when the RTS is received from the ANT chip.
  //This is a 50 usec HIGH signal at the end of each message
  attachInterrupt(0/*digitalPinToInterrupt(RTS_PIN)/*0==pin2==RTS_PIN*/, isr_ant, RISING);
  digitalWrite(RESET_PIN,   HIGH);
  digitalWrite(SUSPEND_PIN, HIGH);
  digitalWrite(SLEEP_PIN,   LOW);
  ANTSerial.begin(9600);
  delay(100);
  scale.set_scale();  // Start scale
  delay(5000);
  Serial.flush();
  delay(50);
  initiate();
  init_hx711();
  calibrate_hx711();
  initSensors();
  Serial.println("setup done, exit the function");
}

void read_hx711(){
    Serial.print("ANT_Torque: ");
    Serial.println(ANT_Torque);
    ANT_Torque += abs(scale.read() * 0.0098065f) * 0.1725f;
    Torque_sum = ANT_Torque * 2;

  if(ANT_Torque > 65535) ANT_Torque = 0;
  }
  
void calibrate_hx711(){
  long zero_factor = scale.read_average(byte(10)); //Get a baseline reading
  Serial.print("Zero factor: "); //This can be used to remove the need to tare the scale. Useful in permanent scale projects.
  Serial.println(zero_factor);
  //scale.power_down();
  }
void init_hx711(){
  scale.power_up();
  scale.set_gain(byte(1));
  scale.tare();
  scale.set_scale(calibration_factor); //Adjust to this calibration factor
  }

void initSensors()
{
  if(!gyro.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no L3GD20 detected ... Check your wiring!");
    while(1);
  }
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}
void angularVelociy(){
  if(angle_time == 0){
    prev_angle = tilt;
    displaySensorData();
    angle_time = millis();
    //angularVelocity(); //recursive call
  }
  else{
    /*   w = da/dt    */
    RPM = uint8_t(long(((prev_angle - tilt)%360)/((millis()-angle_time)*1000000.0F) * 0.1667F));
    Serial.print("RPM: ");
    Serial.println(uint8_t(long(((prev_angle - tilt)%360)/((millis()-angle_time)*1000000.0F) *0.166666667F)));
    Serial.print("ANT_Period: ");
    Serial.println(ANT_Period);
    ANT_Period += uint16_t(double(millis()-angle_time)*2.048f);
    if(prev_angle > tilt){
      //prev ~300, new ~60
      ANT_ticks++;
    }
    if(ANT_ticks > 255) ANT_ticks = 0;
    prev_angle = tilt;
    Serial.print("tilt: ");
    Serial.println(tilt);
    angle_time = millis();
  }
  ANT_Power +=uint16_t(double(Torque_sum * (double(RPM)/60)*6.28));
}

void displaySensorData(){
  float roll, pitch, heading;
  sensors_event_t mag_event;

  mag.getEvent(&mag_event);

  sensors_vec_t   orientation;

  if (dof.magGetOrientation(SENSOR_AXIS_Z, &mag_event, &orientation))
  {
    /* 'orientation' should have valid .heading data now */
    tilt = (int)orientation.heading;
  }
}

int state_counter;
int event_counter = 0;
boolean clear_to_send = false;
short event_count_basic = 0; // this is basicpower update increment
short event_count_torque = 0;
int loop_skip = 0;

void loop() {
  angularVelociy();
  Serial.print("RPM: ");
  Serial.println(RPM);
  //ANTrec();
  read_hx711();
  ANTsendreceive();
}
void ANTsendreceive(){
  int sbuflength = ANTSerial.available();
  uint8_t msg = 0;
  while(sbuflength > 0)
  {
    if (msgsync == 0)
    {
      msg = ANTSerial.read();
      if (msg == 0xA4)
      {
        msgsync = 1;
        sync = 0;
      }
      sbuflength = ANTSerial.available();

    }
    else if (msgsync == 1 && sync == 0)
    {
      msglength = ANTSerial.read();
      sbuflength = ANTSerial.available();
      msglength += 3;
      msgbuf[0] = 0xA4;
      msgbuf[1] = (msglength-3);
      sync++;
    }
    else if (msgsync == 1 && sync > 0 && sync < msglength)
    {
      msgbuf[sync+1] = ANTSerial.read();
      sbuflength = ANTSerial.available();
      sync++;
    }
    else if (msgsync == 1 && sync == msglength)
    {
      msgbuf[sync+1] = ANTSerial.read();
      sbuflength = ANTSerial.available();
      msgsync = 0;
      sync = 0;
      msglength = 0;
      ANTrecproc(msgbuf, msglength);
    }
  }
  if(state == 1)
  {
    //Clear the ISR
    state = 0;
    
    if( digitalRead(RTS_PIN) == LOW )
    {
      displaySensorData(); //use roll value for angle
      clear_to_send = true;
    }
    else
    {
      //Need to make sure it is low again
      while( digitalRead(RTS_PIN) != LOW )
      {

      }
      clear_to_send = true;
    }
  }
  
  if( clear_to_send )
  {
    if((state_counter%2) == 0)
    {
      basicpower();
    }
    else
    if(state_counter %120 == 0)
    {
      commondata_manufacturer();
    }
    else
    if(state_counter % 120 == 0)
    {
      commondata_product();
    }
    else
    if(state_counter == 7000)
    {
      state_counter = 0;
      reset();
    }
    state_counter++;
  }
}

//receive function
void ANTrec()
{
  int sbuflength = ANTSerial.available();
  uint8_t msg = 0;
  while(sbuflength > 0)
  {
    if (msgsync == 0)
    {
      msg = ANTSerial.read();
      if (msg == 0xA4)
      {
        msgsync = 1;
        sync = 0;
      }
      sbuflength = ANTSerial.available();

    }
    else if (msgsync == 1 && sync == 0)
    {
      msglength = ANTSerial.read();
      sbuflength = ANTSerial.available();
      msglength += 3;
      msgbuf[0] = 0xA4;
      msgbuf[1] = (msglength-3);
      sync++;
    }
    else if (msgsync == 1 && sync > 0 && sync < msglength)
    {
      msgbuf[sync+1] = ANTSerial.read();
      sbuflength = ANTSerial.available();
      sync++;
    }
    else if (msgsync == 1 && sync == msglength)
    {
      msgbuf[sync+1] = ANTSerial.read();
      sbuflength = ANTSerial.available();
      msgsync = 0;
      sync = 0;
      msglength = 0;
      ANTrecproc(msgbuf, msglength);
    }
  }
}

void ANTrecproc(uint8_t ANTbuf[], uint8_t ANTlength)
{
  if (ANTbuf[2] == 0x40)
  {
    if (ANTbuf[4] == 0x01) //Transmit Event
    {
      if (ANTbuf[5] == 3)
      {
        Serial.print("transmit success, sendcount: ");
        Serial.println(sendcount);
        Serial.print("time:");
        Serial.println(millis());
        basicpower();
        if (sendcount % 4 == 0)
        {
          //basicpower();
          cranktorque();
          Serial.print("Crank Torque: ");
          Serial.println(ANT_Torque);
        }
        if (sendcount % 120 == 0)
        {
          basicpower();
          commondata_manufacturer();
          commondata_product();
          sendcount = 0;
          Serial.println("manufacturer & product");
        }
        sendcount++;
      }
    }
    if (ANTbuf[5] == 0x28) //invalid message
    {
      Serial.println("invalid message");
    }
    else if (ANTbuf[4] == 0x46 ) //Set Network Key
    {
      if (ANTbuf[5] ==0)
      {
        Serial.println("Set Netowork Key: No Error");
      }
    }
    else if (ANTbuf[4] == 0x42 ) // Assign Channel Assigned
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Assign Channel: No Error");
      }
    }
    else if (ANTbuf[4] == 0x51 ) // Set Channel ID
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Set Channel ID: No Error");
      }      
    }
    else if (ANTbuf[4] == 0x45 ) // Set Frequency
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Set Frequency: No Error");
      }      
    }
    else if (ANTbuf[4] == 0x43 ) // Set Period
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Set Period: No Error");
      }      
    }
    else if (ANTbuf[4] == 0x47 ) //Transmit Power
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Transmit Power: No Error");
      }      
    }
    else if (ANTbuf[4] == 0x44 ) //Set Timeout
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Set Timeout: No Error");
      }      
    }
    else if (ANTbuf[4] == 0x4B ) //Open Channel
    {
      if (ANTbuf[5] == 0)
      {
        Serial.println("Open Channel: No Error");
      }      
    }
  }
  if(ANTbuf[4] == 0x01){
    if(ANTbuf[5] == 0xAA){
      Serial.println("deal with request");
      Serial.println("------------------");
    }
  }
  //Serial.print("ANTRecproc RX: ");
   for(int i = 0 ; i <= int(ANTbuf[1])+2; i++) //take the buffer length byte as int and add sync and checksum
   {
     
     //Serial.print(ANTbuf[i], HEX);
     //Serial.print(" ");
   }
}

void callback()
{

}

UCHAR checkSum(UCHAR *data, int length)
{
   int i;
   UCHAR chksum = data[0];
   for (i = 1; i < length; i++)
      chksum ^= data[i];  // +1 since skip prefix sync code, we already counted it
   return chksum;
}
void reset()
{
   uint8_t buf[5];
   buf[0] = MESG_TX_SYNC; // SYNC Byte
   buf[1] = 0x01; // LENGTH Byte
   buf[2] = MESG_SYSTEM_RESET_ID; // ID Byte
   buf[3] = 0x00; // Data Byte N (N=LENGTH)
   buf[4] = checkSum(buf,4);
   ANTsend(buf,5);
}
    
    
void SetANTPlusNetwork()
{
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_NETWORK_KEY_ID; // ID Byte
    buf[3] = 0x00; // Data Byte N (
    buf[4] = 0x00; // Data Byte N (N=LENGTH)
    buf[5] = 0x00; // Data Byte N (N=LENGTH)
    buf[6] = 0x00; // Data Byte N (N=LENGTH)
    buf[7] = 0x00; // Data Byte N (N=LENGTH)
    buf[8] = 0x00; // Data Byte N (N=LENGTH)
    buf[9] = 0x00; // Data Byte N (N=LENGTH)
    buf[10] = 0x00; // Data Byte N (N=LENGTH)
    buf[11] = 0x00; // Data Byte N (N=LENGTH)
    buf[12] = checkSum(buf, 12);
    ANTsend(buf,13);
}
void SetPublicNetwork()
{
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_NETWORK_KEY_ID; // ID Byte
    buf[3] = 0x00; // Data Byte N (
    buf[4] = 0x00; // Data Byte N (N=LENGTH)
    buf[5] = 0x00; // Data Byte N (N=LENGTH)
    buf[6] = 0x00; // Data Byte N (N=LENGTH)
    buf[7] = 0x00; // Data Byte N (N=LENGTH)
    buf[8] = 0x00; // Data Byte N (N=LENGTH)
    buf[9] = 0x00; // Data Byte N (N=LENGTH)
    buf[10] = 0x00; // Data Byte N (N=LENGTH)
    buf[11] = 0x00; // Data Byte N (N=LENGTH)
    buf[12] = checkSum(buf, 12);
    ANTsend(buf,13);
}
void assignch()
{
   uint8_t buf[7];
   buf[0] = MESG_TX_SYNC; // SYNC Byte
   buf[1] = 0x03; // LENGTH Byte
   buf[2] = MESG_ASSIGN_CHANNEL_ID; // ID Byte
   buf[3] = 0x00; // Channel Number
   buf[4] = 0x10; // Channel Type
   buf[5] = 0x00; // Network ID
   buf[6] = checkSum(buf,6);
   ANTsend(buf,7);
}

void SetChID() // channel parameters
{
    uint8_t buf[9];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x05; // LENGTH Byte
    buf[2] = MESG_CHANNEL_ID_ID; // Assign Channel ID 0x51
    buf[3] = 0x00; // channel number
    buf[4] = 0x00; // Device number MSB
    buf[5] = 0x01; // Device number LSB
    buf[6] = 0x0B; //Device type ID 0x0B for basic power meter
    buf[7] = 0x05; //Transmission type -CHANGED
    buf[8] = checkSum(buf, 8);
    ANTsend(buf,9);
}

void ANTsend(uint8_t buf[], int length){
   //Serial.print("ANTsend TX: ");
   for(int i = 0 ; i <= length ; i++)
   {
     //Serial.print(buf[i], HEX);
     //Serial.print(" ");
     ANTSerial.write(buf[i]);
   }
   //Serial.println("");
}
void SetFreq()
{
    uint8_t buf[6];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x02; // LENGTH Byte
    buf[2] = MESG_CHANNEL_RADIO_FREQ_ID; // Set Channel RF Freq 0x45
    buf[3] = 0x00; // Channel number
    buf[4] = 0x39; // Frequency 0x42 = 66 dec = default 2466Mhz, 39 = 57 = 2457 Mhz
    buf[5] = checkSum(buf, 5);
    ANTsend(buf,6);
}

void SetPeriod()
{
    uint8_t buf[7];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x03; // LENGTH Byte
    buf[2] = MESG_CHANNEL_MESG_PERIOD_ID; // Set channel period 0x43
    buf[3] = 0x00; // Channel number
    buf[4] = 0xF6; // Messaging Period byte1
    buf[5] = 0x1F; // Messaging period byte2 //8182 (4Hz)
    buf[6] = checkSum(buf, 6);
    ANTsend(buf,7);
}

void SetPower()
{
    uint8_t buf[6];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x02; // LENGTH Byte
    buf[2] = MESG_RADIO_TX_POWER_ID; // Set Tx Power 0x47
    buf[3] = 0x00; // Channel Number
    buf[4] = 0x03; // Tx power
    buf[5] = checkSum(buf, 5);
    ANTsend(buf,6);
}

void SetTimeout()
{
    uint8_t buf[6];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x02; // LENGTH Byte
    buf[2] = MESG_CHANNEL_SEARCH_TIMEOUT_ID; // Set Channel Search Timeout 0x44
    buf[3] = 0x00; // Channel number
    buf[4] = 0x1E; // Set timeout
    buf[5] = checkSum(buf, 5);
    ANTsend(buf,6);
}

void OpenChannel()
{
    uint8_t buf[5];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x01; // LENGTH Byte
    buf[2] = MESG_OPEN_CHANNEL_ID; // ID Byte 0x4B
    buf[3] = 0x00;
    buf[4] = checkSum(buf, 4);
    ANTsend(buf,5);
}

void initiate()
{
    Serial.println("Enter ANT initiate: ");
    SetPublicNetwork();
    Serial.println("Network set");
    delay(100);
    assignch();
    Serial.println("assigned channel");
    delay(100);
    SetChID();
    Serial.println("channel ID set");
    delay(100);
    SetFreq();
    Serial.println("Freq set");
    delay(100);
    SetPeriod();
    Serial.println("channel timing period set");
    delay(100);
    SetPower();
    Serial.println("transmission power set");
    delay(100);
    SetTimeout();
    Serial.println("channel timeout set");
    delay(100);
    OpenChannel();
    Serial.println("opened channel");
    delay(100);
}

void basicpower()
{
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
    buf[3] = 0x00;  // Channel number
    buf[4] = 0x10; // Basic power page identifier
    buf[5] = byte(event_count_basic); // Event count
    buf[6] = 0xB2; // Power differential (pedal power) 0xFF when not used
    buf[7] = byte(RPM); // Instantaneous Cadence (0-254 rpm)
    buf[8] = byte(ANT_power_total & 0xFF); // Accumulated power LSB
    buf[9] = byte((ANT_power_total >> 8) & 0xFF); // Accumulated power MSB
    buf[10] = 0x00;//byte(HX711_instPower & 0xFF); // Instantaneous power LSB
    buf[11] = 0x00;//byte((HX711_instPower >> 8) & 0xFF); // Instantaneous power MSB
    buf[12] = checkSum(buf, 12);
    ANTsend(buf, 13);
    event_count_basic++;
    if(event_count_basic > 254) event_count_basic = 0;
}

void cranktorque()
{
    //read_hx711();
    //Serial.println(HX711_instPower);
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
    buf[3] = 0x00;  // Channel number
    buf[4] = 0x12; // crank torque page identifier
    buf[5] = byte(event_count_torque); // Event count
    buf[6] = byte(ANT_ticks); // crank ticks
    buf[7] = byte(RPM); // instantaneous cadence
    buf[8] = byte(ANT_Period & 0xFF); // timestamp MSB
    buf[9] = byte((ANT_Period >> 8) & 0xFF); // timestamp LSB 1/2000s
    buf[10] = byte(ANT_Torque & 0xFF); // torque ticks stamp MSB
    buf[11] = byte((ANT_Torque >> 8) & 0xFF); // torque ticks LSB
    buf[12] = checkSum(buf, 12);
    ANTsend(buf, 13);
    event_count_torque++;
    if(event_count_torque > 254) event_count_torque = 0;
}
void commondata_manufacturer()
{
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
    buf[3] = 0x00;  // Channel number
    buf[4] = 0x50; // manufacturer page identifier
    buf[5] = 0xFF; // Event count
    buf[6] = 0xFF; // Power differential
    buf[7] = 0x01; // HW revision
    buf[8] = 0xFF; // Manufacturer ID LSB
    buf[9] = 0x00; // Manufacturer ID MSB
    buf[10] = 0x01; // Model Number LSB
    buf[11] = 0x00; // Model Number MSB
    buf[12] = checkSum(buf, 12);
    ANTsend(buf, 13);
}
void commondata_product()
{
    uint8_t buf[13];
    buf[0] = MESG_TX_SYNC; // SYNC Byte
    buf[1] = 0x09; // LENGTH Byte
    buf[2] = MESG_BROADCAST_DATA_ID; // 0x4E
    buf[3] = 0x00;  // Channel number
    buf[4] = 0x51; // product page identifier
    buf[5] = 0xFF; // RESERVED 0xFF
    buf[6] = 0xFF; // Supplemental SW Revision (Invalid = 0xFF)
    buf[7] = 0x01; // Main SW Revision defined by manufacturer
    buf[8] = 0xFF; //serial No.
    buf[9] = 0xFF; // serial No.
    buf[10] = 0xFF; // serial No.
    buf[11] = 0xFF; // serial No.
    buf[12] = checkSum(buf, 12);
    ANTsend(buf, 13);
}
