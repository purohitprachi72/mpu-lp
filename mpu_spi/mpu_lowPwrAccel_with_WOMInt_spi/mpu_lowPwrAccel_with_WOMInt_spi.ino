#include <MPU9250_WE.h>
#include <Adafruit_SPIFlash.h>  // Need to be deleted /Documents/Arduino/libraries/SdFat

const int csPin = 3;  // Chip Select Pin
bool useSPI = true;    // SPI use flag

const int intPin = 1;
volatile bool motion = false;

Adafruit_FlashTransport_QSPI flashTransport;
/* There is only one construictor for SPI: */
MPU9250_WE myMPU9250 = MPU9250_WE(&SPI, csPin, useSPI);

void setup() {

  // pinMode(csPin, OUTPUT);
  Serial.begin(115200);
  while(!Serial) delay(10);
  // Power configuration for nRF52840
  NRF_POWER->DCDCEN = 1;
  delay(1000);

  // Onboard Flash enter to Deep Power-Down Mode
  flashTransport.begin();
  flashTransport.runCommand(0xB9);  // enter deep power-down mode
  delayMicroseconds(5);             // tDP=3uS
  flashTransport.end();

  if(!myMPU9250.init()){
    Serial.println("MPU9250 does not respond");
  }
  else{
    Serial.println("MPU9250 is connected");
  }

  /* Choose the SPI clock speed, default is 8 MHz 
     This function must be used only after init(), not before */
  myMPU9250.setSPIClockSpeed(4000000);

  // Serial.println("Position you MPU9250 flat and don't move it - calibrating...");
  delay(1000);
  myMPU9250.autoOffsets();
  Serial.println("Done!");
  
//   myMPU9250.enableCycle(false);
  // myMPU9250.sleep(true);

  //low pwr accelerometer configurations
  myMPU9250.setSampleRateDivider(5);
  myMPU9250.setAccRange(MPU9250_ACC_RANGE_2G);
  myMPU9250.enableAccDLPF(true);
  // *   DLPF     Bandwidth [Hz]      Delay [ms]    Output rate [kHz]
  // *     6             5              66.96           1
  myMPU9250.setAccDLPF(MPU9250_DLPF_6);
  myMPU9250.setLowPowerAccDataRate(MPU9250_LP_ACC_ODR_0_98);
  //wom interrupt
  myMPU9250.setIntPinPolarity(MPU9250_ACT_HIGH);
  myMPU9250.enableIntLatch(true);
  myMPU9250.enableClearIntByAnyRead(false); 
  myMPU9250.enableInterrupt(MPU9250_WOM_INT); 
  myMPU9250.setWakeOnMotionThreshold(128);
  /*  Enable/disable wake on motion (WOM) and  WOM mode:
   *  MPU9250_WOM_DISABLE
   *  MPU9250_WOM_ENABLE
   *  ***
   *  MPU9250_WOM_COMP_DISABLE   // reference is the starting value
   *  MPU9250_WOM_COMP_ENABLE    // reference is the last value
   */
  myMPU9250.enableWakeOnMotion(MPU9250_WOM_ENABLE, MPU9250_WOM_COMP_DISABLE);
  myMPU9250.enableAccAxes(MPU9250_ENABLE_XYZ);
  //disabling gyro
  myMPU9250.enableGyrStandby(true);
  myMPU9250.enableGyrAxes(MPU9250_ENABLE_000);
  //disable magnetometer
  myMPU9250.setMagOpMode(AK8963_PWR_DOWN);
  attachInterrupt(digitalPinToInterrupt(intPin), motionISR, RISING);
  Serial.println("Turn your MPU9250 and see what happens...");
  // digitalWrite(csPin, LOW);
  delay(200);
}

void loop() {
 xyzFloat gValue;
  if(motion){
    byte source = myMPU9250.readAndClearInterrupts();
    Serial.println("Interrupt!");
    if(myMPU9250.checkInterrupt(source, MPU9250_WOM_INT)){
      Serial.println("Interrupt Type: Motion");
      Serial.println("Acceleration in g (x,y,z):");
      gValue = myMPU9250.getGValues();    
      Serial.print(gValue.x);
      Serial.print("   ");
      Serial.print(gValue.y);
      Serial.print("   ");
      Serial.println(gValue.z);
      delay(1000);
    
    motion = false;
    // if additional interrupts have occured in the meantime:
    myMPU9250.readAndClearInterrupts(); 
  }
delay(100);
}

void motionISR() {
  motion = true;
}
