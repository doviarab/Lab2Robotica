//youtube.com/TARUNKUMARDAHAKE
//facebook.com/TARUNKUMARDAHAKE

#include <PID_v1.h>
#include <LMotorController.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  #include "Wire.h"
#endif

#define MIN_ABS_SPEED 20

MPU6050 mpu;

// Inicialización de Módulo Bluetooth HC-06
const int btTx = 11;
const int btRx = 12;
SoftwareSerial bluetooth(btTx, btRx);

// Asignación pin sensores IR
const int sensorPin1 = 4; // Izquierda
const int sensorPin2 = 3; // Derecha

// MPU control/status vars
bool dmpReady = false; // set true if DMP init was successful
uint8_t mpuIntStatus; // holds actual interrupt status byte from MPU
uint8_t devStatus; // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize; // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount; // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q; // [w, x, y, z] quaternion container
VectorFloat gravity; // [x, y, z] gravity vector
float ypr[3]; // [yaw, pitch, roll] yaw/pitch/roll container and gravity vector

//PID
double originalSetpoint = 185;//Busqueda de Referencia angulo 0
double setpoint = originalSetpoint;
double movingAngleOffset = 0.1;
double input, output;

//adjust these values to fit your own design
double Kp = 20;   // entre  y 100 
double Ki = 0; // entre 0 y 200
double Kd = 1.2; // entre 0 y 2

PID pid(&input, &output, &setpoint, Kp, Ki, Kd, DIRECT);

double motorSpeedFactorLeft = 0.5;
double motorSpeedFactorRight = 0.44;
//MOTOR CONTROLLER
int ENA = 5;
int IN1 = 7;
int IN2 = 6;
int IN3 = 8;
int IN4 = 9;
int ENB = 10;
LMotorController motorController(ENA, IN1, IN2, ENB, IN3, IN4, motorSpeedFactorLeft, motorSpeedFactorRight);

bool isDataPending = true;
char btData[5]; // Para almacenar los datos recibidos desde la apk por bluetooth
/*
 * Posee la forma "x,yyy"
 * 'x' posee la información de qué instrucción se está recibiendo por BT
 * 1 = Adelante
 * 2 = Derecha
 * 3 = Izquierda
 * 4 = Atrás
 * 5 = Modificar 'P'
 * 6 = Modificar 'I'
 * 7 = Modificar 'D'
 * 8 = Modificar ángulo de balanceo
 * 'yyy' posee el dato asociado a dicha instrucción, por ejemplo, el nuevo "i" de un PID
 */

volatile bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high
void dmpDataReady()
{
  mpuInterrupt = true;
}


void setup()
{
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    Wire.begin();
    TWBR = 24; // 400kHz I2C clock (200kHz if CPU is 8MHz)
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
    Fastwire::setup(400, true);
  #endif
  Serial.begin(9600);
  bluetooth.begin(57600);
  mpu.initialize();
  
  devStatus = mpu.dmpInitialize();
  
  // supply your own gyro offsets here, scaled for min sensitivity
  mpu.setXGyroOffset(220);
  mpu.setYGyroOffset(76);
  mpu.setZGyroOffset(-85);
  mpu.setZAccelOffset(1788); // 1688 factory default for my test chip
  
  // make sure it worked (returns 0 if so)
  if (devStatus == 0)
  {
    // turn on the DMP, now that it's ready
    mpu.setDMPEnabled(true);
    
    // enable Arduino interrupt detection
    attachInterrupt(0, dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();
    
    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    dmpReady = true;
    
    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
    
    //setup PID
    pid.SetMode(AUTOMATIC);
    pid.SetSampleTime(10);
    pid.SetOutputLimits(-255, 255); 
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }
}
/*
 * Procedimiento que le permite al robot moverse hacia adelante durante medio segundo
 * Entrada: Boolean que le indica al procedimiento si la función fue llamada por el control
 * manual mediante la aplicación de android asociada.
 */
void forward(bool isManual) {
  setpoint = originalSetpoint - 0.5;
  //pid.setpoint(&newSetpoint);
  if (isManual) {
    delay(500);
    setpoint = originalSetpoint;
    //pid.setpoint(&newSetpoint);
  }
}

void backward(bool isManual) {
  setpoint = originalSetpoint + 0.5;
  //pid.setpoint(&newSetpoint);
  if (isManual) {
    delay(500);
    setpoint = originalSetpoint;
    //pid.setpoint(&newSetpoint);
  }
}

void turnLeft() {
  motorSpeedFactorRight += 0.05;
  delay(500);
  motorSpeedFactorRight -= 0.05;
}

void turnRight() {
  motorSpeedFactorLeft += 0.05;
  delay(500);
  motorSpeedFactorLeft -= 0.05;
  //motorController.move(output*5, output*10, MIN_ABS_SPEED);
}

void receiveData() {
    static boolean recvInProgress = false;
    static byte ndx = 0;
    char startMarker = '<';
    char endMarker = '>';
    char rc;
 
    while (bluetooth.available() > 0) {
        rc = bluetooth.read();

        if (recvInProgress == true) {
            if (rc != endMarker) {
                btData[ndx] = rc;
                ndx++;
                if (ndx >= 5) {
                    ndx = 4;
                }
            }
            else {
                btData[ndx] = '\0'; // terminate the string
                recvInProgress = false;
                ndx = 0;
            }
        }

        else if (rc == startMarker) {
            recvInProgress = true;
        }
    }
}

void loop()
{
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  /*if (isDataPending) {
    bluetooth.print((int)pid.GetKp());
    bluetooth.print("|");
    bluetooth.print((int)pid.GetKi());
    bluetooth.print("|");
    bluetooth.print((int)(pid.GetKd() * 100));
    bluetooth.print("|");
    bluetooth.print((int)setpoint);
    //bluetooth.print("|");
    Serial.println("Envie datos a BT");
    //printf("Envie datos a BT\n");
    isDataPending = false;
  }*/

  if(bluetooth.available()) {
    receiveData();
    int aux = (int)btData[0];// - 48; // Convertir de ASCII a int
    char stringAux[4];
    stringAux[0] = btData[1];
    stringAux[1] = btData[2];
    stringAux[2] = btData[3];
    stringAux[3] = '\0';
    while (bluetooth.available() > 0) {
      bluetooth.read();
    }
    double value = (double)atoi(stringAux);
    Serial.print("Dato recibido: ");
    Serial.println(aux, DEC);
    //printf("Dato recibido: %d\n", aux);
    switch(aux) {
      case 1:
        forward(true);
        break;
      case 2:
        motorController.turnRight(159,MIN_ABS_SPEED);
        break;
      case 3:
        motorController.turnLeft(159,MIN_ABS_SPEED);
        break;
      case 4:
        backward(true);
        break;
      case 5:
        pid.SetTunings((double)value, Ki, Kd);
        isDataPending = true;
        break;
      case 6:
        pid.SetTunings(Kp, (double)value, Kd);
        isDataPending = true;
        break;
      case 7:
        pid.SetTunings(Kp, Ki, ((double)value)/100);
        isDataPending = true;
        break;
      case 8:
        setpoint = (double)value; // Debiese funcionar, setpoint es pasado por referencia
        //pid.setpoint(&setSetpoint);
        isDataPending = true;
        break;
       case 9:
        isDataPending = true;
        break;
       default:
        Serial.println("Error, primer elemento invalido.");
    }
  }
  
  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize)
  {
    //no mpu data - performing PID calculations and output to motors 
    pid.Compute();
    motorController.move(output, MIN_ABS_SPEED);
    
  }
  
  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();
  
  // get current FIFO count
  fifoCount = mpu.getFIFOCount();
  
  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024)
  {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));
    
    // otherwise, check for DMP data ready interrupt (this should happen frequently)
  }
  else if (mpuIntStatus & 0x02)
  {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
    
    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;
    
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[1] * 180/M_PI + 180;
  }
}
