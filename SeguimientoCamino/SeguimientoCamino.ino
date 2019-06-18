
const int ENA = 10;   //MOTOR A Velocidad
const int IN1 = 9;    //MOTOR A Adelante
const int IN2 = 8;    //MOTOR A Atras
const int IN3 = 7;    //MOTOR B Adelante
const int IN4 = 6;    //MOTOR B Atras
const int ENB = 5;    //MOTOR B Velocidad


const int sensorPin1 = 3; // Izquierda
const int sensorPin2 = 4; // Centro


int velocidad = 85;


void setup() {
 // Serial.begin(9600);   //iniciar puerto serie
  pinMode(sensorPin1, INPUT);  //definir pin como entrada
  pinMode(sensorPin2, INPUT);

  pinMode(ENA, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);
}

void adelante() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}
void atras() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}
void izquierda() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}
void derecha() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENA, velocidad);
  analogWrite(ENB, velocidad);
}

void stop(){
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);
}

/* NO ME INTERESA
int ping(int pintrigger, int pinecho) {
  long duration, distanceCm;

  digitalWrite(pintrigger, LOW);  //para generar un pulso limpio ponemos a LOW 2us
  delayMicroseconds(2);
  digitalWrite(pintrigger, HIGH);  //generamos Trigger (disparo) de 2us
  delayMicroseconds(2);
  digitalWrite(pintrigger, LOW);

  duration = pulseIn(pinecho, HIGH);  //medimos el tiempo entre pulsos, en microsegundos

  distanceCm = duration * 10 / 292 / 2;  //convertimos a distancia, en cm
  return distanceCm;
} // NO ME INTERESA*/

void loop() {
  int value1 = 0;
  int value2 = 0;
  
  value1 = digitalRead(sensorPin1 );  //lectura digital de pin
  value2 = digitalRead(sensorPin2 );
  


    if ((value1 == HIGH) && (value2 == HIGH) ) {
    Serial.print("buscar linea: ");
    Serial.print("\n");
    adelante();
    
     }
    if ((value1 == LOW) && (value2 == LOW) ) {
     Serial.print("diagonal derecha arriba: ");
     Serial.print("\n");
      stop(); //aqui explota el robot...
      
      }
     if ((value1 == LOW) && (value2 == HIGH) ) {
      Serial.print("derecho: ");
      Serial.print("\n");
      derecha();
     
     }
     if ((value1 == HIGH) && (value2 == LOW) ) {
     Serial.print("arriba derecha corto: ");
     Serial.print("\n");
      izquierda();
      
     }
    
}
