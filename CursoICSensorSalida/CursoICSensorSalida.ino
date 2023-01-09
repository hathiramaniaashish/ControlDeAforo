/* ---------------------------------------------------------------------
 *  Ejemplo MKR1310_LoRa_SendReceive_Binary
 *  Práctica 3
 *  Asignatura (GII-IoT)
 *  
 *  Basado en el ejemplo MKR1310_LoRa_SendReceive_WithCallbacks,
 *  muestra cómo es posible comunicar los parámetros de 
 *  configuración del transceiver entre nodos LoRa en
 *  formato binario *  
 *  
 *  Este ejemplo requiere de una versión modificada
 *  de la librería Arduino LoRa (descargable desde 
 *  CV de la asignatura.
 *  
 *  También usa la librería Arduino_BQ24195 
 *  https://github.com/arduino-libraries/Arduino_BQ24195
 * ---------------------------------------------------------------------
 */

#include <SPI.h>             
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include <Wire.h> // Arduino's I2C library
#define BUZZER_PIN 5
const int sensorDelay = 100;
volatile int sensorTimer = millis();
const int buzzerDelay = 250;
const int alarmDelay = 100;
volatile int buzzerTimer = 0;
volatile int height = 0;
volatile bool alarm = 0;
#define doorHeight 192
const int SensorLimit = doorHeight/2;
// Para Sensor 02727
//#define SRF02_I2C_ADDRESS byte((0xF0)>>1)

#define SRF02_I2C_ADDRESS byte((0xEA)>>1)
#define SRF02_I2C_INIT_DELAY 100 // in milliseconds
#define SRF02_RANGING_DELAY 70 // milliseconds

// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define SOFTWARE_REVISION byte(0x00)
#define RANGE_HIGH_BYTE byte(2)
#define RANGE_LOW_BYTE byte(3)
#define AUTOTUNE_MINIMUM_HIGH_BYTE byte(4)
#define AUTOTUNE_MINIMUM_LOW_BYTE byte(5)

// SRF02's command codes
#define REAL_RANGING_MODE_CMS       byte(81)

inline void write_command(byte address,byte command)
{ 
  Wire.beginTransmission(address);
  Wire.write(COMMAND_REGISTER); 
  Wire.write(command); 
  Wire.endTransmission();
}

byte read_register(byte address,byte the_register)
{
  Wire.beginTransmission(address);
  Wire.write(the_register);
  Wire.endTransmission();
  
  // getting sure the SRF02 is not busy
  Wire.requestFrom(address,byte(1));
  while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
} 



volatile bool ack = false;
volatile bool detected = false;
volatile bool samePerson = false;

// NOTA: Ajustar estas variables 
// Direcciones -> 0x40(central) 0x41(entrada) 0x42(salida) 0x43(palabra de sincronizacion)
const uint8_t localAddress = 0x42;     // Dirección de este dispositivo
uint8_t destination = 0x40;            // Dirección de destino, 0xFF es la dirección de broadcast

volatile bool txDoneFlag = true;       // Flag para indicar cuando ha finalizado una transmisión
volatile bool transmitting = false;

// Estructura para almacenar la configuración de la radio
typedef struct {
  uint8_t bandwidth_index;
  uint8_t spreadingFactor;
  uint8_t codingRate;
  uint8_t txPower; 
} LoRaConfig_t;

double bandwidth_kHz[10] = {7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3,
                            41.7E3, 62.5E3, 125E3, 250E3, 500E3 };

LoRaConfig_t thisNodeConf   = { 9, 7, 5, 2};
LoRaConfig_t remoteNodeConf = { 0,  0, 0, 0};
int remoteRSSI = 0;
float remoteSNR = 0;

// --------------------------------------------------------------------
// Setup function
// --------------------------------------------------------------------
void setup() 
{
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  Serial.begin(115200);  
  while (!Serial); 

  Serial.println("LoRa Duplex with TxDone and Receive callbacks");
  Serial.println("Using binary packets");
  
  // Es posible indicar los pines para CS, reset e IRQ pins (opcional)
  // LoRa.setPins(csPin, resetPin, irqPin);// set CS, reset, IRQ pin

  
  if (!init_PMIC()) {
    Serial.println("Initilization of BQ24195L failed!");
  }
  else {
    Serial.println("Initilization of BQ24195L succeeded!");
  }

  if (!LoRa.begin(868E6)) {      // Initicializa LoRa a 868 MHz
    Serial.println("LoRa init failed. Check your connections.");
    while (true);                
  }

  // Configuramos algunos parámetros de la radio
  LoRa.setSignalBandwidth(long(bandwidth_kHz[thisNodeConf.bandwidth_index])); 
                                  // 7.8E3, 10.4E3, 15.6E3, 20.8E3, 31.25E3
                                  // 41.7E3, 62.5E3, 125E3, 250E3, 500E3 
                                  // Multiplicar por dos el ancho de banda
                                  // supone dividir a la mitad el tiempo de Tx
                                  
  LoRa.setSpreadingFactor(thisNodeConf.spreadingFactor);     
                                  // [6, 12] Aumentar el spreading factor incrementa 
                                  // de forma significativa el tiempo de Tx
                                  // SPF = 6 es un valor especial
                                  // Ver tabla 12 del manual del SEMTECH SX1276
  
  LoRa.setCodingRate4(thisNodeConf.codingRate);         
                                  // [5, 8] 5 da un tiempo de Tx menor
                                  
  LoRa.setTxPower(thisNodeConf.txPower, PA_OUTPUT_PA_BOOST_PIN); 
                                  // Rango [2, 20] en dBm
                                  // Importante seleccionar un valor bajo para pruebas
                                  // a corta distancia y evitar saturar al receptor
  LoRa.setSyncWord(0x43);         // Palabra de sincronización privada por defecto para SX127X 
                                  // Usaremos la palabra de sincronización para crear diferentes
                                  // redes privadas por equipos
  LoRa.setPreambleLength(8);      // Número de símbolos a usar como preámbulo

  
  // Indicamos el callback para cuando se reciba un paquete
  LoRa.onReceive(onReceive);
  
  // Activamos el callback que nos indicará cuando ha finalizado la 
  // transmisión de un mensaje
  LoRa.onTxDone(TxFinished);

  // Nótese que la recepción está activada a partir de este punto
  LoRa.receive();

  Serial.println("LoRa init succeeded.\n");

  Serial.println("initializing Wire interface ...");
  Wire.begin();
  delay(SRF02_I2C_INIT_DELAY);  
   
  byte software_revision=read_register(SRF02_I2C_ADDRESS,SOFTWARE_REVISION);
  Serial.print("SFR02 ultrasonic range finder in address 0x");
  Serial.print(SRF02_I2C_ADDRESS,HEX); Serial.print("(0x");
  Serial.print(software_revision,HEX); Serial.println(")");
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() {

  if (!alarm) {

    if (buzzerTimer <= millis() && digitalRead(BUZZER_PIN) == HIGH) {
      digitalWrite(BUZZER_PIN, LOW);
    }

    write_command(SRF02_I2C_ADDRESS, REAL_RANGING_MODE_CMS);
    delay(SRF02_RANGING_DELAY);

    byte high_byte_range = read_register(SRF02_I2C_ADDRESS, RANGE_HIGH_BYTE);
    byte low_byte_range = read_register(SRF02_I2C_ADDRESS, RANGE_LOW_BYTE);
    byte high_min = read_register(SRF02_I2C_ADDRESS, AUTOTUNE_MINIMUM_HIGH_BYTE);
    byte low_min = read_register(SRF02_I2C_ADDRESS, AUTOTUNE_MINIMUM_LOW_BYTE);

    int distance = int((high_byte_range << 8) | low_byte_range);
    if (distance <= SensorLimit) {
      if (!samePerson) {
        detected = true;
        samePerson = true;
        height = doorHeight - distance;
        Serial.println(height);
        digitalWrite(BUZZER_PIN, HIGH);
        buzzerTimer = millis() + buzzerDelay;
      }
    } else {
      samePerson = false;
    }
    sensorTimer = millis() + sensorDelay;


    static uint32_t lastSendTime_ms = 0;
    static uint16_t msgCount = 0;
    static uint32_t tx_begin_ms = 0;

    if (detected && !transmitting) {
      detected = false;

      uint8_t payload[1];
      uint8_t payloadLength = 1;

      payload[0] = height;

      transmitting = true;
      txDoneFlag = false;
      tx_begin_ms = millis();

      sendMessage(payload, payloadLength, msgCount);
      Serial.print("Sending packet ");
      Serial.print(msgCount++);
      Serial.print(": ");
      printBinaryPayload(payload, payloadLength);
    }

    if (transmitting && txDoneFlag) {
      uint32_t TxTime_ms = millis() - tx_begin_ms;
      Serial.print("----> TX completed in ");
      Serial.print(TxTime_ms);
      Serial.println(" msecs");

      // Ajustamos txInterval_ms para respetar un duty cycle del 1%
      uint32_t lapse_ms = tx_begin_ms - lastSendTime_ms;
      lastSendTime_ms = tx_begin_ms;
      float duty_cycle = (100.0f * TxTime_ms) / lapse_ms;

      Serial.print("Duty cycle: ");
      Serial.print(duty_cycle, 1);
      Serial.println(" %\n");

      transmitting = false;
      // Reactivamos la recepción de mensajes, que se desactiva
      // en segundo plano mientras se transmite
      LoRa.receive();
    }

  }

  else{
    if (buzzerTimer <= millis() && digitalRead(BUZZER_PIN) == HIGH){
      digitalWrite(BUZZER_PIN, LOW);
      buzzerTimer = millis() + alarmDelay;
    }
    else if (buzzerTimer <= millis() && digitalRead(BUZZER_PIN) == LOW){
      digitalWrite(BUZZER_PIN, HIGH);
      buzzerTimer = millis() + alarmDelay;
    }
  }
}

// --------------------------------------------------------------------
// Sending message function
// --------------------------------------------------------------------
void sendMessage(uint8_t* payload, uint8_t payloadLength, uint16_t msgCount) 
{
  while(!LoRa.beginPacket()) {            // Comenzamos el empaquetado del mensaje
    delay(10);                            // 
  }
  LoRa.write(destination);                // Añadimos el ID del destinatario
  LoRa.write(localAddress);               // Añadimos el ID del remitente
  LoRa.write((uint8_t)(msgCount >> 7));   // Añadimos el Id del mensaje (MSB primero)
  LoRa.write((uint8_t)(msgCount & 0xFF)); 
  LoRa.write(payloadLength);              // Añadimos la longitud en bytes del mensaje
  LoRa.write(payload, (size_t)payloadLength); // Añadimos el mensaje/payload 
  LoRa.endPacket(true);                   // Finalizamos el paquete, pero no esperamos a
                                          // finalice su transmisión
}

// --------------------------------------------------------------------
// Receiving message function
// --------------------------------------------------------------------
void onReceive(int packetSize) 
{
  if (transmitting && !txDoneFlag) txDoneFlag = true;
  
  if (packetSize == 0) return;          // Si no hay mensajes, retornamos

  // Leemos los primeros bytes del mensaje
  uint8_t buffer[10];                   // Buffer para almacenar el mensaje
  int recipient = LoRa.read();          // Dirección del destinatario
  uint8_t sender = LoRa.read();         // Dirección del remitente
                                        // msg ID (High Byte first)
  uint16_t incomingMsgId = ((uint16_t)LoRa.read() << 7) | 
                            (uint16_t)LoRa.read();
  
  uint8_t incomingLength = LoRa.read(); // Longitud en bytes del mensaje
  
  uint8_t receivedBytes = 0;            // Leemos el mensaje byte a byte
  while (LoRa.available() && (receivedBytes < uint8_t(sizeof(buffer)-1))) {            
    buffer[receivedBytes++] = (char)LoRa.read();
  }
  
  if (incomingLength != receivedBytes) {// Verificamos la longitud del mensaje
    // Serial.print("Receiving error: declared message length " + String(incomingLength));
    // Serial.println(" does not match length " + String(receivedBytes));
    return;                             
  }

  // Verificamos si se trata de un mensaje en broadcast o es un mensaje
  // dirigido específicamente a este dispositivo.
  // Nótese que este mecanismo es complementario al uso de la misma
  // SyncWord y solo tiene sentido si hay más de dos receptores activos
  // compartiendo la misma palabra de sincronización
  if ((recipient & localAddress) != localAddress ) {
    // Serial.println("Receiving error: This message is not for me.");
    return;
  }

  if (sender != localAddress && sender == 0x40){
    alarm = buffer[0] + 0;
  }
}

void TxFinished()
{
  txDoneFlag = true;
}

void printBinaryPayload(uint8_t * payload, uint8_t payloadLength)
{
  for (int i = 0; i < payloadLength; i++) {
    Serial.print((payload[i] & 0xF0) >> 4, HEX);
    Serial.print(payload[i] & 0x0F, HEX);
    Serial.print(" ");
  }
}
