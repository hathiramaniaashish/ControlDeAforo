#include <SPI.h>      
#include <LoRa.h>
#include <Arduino_PMIC.h>
#include <Wire.h> // Arduino's I2C library
#include <RTCZero.h>

RTCZero rtc;

#define LCD05_I2C_ADDRESS byte((0xC6)>>1)
#define LCD05_I2C_INIT_DELAY 100 // in milliseconds

// LCD05's command related definitions
#define COMMAND_REGISTER byte(0x00)
#define FIFO_AVAILABLE_LENGTH_REGISTER byte(0x00)
#define LCD_STYLE_16X2 byte(5)

// LCD05's command codes
#define CURSOR_HOME             byte(1)
#define SET_CURSOR              byte(2) // specify position with a byte in the interval 0-32/80
#define SET_CURSOR_COORDS       byte(3) // specify position with two bytes, line and column
#define HIDE_CURSOR             byte(4)
#define SHOW_UNDERLINE_CURSOR   byte(5)
#define SHOW_BLINKING_CURSOR    byte(6)
#define BACKSPACE               byte(8)
#define HORIZONTAL_TAB          byte(9) // advances cursor a tab space
#define SMART_LINE_FEED         byte(10) // moves the cursor to the next line in the same column
#define VERTICAL_TAB            byte(11) // moves the cursor to the previous line in the same column
#define CLEAR_SCREEN            byte(12)
#define CARRIAGE_RETURN         byte(13)
#define CLEAR_COLUMN            byte(17)
#define TAB_SET                 byte(18) // specify tab size with a byte in the interval 1-10
#define BACKLIGHT_ON            byte(19)
#define BACKLIGHT_OFF           byte(20) // this is the default
#define DISABLE_STARTUP_MESSAGE byte(21)
#define ENABLE_STARTUP_MESSAGE  byte(22)
#define SAVE_AS_STARTUP_SCREEN  byte(23)
#define SET_DISPLAY_TYPE        byte(24) // followed by the type, which is byte 5 for a 16x2 LCD style
#define CHANGE_ADDRESS          byte(25) // see LCD05 specification
#define CUSTOM_CHAR_GENERATOR   byte(27) // see LCD05 specification
#define DOUBLE_KEYPAD_SCAN_RATE byte(28)
#define NORMAL_KEYPAD_SCAN_RATE byte(29)
#define CONTRAST_SET            byte(30) // specify contrast level with a byte in the interval 0-255
#define BRIGHTNESS_SET          byte(31) // specify brightness level with a byte in the interval 0-255

inline void write_command(byte command)
{ Wire.write(COMMAND_REGISTER); Wire.write(command); }

void set_display_type(byte address, byte type)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SET_DISPLAY_TYPE);
  Wire.write(type);
  Wire.endTransmission();
}

void clear_screen(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(CLEAR_SCREEN);
  Wire.endTransmission();
}

void cursor_home(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(CURSOR_HOME);
  Wire.endTransmission();
}

void set_cursor(byte address, byte pos)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SET_CURSOR);
  Wire.write(pos);
  Wire.endTransmission();
}

void set_cursor_coords(byte address, byte line, byte column)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SET_CURSOR_COORDS);
  Wire.write(line);
  Wire.write(column);
  Wire.endTransmission();
}

void show_blinking_cursor(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(SHOW_BLINKING_CURSOR);
  Wire.endTransmission();
}

void backlight_on(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(BACKLIGHT_ON);
  Wire.endTransmission();
}

void backlight_off(byte address)
{
  Wire.beginTransmission(address); // start communication with LCD 05
  write_command(BACKLIGHT_OFF);
  Wire.endTransmission();
}

bool ascii_chars(byte address, byte* bytes, int length)
{
  if(length<=0) return false;
  Wire.beginTransmission(address); // start communication with LCD 05
  Wire.write(COMMAND_REGISTER);
  for(int i=0; i<length; i++, bytes++) Wire.write(*bytes);
  Wire.endTransmission();
  return true;
}

byte read_fifo_length(byte address)
{
  Wire.beginTransmission(address);
  Wire.write(FIFO_AVAILABLE_LENGTH_REGISTER);                           // Call the register for start of ranging data
  Wire.endTransmission();
  
  Wire.requestFrom(address,byte(1)); // start communication with LCD 05, request one byte
  while(!Wire.available()) { /* do nothing */ }
  return Wire.read();
}

const char* the_msg = "Personas:";
const char* the_msg2 = "Altura:";
const char* the_msg3 = "INTRUSO DETECTADO";

volatile bool alarm = 0;
volatile bool sentAlarm = 0;
volatile bool turnOff = 0;

volatile int personas = 0;
volatile int altura = 0;

// NOTA: Ajustar estas variables 
const uint8_t localAddress = 0x40;     // Dirección de este dispositivo
uint8_t destination = 0xFF;            // Dirección de destino, 0xFF es la dirección de broadcast

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

  Serial.begin(115200);  
  while (!Serial); 

  Serial.println("Initializing RTCZero ...");
  rtc.begin();

  if (!setDateTime(__DATE__, __TIME__)) {
    SerialUSB.println("setDateTime() failed!\nExiting ...");
    while (1) { ; }
  }

  Serial.println("initializing Wire interface ...");
  Wire.begin();
  delay(LCD05_I2C_INIT_DELAY);  
  
  Serial.print("initializing LCD05 display in address 0x");
  Serial.print(LCD05_I2C_ADDRESS,HEX); Serial.println(" ...");
  
  set_display_type(LCD05_I2C_ADDRESS,LCD_STYLE_16X2);
  clear_screen(LCD05_I2C_ADDRESS);
  cursor_home(LCD05_I2C_ADDRESS);
  backlight_on(LCD05_I2C_ADDRESS);

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
  
}

// --------------------------------------------------------------------
// Loop function
// --------------------------------------------------------------------
void loop() {

  if (alarm) {
    ascii_chars(
      LCD05_I2C_ADDRESS,
      reinterpret_cast<byte*>(const_cast<char*>(the_msg3)),
      strlen(the_msg3));
    delay(500);
    clear_screen(LCD05_I2C_ADDRESS);
  }

  else {
    ascii_chars(
      LCD05_I2C_ADDRESS,
      reinterpret_cast<byte*>(const_cast<char*>(the_msg)),
      strlen(the_msg));

    char cadena[7];
    sprintf(cadena, "%d", personas);

    ascii_chars(
      LCD05_I2C_ADDRESS,
      reinterpret_cast<byte*>(const_cast<char*>(cadena)),
      strlen(cadena));

    set_cursor(LCD05_I2C_ADDRESS, 17);

    ascii_chars(
      LCD05_I2C_ADDRESS,
      reinterpret_cast<byte*>(const_cast<char*>(the_msg2)),
      strlen(the_msg2));

    char cadena2[7];
    sprintf(cadena2, "%d", altura);

    ascii_chars(
      LCD05_I2C_ADDRESS,
      reinterpret_cast<byte*>(const_cast<char*>(cadena2)),
      strlen(cadena2));

    delay(500);

    clear_screen(LCD05_I2C_ADDRESS);
  }

  static uint32_t lastSendTime_ms = 0;
  static uint16_t msgCount = 0;
  static uint32_t tx_begin_ms = 0;

  if (alarm && !sentAlarm) {

    if (!transmitting) {
      uint8_t payload[1];
      uint8_t payloadLength = 1;

      payload[0] = alarm;

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
      sentAlarm = 1;
      // Reactivamos la recepción de mensajes, que se desactiva
      // en segundo plano mientras se transmite
      LoRa.receive();
    }
  }

  if (!alarm && Serial.availale()){
    delay(200);
    clearBuffer();
  }

  if (alarm && sentAlarm && Serial.available()){
    delay(200);
    clearBuffer();
    turnOff = 1;
  }

  if (alarm && sentAlarm && turnOff) {

    if (!transmitting) {
      uint8_t payload[1];
      uint8_t payloadLength = 1;

      payload[0] = 0;

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
      sentAlarm = 0;
      alarm = 0;
      turnOff = 0;
      // Reactivamos la recepción de mensajes, que se desactiva
      // en segundo plano mientras se transmite
      LoRa.receive();
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

  if (sender != localAddress && sender == 0x41){
    personas = personas + 1;
    altura = buffer[0] + 0;
  }

  else if (sender != localAddress && sender == 0x42){
    personas = personas - 1;
    altura = buffer[0] + 0;
    if (personas < 0){
      personas = 0;
      altura = 0;
    }
  }

  if (sender != localAddress && (sender == 0x41 || sender == 0x42) && (rtc.getHours() >= 12 || rtc.getHours() <= 4)){
    alarm = 1;
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

bool setDateTime(const char* date_str, const char* time_str) {
  char month_str[4];
  char months[12][4] = { "Jan", "Feb", "Mar", "Apr", "May", "Jun", "Jul", "Aug",
                         "Sep", "Oct", "Nov", "Dec" };
  uint16_t i, mday, month, hour, min, sec, year;
  if (sscanf(date_str, "%3s %hu %hu", month_str, &mday, &year) != 3) return false;
  if (sscanf(time_str, "%hu:%hu:%hu", &hour, &min, &sec) != 3) return false;
  for (i = 0; i < 12; i++) {
    if (!strncmp(month_str, months[i], 3)) {
      month = i + 1;
      break;
    }
  }
  if (i == 12) return false;
  rtc.setTime((uint8_t)hour, (uint8_t)min, (uint8_t)sec);
  rtc.setDate((uint8_t)mday, (uint8_t)month, (uint8_t)(year - 2000));
  return true;
}

void clearBuffer(){
  while(Serial.available() > 0){
    Serial.read();
  }
}
