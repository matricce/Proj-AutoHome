//v2.2.4.0
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <ESP8266WebServer.h>
#include <Ticker.h>
#include "DHT.h"
#include <LiquidCrystal_I2C.h>
#include <Wire.h>
#include <IRremoteESP8266.h>
#include <IRrecv.h>
#include <IRutils.h>

const String ORG = " ";
const String DEVICE_TYPE = " ";
const String DEVICE_ID = " ";
#define DEVICE_TOKEN " "
#define WIFI_SSID " "
#define WIFI_PASSWORD " "

const String CLIENT_ID = "d:" + ORG + ":" + DEVICE_TYPE + ":" + DEVICE_ID;
const String MQTT_SERVER = ORG + ".messaging.internetofthings.ibmcloud.com";

#define CLOCK_PIN 14 //(D5)conetado ao SRCLK pino 11(SHCP) - shift register clock
#define LATCH_PIN 15 //(D8)conectado ao RCLK pino 12(STCP) - output register clock
#define DATA_PIN 13 //(D7)conectado ao SER pino 14(DS) - input
#define DHT_PIN 12 //(D6)
#define RECV_PIN 2 //(D4) se declarar como D4, não funciona corretamente
#define PIN_GREEN_LED_MQTT   0  //D3 LED para mostrar conexão MQTT 
#define PIN_RED_LED_WIFI   16  //D0 LED para mostrar conexão WiFi 
#define COMANDO_TOPICO_L1 "iot-2/cmd/comandoL1/fmt/json"
#define COMANDO_TOPICO_L2 "iot-2/cmd/comandoL2/fmt/json"
#define COMANDO_TOPICO_L3 "iot-2/cmd/comandoL3/fmt/json"
#define COMANDO_TOPICO_L4 "iot-2/cmd/comandoL4/fmt/json"
#define COMANDO_TOPICO_L5 "iot-2/cmd/comandoL5/fmt/json"
#define COMANDO_TOPICO_L6 "iot-2/cmd/comandoL6/fmt/json"
#define COMANDO_TOPICO_L7 "iot-2/cmd/comandoL7/fmt/json"
#define COMANDO_TOPICO_L8 "iot-2/cmd/comandoL8/fmt/json"
#define COMANDO_TOPICO_M  "iot-2/cmd/comandoM/fmt/json"
#define EVENTO_TOPICO_DHT11 "iot-2/evt/DHT11/fmt/json"
#define EVENTO_TOPICO_IF_ON "iot-2/evt/online/fmt/json"
#define DHTTYPE DHT11
#define PUBLICAR_INTERVALO 1000 * 60 * 5 //Publica a cada 5min
#define LCD_INTERVALO 1000 * 1 //Atualiza o display a cada 1seg
#define ONLINE_INTERVALO 1000 * 4 //envia um evento para o Broker a cada 4seg

WiFiClient wifiClient;
PubSubClient client(MQTT_SERVER.c_str(), 1883, wifiClient);
DHT dht(DHT_PIN, DHTTYPE);
Ticker publicarTicker;
Ticker lcdTicker;
Ticker onlineTicker;
LiquidCrystal_I2C lcd(0x27, 16, 2);
IRrecv irrecv(RECV_PIN);
decode_results results;

bool estouOnline = true;
bool atualizaSaidaLCD = true;
bool publicaNovoEstado = true;
float armazenaTecla;
long lastReconnectAttempt = 0;
byte estadoLeds = 255;
byte comandosPB = 255;
byte comandosIR = 255;
//Array simbolo grau
byte grau[8] = {
  B00001100,
  B00010010,
  B00010010,
  B00001100,
  B00000000,
  B00000000,
  B00000000,
  B00000000,
};

void publicar() {
  publicaNovoEstado = true;
}

void setupPins(){
  pinMode(PIN_GREEN_LED_MQTT, OUTPUT);
  pinMode(PIN_RED_LED_WIFI, OUTPUT);
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  pinMode(RECV_PIN, INPUT);
  //Inicializa sensor de temperatura/umidade
  dht.begin();
  //Inicializa o receptor IR
  irrecv.enableIRIn();
}

void setupWifi(){
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Conectando ao Wi-Fi");
  int WifiTimeOut = millis();
  while (WiFi.status() != WL_CONNECTED & millis() - WifiTimeOut < 6000 ){
    Serial.print(".");
    delay(500);
  }
  if(WiFi.status() == WL_CONNECTED){
    Serial.println();
    Serial.print("Conectado: ");
    Serial.println(WiFi.localIP());
  }
  else{
    Serial.println();
    Serial.println("Tempo de conexão WiFi excedido.");
  }
}

void setupLCD() {
  Wire.begin(D2, D1);
  lcd.begin();
  //Cria o caractere customizado com o simbolo do grau
  lcd.createChar(0, grau);
}

void desligarTodosLeds() {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 255);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 255);
  digitalWrite(LATCH_PIN, HIGH);
}

void atualizarLCD() {
  atualizaSaidaLCD = true;
}

void statusOnline(){
  estouOnline = true;
}
void trocarEstadoLEDs(char source, int pino, bool valor){
  if(source == 'R'){//recebido do controle (troca feita agora)
    if (bitRead(comandosIR, pino) == 0)
      bitWrite(comandosIR, pino, 1);
    else
      bitWrite(comandosIR, pino, 0);
  }
  else if(source == 'P'){}//recebido dos botões (troca foi feita antes)
  else if(source == 'A'){}//recebido ao Broker (nenhuma troca é feita)
  retornaEstadoAtualLed(pino);
  if(WiFi.status() == WL_CONNECTED && client.connected()){
    String msg = criarStringJsonLeds(!bitRead(estadoLeds, pino));
    String data = "iot-2/evt/eventoL";
    data += pino+1;
    data += "/fmt/json";
    Serial.println(msg);
    client.publish(data.c_str(), msg.c_str());
  }
}

bool retornaEstadoAtualLed(int pino){
  bool bitLed = !(bitRead(comandosPB, pino) ^ bitRead(comandosIR, pino));
  bitWrite(estadoLeds, pino, bitLed);
  return bitLed;
}

String criarStringJsonDht11(float temperature, float humidity) {
  String data = "{";
    data+= "\"d\": {";
      data+="\"temperature\":";
      data+=String(temperature);
      data+=",";
      data+="\"humidity\":";
      data+=String(humidity);
    data+="}";
  data+="}";
  return data;
}

String criarStringJsonLeds(int valor) {
  String data = "{";
      data+="\"value\":";
      data+=String(valor);
  data+="}";
  return data;
}

void pinBotao(int i) {
   #define tempoDebounce 200
   bool estadoBotao[8];
   static bool estadoBotaoAnt[8]; 
   static bool estadoRet[8] = {true,true,true,true,true,true,true,true};
   static unsigned long delayBotao[8] = {0,0,0,0,0,0,0,0};
   if ( (millis() - delayBotao[i]) > tempoDebounce ) {
      estadoBotao[i] = ((analogRead(A0) < 1000)?1:0);
       if ( estadoBotao[i] && (estadoBotao[i] != estadoBotaoAnt[i]) ) {
          estadoRet[i] = !estadoRet[i];
          delayBotao[i] = millis();
          bitWrite(comandosPB, i, estadoRet[i]);
          trocarEstadoLEDs('P', i, estadoRet[i]);
       }
       estadoBotaoAnt[i] = estadoBotao[i];
   }
}

void estadoAtualpushButtons() {
  for (int i = 7; i >= 0; i--) {
    byte iteracao = 255 - pow(2, i);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, estadoLeds); //leds
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, iteracao); //pushButtons
    digitalWrite(LATCH_PIN, HIGH);
    pinBotao(i);
  }
}

void setup() {
  Serial.begin(9600);
  setupPins();
  setupWifi();
  setupLCD();
  desligarTodosLeds();
  connectMQTTServer();
  // Registra o ticker para publicar de tempos em tempos
  publicarTicker.attach_ms(PUBLICAR_INTERVALO, publicar);
  lcdTicker.attach_ms(LCD_INTERVALO, atualizarLCD);
  onlineTicker.attach_ms(ONLINE_INTERVALO, statusOnline);
  Serial.println("Iniciando.");
}

void loop() {
//  int millisTemp = millis();
  if (WiFi.status() == WL_CONNECTED){   
    digitalWrite(PIN_RED_LED_WIFI, HIGH);
    if (!client.connected()) {
      long now = millis();
      if (now - lastReconnectAttempt > 3000) {
        lastReconnectAttempt = now;
        if (connectMQTTServer()) {
          lastReconnectAttempt = 0;
        }
      }
    } else {
      //Bloco executado quando conectado
      client.loop();
      enviarStatusOnline();
    }
  }
  else{
    digitalWrite(PIN_RED_LED_WIFI, LOW);
  }
  estadoAtualpushButtons();
  publicarUmidadeTemperatura();
  verificarSinalReceptorIR();
  //mostrarDadosNoLCD();
//      lcd.setCursor(9, 1);
//      lcd.print("       ");
//      lcd.setCursor(9, 1);
//      lcd.print(millis() - millisTemp);
//      lcd.print("ms");
  delay(20);//delay necessário para o WiFi.status, caso contrário, ocorrem problemas no wifi
}

void mostrarDadosNoLCD() {
    lcd.setCursor(0, 0);
    lcd.print("        ");
    lcd.setCursor(0, 0);
    lcd.print(comandosIR, BIN);
    lcd.setCursor(0, 1);
    lcd.print("        ");
    lcd.setCursor(0, 1);
    lcd.print(comandosPB, BIN);
    lcd.setCursor(8, 0);
    lcd.print(estadoLeds, BIN);
    lcd.setCursor(8, 1);
    lcd.write(255);
}

//Função responsável pela conexão ao servidor MQTT
bool connectMQTTServer() {
  Serial.println("Conectando ao servidor MQTT...");
  if (client.connect(CLIENT_ID.c_str(), "use-token-auth", DEVICE_TOKEN)) {
     Serial.println("Conectado ao Broker MQTT...");
     client.setCallback(callback);

     client.subscribe(COMANDO_TOPICO_L1);
     client.subscribe(COMANDO_TOPICO_L2);
     client.subscribe(COMANDO_TOPICO_L3);
     client.subscribe(COMANDO_TOPICO_L4);
     client.subscribe(COMANDO_TOPICO_L5);
     client.subscribe(COMANDO_TOPICO_L6);
     client.subscribe(COMANDO_TOPICO_L7);
     client.subscribe(COMANDO_TOPICO_L8);
     client.subscribe(COMANDO_TOPICO_M);
     digitalWrite(PIN_GREEN_LED_MQTT, HIGH);
     for (int i = 7; i >= 0; i--){
       trocarEstadoLEDs('A', i, 0000000000);
     }
  } else {
     Serial.print("erro = ");
     Serial.println(client.state());
     digitalWrite(PIN_GREEN_LED_MQTT, LOW);
  }
  return client.connected();
}

void callback(char* topic, unsigned char* payload, unsigned int length) {
  Serial.print("topico ");
  Serial.println(topic);
  StaticJsonDocument<30> jsonDoc;
  DeserializationError error = deserializeJson(jsonDoc, payload);
  
  if (error) {
     Serial.println("Erro no Json Parse");
     return;
  }
  int value = jsonDoc["value"];

  if(topic[17] == 'L'){
    int topicLedId = int(topic[18]) - 49;
    if(value == bitRead(estadoLeds, topicLedId)){
      bitWrite(comandosIR, topicLedId, !bitRead(comandosIR, topicLedId));
      retornaEstadoAtualLed(topicLedId);
    }
  }
  else if(topic[17] == 'M'){
    Serial.println(value);
  }
}

void enviarStatusOnline(){
  if(estouOnline){
    Serial.println("Enviando sinal...");
      String msg = "{\"value\":1}";
    client.publish(EVENTO_TOPICO_IF_ON, msg.c_str());
    estouOnline = false;
  }
}
void publicarUmidadeTemperatura() {
  if (publicaNovoEstado) {
    Serial.println("Atualizando dados");
    // Obtem os dados do sensor DHT
    float temperatura = round(dht.readTemperature());
    float umidade = dht.readHumidity();
    if (!isnan(umidade) && !isnan(temperatura)) {
      
      Serial.print("Publish message: ");
      //Criamos o json que enviaremos para o server mqtt
      String msg = criarStringJsonDht11(temperatura, umidade);
      Serial.println(msg);
      //Publicamos no tópico onde o servidor espera para receber 
      //e gerar o gráfico
      client.publish(EVENTO_TOPICO_DHT11, msg.c_str());
      
      //Mostar a temperatura no LCD
      lcd.setCursor(0, 0);
      lcd.print("Temp: ");
      lcd.setCursor(6, 0);
      lcd.print(temperatura, 0);
      lcd.setCursor(8, 0);
      //Mostra o simbolo do grau formado pelo array
      lcd.write((byte) 0);
      lcd.print("C");
      //Mostar a umidade no LCD
      lcd.setCursor(0, 1);
      lcd.print("Umid: ");
      lcd.setCursor(6, 1);
      lcd.print(umidade, 0);
      lcd.print("%");
      publicaNovoEstado = false;
    } else {
      Serial.println("Erro ao atualizar (sem dados do sensor)");
    }
  }
}

void verificarSinalReceptorIR() {
  if (irrecv.decode( & results)) {
    serialPrintUint64(results.value, HEX);
    Serial.println();
    armazenaTecla = (results.value);
    if (armazenaTecla == 0xFFA25D) //Verifica se a tecla 1 foi acionada
      trocarEstadoLEDs('R', 0, 0000000000);
    if (armazenaTecla == 0xFF629D) //Verifica se a tecla 2 foi acionada
      trocarEstadoLEDs('R', 1, 0000000000);
    if (armazenaTecla == 0xFFE21D) //Verifica se a tecla 3 foi acionada
      trocarEstadoLEDs('R', 2, 0000000000);
    if (armazenaTecla == 0xFF22DD) //Verifica se a tecla 4 foi acionada
      trocarEstadoLEDs('R', 3, 0000000000);
    if (armazenaTecla == 0xFF02FD) //Verifica se a tecla 5 foi acionada
      trocarEstadoLEDs('R', 4, 0000000000);
    if (armazenaTecla == 0xFFC23D) //Verifica se a tecla 6 foi acionada
      trocarEstadoLEDs('R', 5, 0000000000);
    if (armazenaTecla == 0xFFE01F) //Verifica se a tecla 7 foi acionada
      trocarEstadoLEDs('R', 6, 0000000000);
    if (armazenaTecla == 0xFFA857) //Verifica se a tecla 8 foi acionada
      trocarEstadoLEDs('R', 7, 0000000000);
    if (armazenaTecla == 0xFF6897) { //Verifica se a tecla * foi acionada
      //
    }
    if (armazenaTecla == 0xFFB04F) { //Verifica se a tecla # foi acionada
      //
    }
    irrecv.resume(); //Recebe o proximo valor
  }
}
