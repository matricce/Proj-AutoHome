
const String ORG = " ";
const String DEVICE_TYPE = " ";
const String DEVICE_ID = " ";
#define DEVICE_TOKEN " "
#define WIFI_SSID " "
#define WIFI_PASSWORD " "



//v2.5.1.1
#define CODE_LEVEL_DEBUG "versão 2.5.1.1"
//- o algoritmo pode travar quando cai a conexão e algum push button está sendo clicado
//
extern "C" {
#include "ping.h"
}

#include "os_type.h"
#include <ESP8266WiFi.h>
#include <ESP8266Ping.h>
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
#include "SSD1306Wire.h"//remover

ping_option pingOps; // Struct utilizada por ping_start, https://github.com/esp8266/Arduino/blob/master/tools/sdk/include/ping.h
static os_timer_t ping_timer;

const String ORG = " ";
const String DEVICE_TYPE = " ";
const String DEVICE_ID = " ";
#define DEVICE_TOKEN " "
#define WIFI_SSID " "
#define WIFI_PASSWORD " "

#define PING_DELAY_S 1 //amount of time between pings and before a ping times out, sec
#define PING_COUNT 1 //count of pings per request
#define DELAY 3000 // Time to wait after completing a full ping request (doing PING_COUNT pings) and before the next one
IPAddress PING_TARGET(1,1,1,1); // The IP address you're trying to ping

const String CLIENT_ID = "d:" + ORG + ":" + DEVICE_TYPE + ":" + DEVICE_ID;
const String MQTT_SERVER = ORG + ".messaging.internetofthings.ibmcloud.com";
///13, 14, 15 //D7, D5, D8  ////  14, 12, 11
#define CLOCK_PIN           15  //(D8)conetado ao SRCLK pino 11(SHCP) - shift register clock
#define LATCH_PIN           14  //(D5)conectado ao RCLK pino 12(STCP) - output register clock
#define DATA_PIN            13  //(D7)conectado ao SER pino 14(DS) - input
#define DHT_PIN             0   //(D3)
#define RECV_PIN            3   //(RX) se declarar como D4, não funciona corretamente
#define PUSH_BUTTONS_PIN    16  //(D0)
#define ENABLE_74HC595_PIN      12  //(D6) Pino utilizado como output para ligar o 74hc595
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
#define ONLINE_INTERVALO 1000 * 4 //envia um evento para o Broker a cada 4seg
#define WIFI_INTERVALO 200 //checa se o wifi está conectado a cada 200ms
#define TEMPO_DEBOUNCE 150

WiFiClient wifiClient;
PubSubClient client(MQTT_SERVER.c_str(), 1883, wifiClient);
DHT dht(DHT_PIN, DHTTYPE);
Ticker publicarTicker;
Ticker onlineTicker;
Ticker wifiTicker;
LiquidCrystal_I2C lcd(0x27, 16, 2);
SSD1306Wire  display(0x3c, D2, D1);//remover
IRrecv irrecv(RECV_PIN);
decode_results results;

bool executaPingarNoServidor = true;
bool executaPublicarUmidadeTemperatura = true;
bool executaTestarConexaoWifi = true;
bool conexaoInternet = false;
bool pingResposta = false;
bool executaTestePing = true;
float armazenaTecla;
long lastReconnectAttempt = 0;
byte byteEstadoLEDs = 255;
byte byteEstadoPb = 255;
byte byteEstadoIrWeb = 255;
byte termometro[8] = {B00100,B01010,B01010,B01110,B01110,B11111,B11111,B01110};
byte gota[8] = {B00100,B00100,B01010,B01010,B10001,B10001,B10001,B01110};
byte wifi1[8] = {B00000,B00000,B00000,B00000,B00000,B00000,B00000,B00100};
byte wifi2[8] = {B00000,B00000,B00000,B00000,B00100,B01010,B00000,B00100};
byte wifi3[8] = {B00000,B01110,B10001,B00000,B00100,B01010,B00000,B00100};
byte ledDesligado[8] = {B01110,B10001,B11111,B10101,B10101,B01110,B01110,B01110};
byte ledLigado[8] = {B01110,B11111,B11111,B11111,B11111,B01110,B01110,B01110};
byte mqtt1[8] = {B00000,B00000,B01110,B11111,B11111,B11111,B01110,B00000};
///////////////////////#########Início - PING#########///////////////////////
// Essa função é chamada quando ping_timer é acionado:
static void ICACHE_FLASH_ATTR ping_cb(void *arg){
  os_timer_disarm(&ping_timer);
  Serial.println("---");
  Serial.print("Pinging: ");
  Serial.print(PING_TARGET);
  Serial.println(" with 32 bytes of data:");
  ping_start(&pingOps);
}

//Essa função é chamada quando o ping é recebido ou o tempo do request acaba:
static void ICACHE_FLASH_ATTR ping_recv (void* arg, void *pdata){
  struct ping_resp *pingrsp = (struct ping_resp *)pdata;

  if (pingrsp->bytes > 0){
    conexaoInternet = true;
    Serial.print("Reply from: ");
    Serial.print(PING_TARGET);
    Serial.print(": ");
    Serial.print("bytes=");
    Serial.print(pingrsp->bytes);
    Serial.print(" time=");
    Serial.print(pingrsp->resp_time);
    Serial.println("ms");
  }
  else{
    conexaoInternet = false;
    Serial.println("Request timed out");
  }
  pingResposta = true;
}

// Essa função é chamada quando o ping request é completado
// (i.e., after PING_COUNT pings are done)
static void ICACHE_FLASH_ATTR ping_sent (void* arg, void *pdata){
  struct ping_resp *pingrsp = (struct ping_resp *)pdata;

  Serial.println();
  Serial.print("Ping statistics for: ");
  Serial.println(PING_TARGET);
  Serial.print("Packets: Sent = ");
  Serial.print(pingrsp->total_count);
  Serial.print(", Recieved = ");
  Serial.print(pingrsp->total_count-pingrsp->timeout_count);
  Serial.print(", Lost = ");
  Serial.print(pingrsp->timeout_count);
  Serial.print(" (");
  Serial.print(float(pingrsp->timeout_count)/pingrsp->total_count*100);
  Serial.println("% loss)");

  //Ping request completo, rearma o timer então um novo request é enviado em DELAY segundos
  //os_timer_arm(&ping_timer, DELAY, 1);//Inicia o serviço de ping
  //os_timer_disarm(&ping_timer);//Pausa o serviço de ping
}
///////////////////////#########Fim - PING#########///////////////////////
void publicar() {
  executaPublicarUmidadeTemperatura = true;
}

void pingar(){
  executaPingarNoServidor = true;
}

void testewifi(){
  executaTestarConexaoWifi = true;
}

void setupPins(){
  pinMode(LATCH_PIN, OUTPUT);
  pinMode(CLOCK_PIN, OUTPUT);
  pinMode(DATA_PIN, OUTPUT);
  //pinMode(RECV_PIN, INPUT);//RX//desligar esse pino para vizualizar o monitor Serial
  pinMode(PUSH_BUTTONS_PIN, INPUT_PULLUP);
  pinMode(ENABLE_74HC595_PIN, OUTPUT);
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

void setupPing(){
  pingOps.count = PING_COUNT;
  pingOps.ip = uint32_t(PING_TARGET);
  pingOps.coarse_time = PING_DELAY_S;
  ping_regist_sent(&pingOps, ping_sent); 
  ping_regist_recv(&pingOps, ping_recv);

  os_timer_disarm(&ping_timer);
  os_timer_setfn(&ping_timer, (os_timer_func_t *)ping_cb, (void *)0);

  //O timer ping_timer irá disparar depois decorrer DELAY segundos, chamando a função ping_cb
  //O timer é desarmado na primeira linha de ping_cb, então ele não dispara em todos os segundos de DELAY continuamente
  //os_timer_arm(&ping_timer, DELAY, 1);
}

void setupLCD() {
  Wire.begin(D2, D1);
  lcd.begin();
  lcd.createChar(0, mqtt1);
  lcd.createChar(1, wifi1);
  lcd.createChar(2, wifi2);
  lcd.createChar(3, wifi3);
  lcd.createChar(4, termometro);
  lcd.createChar(5, gota);
  lcd.createChar(6, ledDesligado);
  lcd.createChar(7, ledLigado);
}

void setupDisplay(){//remover
  display.init();
  display.flipScreenVertically();
  //Apaga o display
  display.clear();
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  //Seleciona a fonte
  display.setFont(ArialMT_Plain_16);
  display.drawString(0, 0, CODE_LEVEL_DEBUG);
  display.setFont(ArialMT_Plain_10);
  display.drawString(0, 15, "L1");
  display.drawString(0, 24, "L2");
  display.drawString(0, 33, "L3");
  display.drawString(0, 42, "L4");
  display.drawString(0, 51, "L5");
  display.display();
}

void desenharAlgoOledDebug(int posicaoX, int posicaoY, String texto){//remover
  display.setColor(BLACK);
  display.fillRect(posicaoX, posicaoY+3, 128, 9);//coluna, linha, largura, altura
  display.setColor(WHITE);
  display.drawString(posicaoX, posicaoY, texto);
  display.display();
}

void desligarTodosLeds() {
  digitalWrite(LATCH_PIN, LOW);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 255);
  shiftOut(DATA_PIN, CLOCK_PIN, LSBFIRST, 255);
  digitalWrite(LATCH_PIN, HIGH);
}

void trocarEstadoLEDs(char source, int pino){
  if(source == 'R'){//recebido do controle (troca feita agora)
    if (bitRead(byteEstadoIrWeb, pino) == 0)
      bitWrite(byteEstadoIrWeb, pino, 1);
    else
      bitWrite(byteEstadoIrWeb, pino, 0);
  }
  else if(source == 'P'){}//recebido dos botões (troca foi feita antes)
  else if(source == 'A'){}//recebido do Broker (nenhuma troca é feita)
  retornaEstadoAtualLed(pino);
  if(client.connected()){
    String msg = criarStringJsonLeds(!bitRead(byteEstadoLEDs, pino));
    String data = "iot-2/evt/eventoL";
    data += pino+1;
    data += "/fmt/json";
    Serial.println(msg);
    client.publish(data.c_str(), msg.c_str());
    onlineTicker.attach_ms(ONLINE_INTERVALO, pingar);
  }
}

bool retornaEstadoAtualLed(int pino){
  bool bitLed = !(bitRead(byteEstadoPb, pino) ^ bitRead(byteEstadoIrWeb, pino));
  bitWrite(byteEstadoLEDs, pino, bitLed);
  desenharIconeLeds(pino);
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

String criarStringJsonInformacoes(int forcaWifiSinal) {
  String data = "{";
    data+= "\"d\": {";
      data+="\"rssi\":";
      data+=String(forcaWifiSinal);
      data+=",";
      data+="\"value\":";
      data+="1";
    data+="}";
  data+="}";
  return data;
}

void verificaSeBotaoPressionado(int i) {
   bool estadoBotao;
   static bool estadoBotaoAnt[8]; 
   static bool estadoRetencao[8] = {true,true,true,true,true,true,true,true};
   static unsigned long delayBotao[8] = {0,0,0,0,0,0,0,0};
   if ( (millis() - delayBotao[i]) > TEMPO_DEBOUNCE ) {
      estadoBotao = !digitalRead(PUSH_BUTTONS_PIN);
      if(estadoBotao){
        delayBotao[i] = millis();
        if (estadoBotao != estadoBotaoAnt[i] ) {
           estadoRetencao[i] = !estadoRetencao[i];
           bitWrite(byteEstadoPb, i, estadoRetencao[i]);
           trocarEstadoLEDs('P', i);
        }
      }
       estadoBotaoAnt[i] = estadoBotao;
   }
}

void estadoAtualpushButtons() {
  for (int i = 7; i >= 0; i--) {
    byte iteracao = 255 - pow(2, i);
    digitalWrite(LATCH_PIN, LOW);
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, byteEstadoLEDs); //leds
    shiftOut(DATA_PIN, CLOCK_PIN, MSBFIRST, iteracao); //pushButtons
    digitalWrite(LATCH_PIN, HIGH);
    verificaSeBotaoPressionado(i);
  }
}

bool testarConexaoWifi(){
  static bool conectado = true;
  if(executaTestarConexaoWifi){
    if(WiFi.status() == WL_CONNECTED){
      conectado = true;
    }else{
      conectado = false;
    }
    executaTestarConexaoWifi = false;
  }
  return conectado;  
}

void setup() {
  Serial.begin(9600);
  setupPins();
  desligarTodosLeds();
  setupWifi();
  setupPing();
  setupLCD();
  setupDisplay();//remover
  digitalWrite(ENABLE_74HC595_PIN, HIGH);
  //connectMQTTServer();
  for (int i = 7; i >= 0; i--){
  desenharIconeLeds(i);
  }
  // Registra o ticker para publicar de tempos em tempos
  publicarTicker.attach_ms(PUBLICAR_INTERVALO, publicar);
  onlineTicker.attach_ms(ONLINE_INTERVALO, pingar);
  wifiTicker.attach_ms(WIFI_INTERVALO, testewifi);
}

void loop() {
  if(testarConexaoWifi()){
    desenharAlgoOledDebug(0, 15, "Wifi conectado");//remover
    desenharIconeWifiConectado(15, 0);
    if (!client.connected()) {
      if(testarConexaoInternet()){
        if (millis() - lastReconnectAttempt > 5000) {
          lastReconnectAttempt = millis();
          if (connectMQTTServer()) {
            lastReconnectAttempt = 0;
          }
        }
      }
      else{
        desenharAlgoOledDebug(0, 24, "Sem internet");//remover
        desenharIconeMqttDesconectado(14, 0);
      }
    }else{
      client.loop();
      pingarNoServidor();
    }
  }else{
    desenharAlgoOledDebug(0, 15, "Wifi desconectado");//remover
    desenharIconeWifiDesconectado(15, 0);
  }
  estadoAtualpushButtons();
  publicarUmidadeTemperatura();
  verificarSinalReceptorIR();
  desenharWifiForcaSinal(11,0);
  showMillis();
}

bool testarConexaoInternet(){
  static unsigned long testePing = millis();
  static bool conectado = false;
  if(executaTestePing){
    os_timer_arm(&ping_timer, DELAY, 1);
    executaTestePing = false;
  }
  if(((millis() - testePing) > DELAY) || pingResposta){
    if(conexaoInternet)
      desenharAlgoOledDebug(0, 42, "Success!!");//remover
  else
      desenharAlgoOledDebug(0, 42, "Error :(");//remover
    desenharAlgoOledDebug(0, 51, String(millis() - testePing));//remover
    testePing = millis();
    os_timer_disarm(&ping_timer);
    executaTestePing = true;
    pingResposta = false;
  }
  conectado = conexaoInternet;
  conexaoInternet = false;
  return conectado;
}

void showMillis(){//remover
  static unsigned long testeMillis = millis();
  if((millis() - testeMillis) > 100){
  desenharAlgoOledDebug(0, 33, String(millis()));//remover
  testeMillis = millis();
  }
}

bool connectMQTTServer() {
  Serial.println("Conectando ao servidor MQTT...");
  int unsigned agoraMqtt = millis();//remover
  if (client.connect(CLIENT_ID.c_str(), "use-token-auth", DEVICE_TOKEN)) {
     String result = "Conectado MQTT ";//remover
     result += String(millis() - agoraMqtt);//remover
     result += "ms";//remover
     desenharAlgoOledDebug(0, 24, result);//remover
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
     desenharIconeMqttConectado(14, 0);
     for (int i = 7; i >= 0; i--){
       trocarEstadoLEDs('A', i);
     }
  } else {
     Serial.print("erro = ");
     Serial.println(client.state());
     desenharIconeMqttDesconectado(14, 0);
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
    if(value == bitRead(byteEstadoLEDs, topicLedId)){
      bitWrite(byteEstadoIrWeb, topicLedId, !bitRead(byteEstadoIrWeb, topicLedId));
      retornaEstadoAtualLed(topicLedId);
    }
  }
  else if(topic[17] == 'M'){
    Serial.println(value);
  }
}

void pingarNoServidor(){
  if(executaPingarNoServidor){
    Serial.println("Enviando sinal ...");
    String msg = criarStringJsonInformacoes(WiFi.RSSI());
    client.publish(EVENTO_TOPICO_IF_ON, msg.c_str());
    executaPingarNoServidor = false;
  }
}

void desenharIconeWifiDesconectado(int coluna, int linha){
  static unsigned long delayWifiIcone = 0;
  static int frameWifiIcone = 1;
  static bool sentidoFrame = 0;
  if((millis() - delayWifiIcone > 500)){
    lcd.setCursor(coluna, linha);
    lcd.write((byte)frameWifiIcone);
    if(frameWifiIcone == 3)
      sentidoFrame = 0;
    else if(frameWifiIcone == 1)
      sentidoFrame = 1;
    sentidoFrame ? (frameWifiIcone++):(frameWifiIcone--);
    delayWifiIcone = millis();
  }  
}

void desenharWifiForcaSinal(int coluna, int linha){
  static unsigned long delayWifiSinal = 0;
  if((millis() - delayWifiSinal > 1000)){
    lcd.setCursor(coluna, linha);
    lcd.print(WiFi.RSSI());
    delayWifiSinal = millis();
  }  
}

void desenharIconeWifiConectado(int coluna, int linha){
  lcd.setCursor(coluna, linha);
  lcd.write((byte)3);
}
void desenharIconeMqttDesconectado(int coluna, int linha){
  lcd.setCursor(coluna, linha);
  lcd.print((char)111);
}
void desenharIconeMqttConectado(int coluna, int linha){
  lcd.setCursor(coluna, linha);
  lcd.write((byte)0);
}
void desenharIconeLeds(int led){
  lcd.setCursor(led, 1);
  (bitRead(byteEstadoLEDs, led)?lcd.write((byte)6):lcd.write((byte)7));
}
void publicarUmidadeTemperatura() {
  if (executaPublicarUmidadeTemperatura) {
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
      lcd.write((byte)4);
      lcd.print(temperatura, 0);
      lcd.print((char)223);
      lcd.print("C");
      lcd.setCursor(6, 0);
      lcd.write((byte)5);
      lcd.print(umidade, 0);
      lcd.print("%");
      executaPublicarUmidadeTemperatura = false;
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
      trocarEstadoLEDs('R', 0);
      trocarEstadoLEDs('R', 1);
    if (armazenaTecla == 0xFFE21D) //Verifica se a tecla 3 foi acionada
      trocarEstadoLEDs('R', 2);
    if (armazenaTecla == 0xFF22DD) //Verifica se a tecla 4 foi acionada
      trocarEstadoLEDs('R', 3);
    if (armazenaTecla == 0xFF02FD) //Verifica se a tecla 5 foi acionada
      trocarEstadoLEDs('R', 4);
    if (armazenaTecla == 0xFFC23D) //Verifica se a tecla 6 foi acionada
      trocarEstadoLEDs('R', 5);
    if (armazenaTecla == 0xFFE01F) //Verifica se a tecla 7 foi acionada
      trocarEstadoLEDs('R', 6);
    if (armazenaTecla == 0xFFA857) //Verifica se a tecla 8 foi acionada
      trocarEstadoLEDs('R', 7);
    if (armazenaTecla == 0xFF6897) { //Verifica se a tecla * foi acionada
      //
    }
    if (armazenaTecla == 0xFFB04F) { //Verifica se a tecla # foi acionada
      //
    }
    irrecv.resume(); //Recebe o proximo valor
  }
}
