#include <Arduino.h>
/*************************************************************
   Monitoracão de Poço Artesiano - Espressif - ESP32 DEVKIT V1
   19.10.2025

  Blynk library is licensed under MIT license
 *************************************************************
  Blynk.Edgent implements:
  - Blynk.Inject - Dynamic WiFi credentials provisioning
  - Blynk.Air    - Over The Air firmware updates
  - Device state indication using a physical LED
  - Credentials reset using a physical Button
 *************************************************************/

#define BLYNK_TEMPLATE_ID           "TMPL2z_jxo6v0"
#define BLYNK_TEMPLATE_NAME         "Artesiano"
#define BLYNK_FIRMWARE_VERSION      "0.1.1"
//#define BLYNK_PRINT Serial
//#define APP_DEBUG
#define USE_ESP32_DEV_MODULE
// #define a custom board in Settings.h (LED no pino GPIO 2)
#include "time.h" 
#include "BlynkEdgent.h"
#include "uptime_formatter.h"
#include "LoRaMESH.h"
#include "HardwareSerial.h"

// ------------------------------------- LORA -----------------------------------------
// pinos e conexões dos módulos
#define PIN_RX 16                // ESP32 Serial2 RX 
#define PIN_TX 17                // ESP32 Serial2 TX 

uint8_t ID = 0;                  // ID = 0 - endereco do modulo MASTER

uint8_t flagRSSI_ID1 = 1;        // flag para envio uma unica vez da informacao
uint8_t flagRSSI_ID2 = 1;        // de falha de comunicacao
uint8_t CommFail_ID1 = 0;        // contador de nr. de  falhas
uint8_t CommFail_ID2 = 0;

//HardwareSerial Serial(0);
HardwareSerial Serial_LoRa(1);

LoRaMESH lora(&Serial_LoRa);

int currentSec;
int currentMin;
int currentHour;
int currentDay;
int currentMonth;
int currentYear;

Preferences  preferences;                      // biblioteca para armazenamento de dados
unsigned int  counterRST;                      // contador de reset's
//uint32_t servicoIoTState;                      // recebe a informação de BlynkState::get();
//String     StrStateBlynk;                      // string de BlynkState para mostrar em um display
bool    sendBlynk = true;                      // usado como flag para envio ao servidor

// ----------------------------------- SETUP Watchdog ------------------------------------------
#include "soc/rtc_wdt.h"
#define  WDT_TIMEOUT        120000                          // XXX miliseconds WDT

// Converte razões do reset para string
const char *resetReasonName(esp_reset_reason_t r) {
  switch (r) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN RESET";
    case ESP_RST_POWERON:   return "POWER ON RESET";        //Power on or RST pin toggled
    case ESP_RST_EXT:       return "EXTERN PIN RESET";      //External pin - not applicable for ESP32
    case ESP_RST_SW:        return "SOFTWARE REBOOT";       //esp_restart()
    case ESP_RST_PANIC:     return "CRASH RESET";           //Exception/panic
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";    //Interrupt watchdog (software or hardware)
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";         //Task watchdog
    case ESP_RST_WDT:       return "RTC WATCHDOG";          //Other watchdog
    case ESP_RST_DEEPSLEEP: return "SLEEP RESET";           //Reset after exiting deep sleep mode
    case ESP_RST_BROWNOUT:  return "BROWNOUT RESET";        //Brownout reset (software or hardware)
    case ESP_RST_SDIO:      return "RESET OVER SDIO";       //Reset over SDIO
    default:                return "";
  }
}
// ----------------------------------- Fim Watchdog ----------------------------------------
void PrintResetReason(void) {
  esp_reset_reason_t r = esp_reset_reason();
  Serial.printf("Reset reason:     %i - %s\r\n\r\n", r, resetReasonName(r));
}

//-------------------------------------  NTP Server time ----------------------------------------------
const char* ntpServer = "br.pool.ntp.org";      // "pool.ntp.org"; "a.st1.ntp.br";
const long  gmtOffset_sec = -10800;             // -14400; Fuso horário em segundos (-03h = -10800 seg)
const int   daylightOffset_sec = 0;             // ajuste em segundos do horario de verão

//-------------------------------------  Sensor interno de temepratura --------------------------------
#ifdef __cplusplus
extern "C" {
#endif
uint8_t temprature_sens_read();
#ifdef __cplusplus
}
#endif
uint8_t temprature_sens_read();

// ------ Protótipo de funções ------
void NTPserverTime(void);
void sendLogReset(void);

// LORA - Comando digital
BLYNK_WRITE(V7){                            // quando aperta = 1; solta = 0
    uint comando = param.asInt();
    Serial.println("Recebido e encaminhado o comando liga LED para o Slave ID 1");
    if(comando == 1){
       lora.digitalWrite(1, 0, 1);  // escreve no Slave 1 - LoRa_GPIO0, LoRa_LOGICAL_LEVEL_HIGH
    } else {
       lora.digitalWrite(1, 0, 0);  //  LoRa_GPIO0, LoRa_LOGICAL_LEVEL_LOW
      }
}

BLYNK_WRITE(V14){                            // quando aperta = 1; solta = 0
    uint comando = param.asInt();
    Serial.println("Recebido e encaminhado o comando liga LED para o Slave ID 2");
    if(comando == 1){
       lora.digitalWrite(2, 0, 1);  // escreve no Slave 1 - LoRa_GPIO0, LoRa_LOGICAL_LEVEL_HIGH
    } else {
       lora.digitalWrite(2, 0, 0);  //  LoRa_GPIO0, LoRa_LOGICAL_LEVEL_LOW
      }
}

void LoraCOMM(){
  //                      RADIO LORA SLAVE 1

  // LORA - Leitura de ruido
  uint16_t NOISE_ID0 = lora.getNoise(0, 1);     // requisita ruido do Master
  Serial.print("Ruido Master ID 00: "); Serial.println(NOISE_ID0);
  Blynk.virtualWrite(V8, NOISE_ID0);
  Serial.println("====================================================="); 
  //___________________________________________________________________________________

  // LORA - Status digital
  bool SlaveGPIO04 = lora.digitalRead(1, 4);    // requisita do Slave 1 - GPIO 4
  Serial.print("Estado do Slave 1 - GPIO 04 IN: "); Serial.println(SlaveGPIO04);
  Blynk.virtualWrite(V4, SlaveGPIO04);

  // LORA - Leitura analogica
  uint16_t SlaveGPIO05 = lora.analogRead(1, 5); // requisita do Slave 1 - GPIO 5
  Serial.print("Nivel (Volts) do Slave 1 - GPIO 05 IN: "); Serial.println(SlaveGPIO05);
  Blynk.virtualWrite(V5, SlaveGPIO05);

  // LORA - Leitura de ruido minimo
  uint16_t NOISE_ID1 = lora.getNoise(1, 2);     // requisita ruido minimo do Slave 1 
  Serial.print("Ruido Slave ID 01:  "); Serial.println(NOISE_ID1);
  Blynk.virtualWrite(V9, NOISE_ID1);

  // LORA - Leitura de RSSI de ida
  uint16_t RSSI_ID1 = lora.getRSSI(1, 2);       // requisita RSSI de ida do Slave 1
  Serial.print("RSSI Slave ID 01:   "); Serial.println(RSSI_ID1);
  Blynk.virtualWrite(V15, RSSI_ID1);

  if (RSSI_ID1 <= 254){flagRSSI_ID1 = 1;}         // flag para enviar so uma vez
  if (RSSI_ID1 > 254 && flagRSSI_ID1 == 1){
    CommFail_ID1 ++;                              // incrementa 1
    Serial.println("ALARME - Caixa D`água sem comunicação!");
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "", CommFail_ID1, "Caixa D`água falha de comunicação!");
    flagRSSI_ID1 = 0;
  }
  
  Serial.println("====================================================="); 
  delay(2000);
  //                      RADIO LORA SLAVE 2

  // LORA - Status digital
  bool SlaveGPIO04_2 = lora.digitalRead(2, 4);     // requisita do Slave 2 - GPIO 4
  Serial.print("Estado do Slave 1 - GPIO 04 IN: "); Serial.println(SlaveGPIO04_2);
  Blynk.virtualWrite(V12, SlaveGPIO04_2);

  // LORA - Leitura analogica
  uint16_t SlaveGPIO05_2 = lora.analogRead(2, 5);  // requisita do Slave 2 - GPIO 5
  Serial.print("Nivel (Volts) do Slave 1 - GPIO 05 IN: "); Serial.println(SlaveGPIO05_2);
  Blynk.virtualWrite(V13, SlaveGPIO05_2);

  // LORA - Leitura de ruido minimo
  uint16_t NOISE_ID2 = lora.getNoise(2, 2);        // requisita ruido minimo do Slave 2
  Serial.print("Ruido Slave ID 02:   "); Serial.println(NOISE_ID2);
  Blynk.virtualWrite(V11, NOISE_ID2);

  // LORA - Leitura de RSSI de ida
  uint16_t RSSI_ID2 = lora.getRSSI(2, 2);          // requisita RSSI de ida do Slave 2
  Serial.print("RSSI Slave ID 02:   "); Serial.println(RSSI_ID2);
  Blynk.virtualWrite(V16, RSSI_ID2);

  if (RSSI_ID2 <= 254){flagRSSI_ID2 = 1;}         // flag para enviar so uma vez
  if (RSSI_ID2 > 254 && flagRSSI_ID2 == 1){
    CommFail_ID2 ++;                              // incrementa 1
    Serial.println("ALARME - Poço sem comunicação!");
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "", CommFail_ID2, "Poço falha de comunicação!");
    //Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "",resetReasonName(r), " ",counterRST);
    flagRSSI_ID2 = 0;
  }

  Serial.println("====================================================="); 
  delay(2000);
}

//___________________________________________________________________________________________________________

void Main2(){
  unsigned long tempo_start = millis();      // usado no final para imprimir o tempo de execução dessa rotina
    // habilitar para teste de Sofware Reboot ou RTC Watchdog
  //if (currentSec == 59){ESP.restart();}
  //if (currentSec == 59){int i = WDT_TIMEOUT/1000;
  //   while(1){Serial.print("Watchdog vai atuar em... "); Serial.println (i);delay(980);i = i - 1;}}
  
  if ((currentHour == 10) && (currentMin == 0) && (currentSec < 10)){
    preferences.begin  ("my-app", false);              // inicia 
    preferences.putUInt("counterRST", 0);              // grava em Preferences/My-app/counterRST, counterRST
    counterRST = preferences.getUInt("counterRST", 0); // Le da NVS
    preferences.end();
    Serial.println("A data e hora foram recalibradas no relógio interno, e o contador de RESETs foi zerado!");

    CommFail_ID1 = 0;        // zera contador de nr. de falhas
    CommFail_ID2 = 0;

    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, " Auto calibrado o Relógio");
  }

  rtc_wdt_feed();                                 // reseta o temporizador do Watchdog;
  NTPserverTime();                                // busca e envia a data/hora

    // ------ Integrações com o app ------
servicoIoTState = BlynkState::get();    // requisita estado da biblioteca BlynkEdgent.h
   switch (servicoIoTState) {
      case 0: StrStateBlynk = "WAIT CONFIG"  ; break;
      case 1: StrStateBlynk = "CONFIGURING"  ; break;
      case 2: StrStateBlynk = "NET Connect"  ; break;
      case 3: StrStateBlynk = "CLOUD Connect"; break;
      case 4: StrStateBlynk = "RUNNING..."   ; break;
      case 5: StrStateBlynk = "UPGRADE OTA"  ; break;
      case 6: StrStateBlynk = "STATION Mode" ; break;
      case 7: StrStateBlynk = "RESET Config" ; break;
      case 8: StrStateBlynk = "ERROR !"      ; break;
    }

  // ------ Coleta e envio do nivel de RF ------
  long rssi = WiFi.RSSI();
  Serial.print("RF Signal Level: ");
  Serial.println(rssi);                                                  // imprime o indicador de nível de sinal Wi-Fi
  Blynk.virtualWrite(V3, rssi);                                          // envia ao Blynk informação RF Signal Level
  Blynk.virtualWrite(V10, uptime_formatter::getUptime());                // envia ao Blynk a informação uptime

  Serial.printf("Período (ms) do Main2: %u\n", (millis() - tempo_start)); // cálculo do tempu utilizado até aqui
  Serial.println("------------------------------------------------------");  
}

void NTPserverTime(){          // Horário recebido da internet
  struct tm timeinfo;
     //  "%u, %d.%m.%Y %H:%M:%S"  = 1, 17.08.2021 14:33:33
     //   %u = dia da semana em decimal, range 1 a 7, Segunda = 1.
     // https://www.ibm.com/docs/en/z-workload-scheduler/9.5.0?topic=troubleshooting-date-time-format-reference-strftime 
  if(!getLocalTime(&timeinfo)){                                    // testa conexão com a internet
    Serial.println("Sincronizando com o servidor NTP...");
    Blynk.virtualWrite(V1, "Sincronizando...");                    // envia ao Blynk
    } else {                                                       // quando conectar segue...
      currentYear  = timeinfo.tm_year + 1900;
      currentMonth = timeinfo.tm_mon + 1;
      currentDay   = timeinfo.tm_mday;

      currentHour  = timeinfo.tm_hour;
      currentMin   = timeinfo.tm_min;
      currentSec   = timeinfo.tm_sec;

      sendLogReset();                                               // manda o log uma vez

      char RTC_Time[64];                                            //Cria uma string formatada da estrutura "timeinfo"
      sprintf(RTC_Time, "%02d.%02d.%04d  -  %02d:%02d:%02d", currentDay, currentMonth, currentYear, currentHour, currentMin, currentSec);
      Serial.print("Data/hora do sistema:  ");
      Serial.println(RTC_Time);
      Blynk.virtualWrite(V1, RTC_Time);                             // envia ao Blynk a informação de data, hora e minuto do RTC
    
      int temp=((temprature_sens_read() - 32) / 1.8)-41;             // -7 Viamão,   -31 Restinga Seca 
      Serial.print("Temperatura: ");
      Serial.print(temp);
      Serial.println(" C");
      Blynk.virtualWrite(V2, temp);
    }
}

void sendLogReset(){
  // envia razao do reset para o servidor
  if ((servicoIoTState==4) && (sendBlynk)){
    Serial.print("               BLYNK:  RODANDO COM SUCESSO!"); // delay(100);
    esp_reset_reason_t r = esp_reset_reason();
    Serial.printf("\r\nReset reason %i - %s\r\n", r, resetReasonName(r));
    Blynk.virtualWrite(V45, currentDay, "/", currentMonth, " ", currentHour, ":", currentMin, "",resetReasonName(r), " ",counterRST);
    Blynk.virtualWrite(V53, counterRST);                        // envia para tela do app
    delay(500);
    // se reiniciar por (1) POWER ON RESET
    /*
    if (r == 1){
      //Blynk.logEvent("falha_de_energia", String("Teste - Falha de Energia!"));
      //Blynk.logEvent("falha_de_energia");                 // registra o evento falha_de_energia no servidor
      }
    */
    sendBlynk = false;
  }
}

void setup(){
  // configuração do RTC Watchdog
  rtc_wdt_protect_off();                  //Disable RTC WDT write protection
  rtc_wdt_set_stage(RTC_WDT_STAGE0, RTC_WDT_STAGE_ACTION_RESET_RTC);
  rtc_wdt_set_time (RTC_WDT_STAGE0, WDT_TIMEOUT);
  rtc_wdt_enable();                       //Start the RTC WDT timer
  rtc_wdt_protect_on();                   //Enable RTC WDT write protection
 
  Serial.begin(115200);
  delay(100);

  edgentTimer.setInterval ( 1000L, Main2);        // rotina se repete a cada XXXXL (milisegundos)
  edgentTimer.setInterval (55000L, LoraCOMM);     // rotina
  BlynkEdgent.begin();

  Serial_LoRa.begin(9600, SERIAL_8N1, PIN_RX, PIN_TX);
  lora.begin(true);

   if(lora.localId != ID){
    if(!lora.setNetworkID(ID)){
      Serial.println("Erro ao definir o novo ID");
      while(1);
    }

    Serial.println("ID configurado com sucesso!");

    if(!lora.setBPS(BW125, SF7, CR4_5)){
      Serial.println("Erro ao configurar bps");
      while(1);
    }

    Serial.println("Parametros LoRa configurados com sucesso!");
    
    if(!lora.setClass(CLASS_C, WINDOW_15s)){
      Serial.println("Erro ao configurar a classe");
      while(1);
    }

    Serial.println("Modo de operacao configurado com sucesso!");

    if(!lora.setPassword(123)){
      Serial.println("Erro ao gravar a senha ou a senha gravada não condiz com a senha definida");
      while(1);
    }

    Serial.println("Senha configurada com sucesso!");
  }

  // se for Slave ID=0 faz a GPIO 0, sem pull up, como saida, inciando em nivel baixo
  //if(ID == 1)
  //  lora.config_digital_gpio(LoRa_GPIO0, LoRa_NOT_PULL, LoRa_INOUT_DIGITAL_OUTPUT, LoRa_LOGICAL_LEVEL_LOW);

  // config LoRaMESH
  Serial.println("LocalID: " + String(lora.localId));
  Serial.println("UniqueID: " + String(lora.localUniqueId));
  Serial.println("Pass <= 65535: " + String(lora.registered_password));
  Serial.println("Radio LoRaMESH Master iniciado...");

  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);   // inicia e busca as infos de data e hora (NTP)
  PrintResetReason();                     // imiprime a razão do último reset

  // ------   Inicia e cria espaco na memoria NVS - namespace:my-app    ------
  preferences.begin("my-app", false);   // Note: Namespace name is limited to 15 chars.
  //preferences.clear();                // remove all preferences under the opened namespace
  //preferences.remove("counterRST");   // or remove the counter key only
  counterRST = preferences.getUInt("counterRST", 0); // Le da NVS, se counterRST nao existir retorna o valor 0
  counterRST++;                         // incrementa a cada restart
  //Serial.printf("Quantidade de RESETs: %u\n", counterRST);
  preferences.putUInt("counterRST", counterRST);  // grava em Preferences/My-app/counterRST, counterRST
  preferences.end();
  delay(50);
  Serial.printf("Quantidade de RESETs: %u\n", counterRST);
  sendBlynk = true;

  delay(5000);
}

void loop(){
  BlynkEdgent.run();

}

