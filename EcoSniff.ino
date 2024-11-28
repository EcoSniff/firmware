// Definições do Blynk - substitua com suas informações do Blynk
#define BLYNK_TEMPLATE_ID "TMPL2eLUavM2w"
#define BLYNK_TEMPLATE_NAME "EcoSniff"
#define BLYNK_AUTH_TOKEN "hXetaTpAuR3cdPUlcKz3A3E3SloyYHTX"

// Definição para permitir o uso do Serial para depuração no Blynk
#define BLYNK_PRINT Serial

// Inclusão das bibliotecas necessárias
#include <WiFi.h>                 // Biblioteca para conexão Wi-Fi no ESP32
#include <WiFiClient.h>           // Biblioteca para cliente Wi-Fi
#include <BlynkSimpleEsp32.h>     // Biblioteca Blynk para ESP32
#include "DHT.h"                  // Biblioteca para o sensor DHT11
#include "MQUnifiedsensor.h"      // Biblioteca unificada para sensores MQ

// Configuração do sensor DHT11
#define DHTPIN 32                 // Pino digital conectado ao sensor DHT11
#define DHTTYPE DHT11             // Definição do tipo de sensor DHT utilizado

// Inicialização do objeto DHT
DHT dht(DHTPIN, DHTTYPE);

// Configurações gerais para os sensores MQ
#define Board        ("ESP-32")           // Nome da placa
#define Voltage_Resolution  3.3           // Resolução de tensão do ADC (3.3V para ESP32)
#define ADC_Bit_Resolution  12            // Resolução em bits do ADC (12 bits para ESP32)
#define RatioMQ135CleanAir  3.6           // Razão RS/R0 em ar limpo para o sensor MQ-135
#define RatioMQ4CleanAir    4.4           // Razão RS/R0 em ar limpo para o sensor MQ-4
#define RatioMQ137CleanAir  27.5          // Razão RS/R0 em ar limpo para o sensor MQ-137
#define RatioMQ136CleanAir  6.5           // Razão RS/R0 em ar limpo para o sensor MQ-136

// Definição dos pinos analógicos onde os sensores MQ estão conectados
#define MQ135PIN 36                       // Pino analógico para o sensor MQ-135
#define MQ4PIN   39                       // Pino analógico para o sensor MQ-4
#define MQ137PIN 34                       // Pino analógico para o sensor MQ-137
#define MQ136PIN 33                       // Pino analógico para o sensor MQ-136

// Criação dos objetos para cada sensor MQ
MQUnifiedsensor MQ135(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ135PIN, "MQ-135");
MQUnifiedsensor MQ4(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ4PIN, "MQ-4");
MQUnifiedsensor MQ137(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ137PIN, "MQ-137");
MQUnifiedsensor MQ136(Board, Voltage_Resolution, ADC_Bit_Resolution, MQ136PIN, "MQ-136");

// Variáveis booleanas para indicar a disponibilidade dos sensores após calibração
bool sensorMQ135Disponivel = true;
bool sensorMQ4Disponivel = true;
bool sensorMQ137Disponivel = true;
bool sensorMQ136Disponivel = true;

// Credenciais da rede Wi-Fi - substitua pelos dados da sua rede
char ssid[] = "POCO X3 Pro";   // Nome da rede Wi-Fi (SSID)
char pass[] = "12345678";      // Senha da rede Wi-Fi

void setup() {
  // Inicialização da comunicação serial para depuração
  Serial.begin(115200);

  // Inicialização do sensor DHT11
  dht.begin();

  // Configuração dos sensores MQ

  // Configuração do MQ135 para medir CO2
  MQ135.setRegressionMethod(1);           // Define o método de regressão (1 = potência)
  MQ135.setA(110.47); MQ135.setB(-2.862); // Define os coeficientes da equação para CO2
  MQ135.init();                           // Inicializa o sensor MQ135

  // Configuração do MQ4 para medir Metano (CH4)
  MQ4.setRegressionMethod(1);             // Define o método de regressão
  MQ4.setA(1000.0); MQ4.setB(-2.186);     // Define os coeficientes para CH4
  MQ4.init();                             // Inicializa o sensor MQ4

  // Configuração do MQ137 para medir Amônia (NH3)
  MQ137.setRegressionMethod(1);           // Define o método de regressão
  MQ137.setA(102.2); MQ137.setB(-2.473);  // Define os coeficientes para NH3
  MQ137.init();                           // Inicializa o sensor MQ137

  // Configuração do MQ136 para medir Sulfeto de Hidrogênio (H2S)
  MQ136.setRegressionMethod(1);           // Define o método de regressão
  MQ136.setA(44.947); MQ136.setB(-3.445); // Define os coeficientes para H2S
  MQ136.init();                           // Inicializa o sensor MQ136

  // Início da calibração dos sensores MQ
  Serial.println("Calibrando sensores... Por favor, aguarde.");

  // Calibração do MQ135
  {
    int tentativas = 0;
    float calcR0_MQ135 = 0;
    while (tentativas < 3) {
      calcR0_MQ135 = 0;
      // Realiza 10 leituras para calcular o valor de R0
      for(int i = 0; i < 10; i++) {
        MQ135.update(); // Atualiza a leitura do sensor
        calcR0_MQ135 += MQ135.calibrate(RatioMQ135CleanAir); // Calibra usando a razão em ar limpo
        delay(1000); // Aguarda 1 segundo entre as leituras
      }
      // Calcula a média das leituras de R0
      MQ135.setR0(calcR0_MQ135 / 10);
      Serial.print("R0 do MQ135: ");
      Serial.println(MQ135.getR0());
      // Verifica se o valor de R0 é válido
      if(!isinf(MQ135.getR0()) && MQ135.getR0() != 0){
        break; // Calibração bem-sucedida
      }
      tentativas++;
      Serial.println("Tentando calibrar o MQ135 novamente...");
    }
    // Se após 3 tentativas o valor de R0 não for válido, marca o sensor como indisponível
    if(isinf(MQ135.getR0()) || MQ135.getR0() == 0){
      Serial.println("Erro na calibração do MQ135 após 3 tentativas. Prosseguindo sem este sensor.");
      sensorMQ135Disponivel = false;
    }
  }

  // Calibração do MQ4
  {
    int tentativas = 0;
    float calcR0_MQ4 = 0;
    while (tentativas < 3) {
      calcR0_MQ4 = 0;
      for(int i = 0; i < 10; i++) {
        MQ4.update();
        calcR0_MQ4 += MQ4.calibrate(RatioMQ4CleanAir);
        delay(1000);
      }
      MQ4.setR0(calcR0_MQ4 / 10);
      Serial.print("R0 do MQ4: ");
      Serial.println(MQ4.getR0());
      if(!isinf(MQ4.getR0()) && MQ4.getR0() != 0){
        break; // Calibração bem-sucedida
      }
      tentativas++;
      Serial.println("Tentando calibrar o MQ4 novamente...");
    }
    if(isinf(MQ4.getR0()) || MQ4.getR0() == 0){
      Serial.println("Erro na calibração do MQ4 após 3 tentativas. Prosseguindo sem este sensor.");
      sensorMQ4Disponivel = false;
    }
  }

  // Calibração do MQ137
  {
    int tentativas = 0;
    float calcR0_MQ137 = 0;
    while (tentativas < 3) {
      calcR0_MQ137 = 0;
      for(int i = 0; i < 10; i++) {
        MQ137.update();
        calcR0_MQ137 += MQ137.calibrate(RatioMQ137CleanAir);
        delay(1000);
      }
      MQ137.setR0(calcR0_MQ137 / 10);
      Serial.print("R0 do MQ137: ");
      Serial.println(MQ137.getR0());
      if(!isinf(MQ137.getR0()) && MQ137.getR0() != 0){
        break; // Calibração bem-sucedida
      }
      tentativas++;
      Serial.println("Tentando calibrar o MQ137 novamente...");
    }
    if(isinf(MQ137.getR0()) || MQ137.getR0() == 0){
      Serial.println("Erro na calibração do MQ137 após 3 tentativas. Prosseguindo sem este sensor.");
      sensorMQ137Disponivel = false;
    }
  }

  // Calibração do MQ136
  {
    int tentativas = 0;
    float calcR0_MQ136 = 0;
    while (tentativas < 3) {
      calcR0_MQ136 = 0;
      for(int i = 0; i < 10; i++) {
        MQ136.update();
        calcR0_MQ136 += MQ136.calibrate(RatioMQ136CleanAir);
        delay(1000);
      }
      MQ136.setR0(calcR0_MQ136 / 10);
      Serial.print("R0 do MQ136: ");
      Serial.println(MQ136.getR0());
      if(!isinf(MQ136.getR0()) && MQ136.getR0() != 0){
        break; // Calibração bem-sucedida
      }
      tentativas++;
      Serial.println("Tentando calibrar o MQ136 novamente...");
    }
    if(isinf(MQ136.getR0()) || MQ136.getR0() == 0){
      Serial.println("Erro na calibração do MQ136 após 3 tentativas. Prosseguindo sem este sensor.");
      sensorMQ136Disponivel = false;
    }
  }

  Serial.println("Calibração concluída!");

  // Conexão à rede Wi-Fi
  WiFi.begin(ssid, pass); // Inicia a conexão Wi-Fi usando o SSID e a senha fornecidos
  // Aguarda até que a conexão seja estabelecida
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("."); // Imprime um ponto a cada meio segundo enquanto não está conectado
  }
  Serial.println("\nConectado ao Wi-Fi");

  // Configuração e conexão ao Blynk
  Blynk.config(BLYNK_AUTH_TOKEN); // Configura o Blynk com o token de autenticação
  Blynk.connect();                // Conecta ao servidor Blynk
}

void loop() {
  Blynk.run(); // Mantém a comunicação com o Blynk ativa

  // Leitura dos dados do sensor DHT11 (temperatura e umidade)
  float temperatura = dht.readTemperature(); // Lê a temperatura em graus Celsius
  float umidade = dht.readHumidity();        // Lê a umidade relativa em porcentagem

  // Verifica se as leituras são válidas
  if (isnan(temperatura) || isnan(umidade)) {
    Serial.println("Falha ao ler do sensor DHT11!");
  } else {
    // Imprime os valores lidos no monitor serial
    Serial.print("Temperatura: ");
    Serial.print(temperatura);
    Serial.print(" °C | Umidade: ");
    Serial.print(umidade);
    Serial.println(" %");

    // Envia os dados para o aplicativo Blynk
    Blynk.virtualWrite(V1, temperatura); // Envia a temperatura para o Virtual Pin V1
    Blynk.virtualWrite(V2, umidade);     // Envia a umidade para o Virtual Pin V2
  }

  // Leitura e envio dos dados do sensor MQ135 (CO2)
  if(sensorMQ135Disponivel){
    MQ135.update();                       // Atualiza a leitura do sensor
    float ppmMQ135 = MQ135.readSensor();  // Lê a concentração de CO2 em ppm
    Serial.print("CO2 (MQ135): ");
    Serial.print(ppmMQ135);
    Serial.println(" ppm");
    Blynk.virtualWrite(V0, ppmMQ135);     // Envia o valor para o Virtual Pin V0
  } else {
    Serial.println("MQ135 indisponível.");
  }

  // Leitura e envio dos dados do sensor MQ4 (Metano - CH4)
  if(sensorMQ4Disponivel){
    MQ4.update();
    float ppmMQ4 = MQ4.readSensor();
    Serial.print("CH4 (MQ4): ");
    Serial.print(ppmMQ4);
    Serial.println(" ppm");
    Blynk.virtualWrite(V3, ppmMQ4);
  } else {
    Serial.println("MQ4 indisponível.");
  }

  // Leitura e envio dos dados do sensor MQ137 (Amônia - NH3)
  if(sensorMQ137Disponivel){
    MQ137.update();
    float ppmMQ137 = MQ137.readSensor();
    Serial.print("NH3 (MQ137): ");
    Serial.print(ppmMQ137);
    Serial.println(" ppm");
    Blynk.virtualWrite(V4, ppmMQ137);
  } else {
    Serial.println("MQ137 indisponível.");
  }

  // Leitura e envio dos dados do sensor MQ136 (Sulfeto de Hidrogênio - H2S)
  if(sensorMQ136Disponivel){
    MQ136.update();
    float ppmMQ136 = MQ136.readSensor();
    Serial.print("H2S (MQ136): ");
    Serial.print(ppmMQ136);
    Serial.println(" ppm");
    Blynk.virtualWrite(V5, ppmMQ136);
  } else {
    Serial.println("MQ136 indisponível.");
  }

  // Aguarda 2 segundos antes de realizar a próxima leitura
  delay(2000);
}
