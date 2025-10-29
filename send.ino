#include <WiFi.h>
#include <esp_now.h>
#define MIC_PIN      36
#define BUTTON_PIN   25
#define SAMPLE_RATE  8000
#define DURATION_SEC 1.5
#define BUFFER_SIZE  ((int)(SAMPLE_RATE * DURATION_SEC)) 
#define LPC_ORDER    8
#define PRE_EMPHASIS_ALPHA 0.97
#define ENERGY_THRESHOLD 0.001 
#define MIN_TIME_BETWEEN_SENDS 500  

float* audioBuffer;
float* preEmphasized;
float r[LPC_ORDER + 1];
float a[LPC_ORDER + 1];
bool isRecording = false;
unsigned long lastSendTime = 0;  

uint8_t receiverMac[] = {0x94, 0x54, 0xC5, 0xA9, 0x36, 0xC4};

typedef struct {
  float lpc[LPC_ORDER];
  float energy;
} LPCPacket;

LPCPacket packet;

void apply_hamming_window(float* data, int len) {
  for (int i = 0; i < len; i++) {
    float w = 0.54 - 0.46 * cos(2 * PI * i / (len - 1));
    data[i] *= w;
  }
}

void pre_emphasis(float* input, float* output, int len, float alpha) {
  output[0] = input[0];
  for (int i = 1; i < len; i++) {
    output[i] = input[i] - alpha * input[i - 1];
  }
}

void compute_autocorrelation(float* x, int N, int order, float* r) {
  for (int lag = 0; lag <= order; lag++) {
    float sum = 0.0;
    for (int i = 0; i < N - lag; i++) {
      sum += x[i] * x[i + lag];
    }
    r[lag] = sum;
  }
}

bool levinson_durbin(float* r, float* a, int order) {
  if (r[0] < ENERGY_THRESHOLD) {
    Serial.println("Energi sinyal terlalu rendah");
    return false;
  }
  
  float e = r[0];
  if (e <= 0.0 || isnan(e) || isinf(e)) {
    Serial.println("Error energi tidak valid");
    return false;
  }

  a[0] = 1.0;
  float* k = (float*)ps_malloc((order+1) * sizeof(float));
  float* temp_a = (float*)ps_malloc((order+1) * sizeof(float));
  
  if (!k || !temp_a) {
    Serial.println("Gagal alokasi memori untuk perhitungan Levinson-Durbin");
    if (k) free(k);
    if (temp_a) free(temp_a);
    return false;
  }

  for (int i = 1; i <= order; i++) {
    float acc = 0.0;
    for (int j = 1; j < i; j++) {
      acc += a[j] * r[i - j];
    }

    k[i] = (r[i] - acc) / e;
    
    if (isnan(k[i]) || isinf(k[i])) {
      Serial.printf("Koef. refleksi tidak valid pada i=%d: %f\n", i, k[i]);
      free(k);
      free(temp_a);
      return false;
    }
    

    k[i] = constrain(k[i], -0.99, 0.99);
    
    memcpy(temp_a, a, (order+1) * sizeof(float));
    
    for (int j = 1; j < i; j++) {
      a[j] = temp_a[j] - k[i] * temp_a[i-j];
    }
    a[i] = k[i];
    
    e *= (1.0 - k[i] * k[i]);
    
    if (e <= 0.0 || isnan(e) || isinf(e)) {
      Serial.printf("Error prediction tidak valid pada i=%d: %f\n", i, e);
      free(k);
      free(temp_a);
      return false;
    }
  }
  
  free(k);
  free(temp_a);

  for (int i = 1; i <= order; i++) {
    if (isnan(a[i]) || isinf(a[i]) || fabs(a[i]) > 2.0) {
      Serial.printf("Koefisien LPC tidak valid a[%d]=%f\n", i, a[i]);
      return false;
    }
  }

  return true;
}

void onSent(const uint8_t* mac, esp_now_send_status_t status) {
  Serial.print("Kirim LPC: ");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Sukses" : "Gagal");
}

void setup() {
  Serial.begin(115200);
  pinMode(BUTTON_PIN, INPUT); 
  analogReadResolution(12);
  delay(300);

  Serial.printf("PSRAM Size: %d bytes\n", ESP.getPsramSize());
  Serial.printf("Free PSRAM: %d bytes\n", ESP.getFreePsram());

  audioBuffer = (float*)ps_malloc(BUFFER_SIZE * sizeof(float));
  preEmphasized = (float*)ps_malloc(BUFFER_SIZE * sizeof(float));

  if (!audioBuffer || !preEmphasized) {
    Serial.println("Gagal alokasi PSRAM!");
    while (true);
  }

  // Inisialisasi buffer audio
  memset(audioBuffer, 0, BUFFER_SIZE * sizeof(float));
  memset(preEmphasized, 0, BUFFER_SIZE * sizeof(float));

  WiFi.mode(WIFI_STA);
  if (esp_now_init() != ESP_OK) {
    Serial.println("ESP-NOW init gagal!");
    return;
  }

  esp_now_register_send_cb(onSent);

  esp_now_peer_info_t peerInfo = {};
  memcpy(peerInfo.peer_addr, receiverMac, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  if (esp_now_add_peer(&peerInfo) == ESP_OK) {
    Serial.println("Peer ESP-NOW ditambahkan.");
  } else {
    Serial.println("Gagal menambahkan peer ESP-NOW!");
  }

  Serial.println("Siap. Tekan tombol untuk mulai rekaman dan kirim LPC...");
}

void loop() {
  static bool lastButton = HIGH;
  bool currentButton = digitalRead(BUTTON_PIN);

  if (lastButton == HIGH && currentButton == LOW) {
    unsigned long currentTime = millis();
    if (currentTime - lastSendTime < MIN_TIME_BETWEEN_SENDS) {
      Serial.println(" Terlalu cepat! Tunggu sebentar...");
    } else if (!isRecording) {
      isRecording = true;
      Serial.println(" Rekaman dimulai...");
      delay(50); // Debounce
    }
  }
  lastButton = currentButton;

  if (isRecording) {
    float totalEnergy = 0.0;
    float avg = 0.0;

    memset(audioBuffer, 0, BUFFER_SIZE * sizeof(float));
    memset(preEmphasized, 0, BUFFER_SIZE * sizeof(float));

    Serial.println("Perekaman...");
    for (int i = 0; i < BUFFER_SIZE; i++) {
      int adc = analogRead(MIC_PIN);
      audioBuffer[i] = ((float)adc - 2048.0) / 1024.0;
      avg += audioBuffer[i];
      delayMicroseconds(125); 
    }

    avg /= BUFFER_SIZE;
    for (int i = 0; i < BUFFER_SIZE; i++) {
      audioBuffer[i] -= avg;
      totalEnergy += audioBuffer[i] * audioBuffer[i];
    }
    
    totalEnergy /= BUFFER_SIZE;
    if (totalEnergy < ENERGY_THRESHOLD) {
      Serial.printf("Audio terlalu lemah (energi: %f), rekaman dibatalkan\n", totalEnergy);
      isRecording = false;
      return;
    }

    pre_emphasis(audioBuffer, preEmphasized, BUFFER_SIZE, PRE_EMPHASIS_ALPHA);
    apply_hamming_window(preEmphasized, BUFFER_SIZE);
    
    compute_autocorrelation(preEmphasized, BUFFER_SIZE, LPC_ORDER, r);

    Serial.print("Energi (r[0]) = ");
    Serial.println(r[0], 6);

    bool lpcValid = levinson_durbin(r, a, LPC_ORDER);

    if (lpcValid) {
      Serial.println("Koefisien LPC:");
      for (int i = 1; i <= LPC_ORDER; i++) {
        Serial.printf("a[%d] = %f\n", i, a[i]);
        packet.lpc[i - 1] = a[i];
      }

      packet.energy = r[0];
      
      esp_err_t result = esp_now_send(receiverMac, (uint8_t*)&packet, sizeof(packet));
      
      if (result == ESP_OK) {
        Serial.println("Paket data dikirim ke transmisi");
        lastSendTime = millis(); // Update waktu kirim terakhir
      } else {
        Serial.println("Gagal mengirim data LPC");
      }
    } else {
      Serial.println("Perhitungan LPC tidak valid, pengiriman dibatalkan");
    }

    isRecording = false;
    Serial.println("Tekan tombol lagi untuk rekam ulang");
  }
  
  delay(10); 
}
