#include <WiFi.h>
#include <esp_now.h>

#define LPC_ORDER 8

typedef struct {
  float lpc[LPC_ORDER];
  float energy;
} LPCPacket;

LPCPacket incomingPacket;

void onReceive(const esp_now_recv_info_t *info, const uint8_t *data, int len) {
  if (len == sizeof(LPCPacket)) {
    memcpy(&incomingPacket, data, sizeof(incomingPacket));

    Serial.println(" Paket diterima:");
    Serial.printf(" Energi: %.6f\n", incomingPacket.energy);

    for (int i = 0; i < LPC_ORDER; i++) {
      Serial.printf("a[%d] = %.6f\n", i + 1, incomingPacket.lpc[i]);
    }

    Serial.println(" Selesai terima LPC\n");
  } else {
    Serial.print("⚠️ Data tidak valid. Panjang: ");
    Serial.println(len);
  }
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);

  if (esp_now_init() != ESP_OK) {
    Serial.println(" Gagal inisialisasi ESP-NOW");
    return;
  }

  esp_now_register_recv_cb(onReceive);
  Serial.println(" Siap menerima data LPC dari pengirim...");
}

void loop() {

}
