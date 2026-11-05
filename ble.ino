// ======================================================================
// BLUETOOTH (BLE)
//   ESP32 -> página:
//     'N' (13 bytes, 1/s): signos, NEWS2, alarma, simulación y silencio restante
//     'E' (19 bytes): 9 puntos de ECG  (int16, µV)
//     'P' (19 bytes): 9 puntos de PPG  (int16, 10 µV)
//     'L' (6 bytes, cada 2 s): límites en uso
//   Página -> ESP32 (característica de comandos):
//     'L' fcMax fcMin spo2Min pasMax pasMin   -> límites
//     'S' 1/0                                 -> silenciar / reactivar
//     'C' sbp dbp                             -> calibrar la PA
//     'D' 0/1/2                               -> simulación NEWS2: apagada / medio / alto
// ======================================================================

#define USAR_BLE 1     // 0 = sin Bluetooth

#if USAR_BLE

#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLE2902.h>

#define BLE_NOMBRE        "Monitor-MTSS"
#define BLE_UUID_SERVICIO "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define BLE_UUID_DATOS    "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define BLE_UUID_COMANDOS "beb5483f-36e1-4688-b7f5-ea07361b26a8"
#define BLE_PUNTOS        9
#define BLE_ESCALA_ECG    1.0e6f
#define BLE_ESCALA_PPG    1.0e5f

struct PuntoBle { char tipo; int16_t v; };

QueueHandle_t      colaBle      = NULL;
BLECharacteristic* bleCaract    = NULL;
volatile bool      bleConectado = false;

// Órdenes recibidas (se aplican en el loop, no en el callback)
volatile bool    cmdLimPend = false, cmdSilPend = false, cmdCalPend = false, cmdDemoPend = false;
volatile uint8_t cmdLim[5];
volatile uint8_t cmdSil  = 0;
volatile uint8_t cmdCal[2];
volatile uint8_t cmdDemo = 0;

class BleCallbacks : public BLEServerCallbacks {
    void onConnect(BLEServer* s) override    { bleConectado = true;  }
    void onDisconnect(BLEServer* s) override { bleConectado = false; }
};

class ComandosCallbacks : public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic* c) override {
        auto v = c->getValue();
        size_t n = v.length();
        if (n < 1) return;
        uint8_t tipo = (uint8_t)v[0];
        if (tipo == 'L' && n >= 6) {
            for (int i = 0; i < 5; i++) cmdLim[i] = (uint8_t)v[1 + i];
            cmdLimPend = true;
        } else if (tipo == 'S' && n >= 2) {
            cmdSil = (uint8_t)v[1];
            cmdSilPend = true;
        } else if (tipo == 'C' && n >= 3) {
            cmdCal[0] = (uint8_t)v[1];
            cmdCal[1] = (uint8_t)v[2];
            cmdCalPend = true;
        } else if (tipo == 'D' && n >= 2) {
            cmdDemo = (uint8_t)v[1];
            cmdDemoPend = true;
        }
    }
};

// Se llama desde el loop: aplica las órdenes de la página
void bleProcesarComandos() {
    if (cmdLimPend) {
        cmdLimPend = false;
        Limites L = { cmdLim[0], cmdLim[1], cmdLim[2], cmdLim[3], cmdLim[4] };
        if (limitesValidos(L)) { lim = L; guardarLimites(); }
    }
    if (cmdSilPend) {
        cmdSilPend = false;
        if (cmdSil) {
            uint32_t fin = millis() + SILENCIO_MS;
            silencioHastaMs = (fin == 0) ? 1 : fin;
        } else {
            silencioHastaMs = 0;
        }
    }
    if (cmdCalPend) {
        cmdCalPend = false;
        recibirReferencia(cmdCal[0], cmdCal[1]);
    }
    if (cmdDemoPend) {
        cmdDemoPend = false;
        if (cmdDemo == 1 || cmdDemo == 2) {
            modoDemo = cmdDemo;
            demoHastaMs = millis() + DEMO_MS;
        } else {
            modoDemo = 0;
        }
    }
}

void bleAgregarPunto(char tipo, float voltios) {
    if (colaBle == NULL || !bleConectado) return;
    float x = voltios * ((tipo == 'E') ? BLE_ESCALA_ECG : BLE_ESCALA_PPG);
    if (x >  32767.0f) x =  32767.0f;
    if (x < -32768.0f) x = -32768.0f;
    PuntoBle p = { tipo, (int16_t)lroundf(x) };
    xQueueSend(colaBle, &p, 0);
}

void bleNotificar(uint8_t* datos, size_t n) {
    if (!bleConectado || bleCaract == NULL) return;
    bleCaract->setValue(datos, n);
    bleCaract->notify();
}

void bleEnviarOnda(char tipo, int16_t* buf) {
    uint8_t pkt[1 + 2 * BLE_PUNTOS];
    pkt[0] = (uint8_t)tipo;
    for (int i = 0; i < BLE_PUNTOS; i++) {
        pkt[1 + 2 * i] = (uint8_t)(buf[i] & 0xFF);
        pkt[2 + 2 * i] = (uint8_t)((buf[i] >> 8) & 0xFF);
    }
    bleNotificar(pkt, sizeof(pkt));
}

uint8_t bleByte(int x) { return (uint8_t)constrain(x, 0, 255); }

void bleEnviarNumeros() {
    uint8_t vis = (nivelActual == NIVEL_ALTO)  ? 2 :
                  (nivelActual == NIVEL_MEDIO) ? 1 :
                  alarmaTecnica                ? 3 : 0;
    int fr = frMostrar;

    uint8_t silRest = 0;
    uint32_t fin = silencioHastaMs;
    if (fin != 0) {
        int32_t r = (int32_t)(fin - millis());
        if (r > 0) silRest = (uint8_t)min((int32_t)255, (r + 999) / 1000);
    }

    uint8_t pkt[13];
    pkt[0]  = 'N';
    pkt[1]  = bleByte(fcEstable);
    pkt[2]  = bleByte(prEstable);
    pkt[3]  = bleByte(spo2Actual);
    pkt[4]  = bleByte(fr > 0 ? fr : 0);
    pkt[5]  = bleByte(pasActual > 0 ? pasActual : 0);
    pkt[6]  = bleByte(pasActual > 0 ? dbpActual : 0);
    pkt[7]  = 0;
    pkt[8]  = 0;
    pkt[9]  = news2Total;
    pkt[10] = vis;
    pkt[11] = ((spo2Actual > 0) ? 2 : 0) | ((modoDemo != 0) ? 0x80 : 0);   // bit 7 = simulación activa
    pkt[12] = silRest;
    bleNotificar(pkt, sizeof(pkt));
}

void bleEnviarLimites() {
    uint8_t pkt[6] = { 'L', lim.fcMax, lim.fcMin, lim.spo2Min, lim.pasMax, lim.pasMin };
    bleNotificar(pkt, sizeof(pkt));
}

void tareaBle(void* arg) {
    int16_t bufE[BLE_PUNTOS], bufP[BLE_PUNTOS];
    uint8_t nE = 0, nP = 0;
    bool conectadoAntes = false;
    uint32_t tNum = millis();
    uint32_t tLim = millis();
    PuntoBle p;

    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(20));

        while (xQueueReceive(colaBle, &p, 0) == pdTRUE) {
            if (p.tipo == 'E') {
                bufE[nE++] = p.v;
                if (nE >= BLE_PUNTOS) { bleEnviarOnda('E', bufE); nE = 0; }
            } else {
                bufP[nP++] = p.v;
                if (nP >= BLE_PUNTOS) { bleEnviarOnda('P', bufP); nP = 0; }
            }
        }

        if (millis() - tNum >= 1000) { tNum += 1000; bleEnviarNumeros(); }
        if (millis() - tLim >= 2000) { tLim += 2000; bleEnviarLimites(); }

        if (!bleConectado && conectadoAntes) {
            vTaskDelay(pdMS_TO_TICKS(300));
            BLEDevice::startAdvertising();
            nE = nP = 0;
        }
        conectadoAntes = bleConectado;
    }
}

void bleIniciar() {
    BLEDevice::init(BLE_NOMBRE);
    BLEServer* servidor = BLEDevice::createServer();
    servidor->setCallbacks(new BleCallbacks());

    BLEService* servicio = servidor->createService(BLE_UUID_SERVICIO);

    bleCaract = servicio->createCharacteristic(
        BLE_UUID_DATOS,
        BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
    bleCaract->addDescriptor(new BLE2902());

    BLECharacteristic* comandos = servicio->createCharacteristic(
        BLE_UUID_COMANDOS,
        BLECharacteristic::PROPERTY_WRITE | BLECharacteristic::PROPERTY_WRITE_NR);
    comandos->setCallbacks(new ComandosCallbacks());

    servicio->start();

    BLEAdvertising* adv = BLEDevice::getAdvertising();
    BLEAdvertisementData datosAnuncio;
    datosAnuncio.setFlags(0x06);
    datosAnuncio.setName(BLE_NOMBRE);
    adv->setAdvertisementData(datosAnuncio);
    BLEAdvertisementData datosRespuesta;
    datosRespuesta.setCompleteServices(BLEUUID(BLE_UUID_SERVICIO));
    adv->setScanResponseData(datosRespuesta);
    adv->setScanResponse(true);
    BLEDevice::startAdvertising();

    colaBle = xQueueCreate(128, sizeof(PuntoBle));
    xTaskCreatePinnedToCore(tareaBle, "ble", 4096, NULL, 1, NULL, 0);
}

#else

void bleIniciar() {}
void bleAgregarPunto(char tipo, float voltios) {}
void bleProcesarComandos() {}

#endif