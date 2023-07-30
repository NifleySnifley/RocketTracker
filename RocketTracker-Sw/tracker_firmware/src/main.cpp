#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "Arduino.h"

#include "pindefs.h"

#include <NimBLEDevice.h>

#define TAG_BT "BLE"

static NimBLEServer* pServer;

class ServerCallbacks : public NimBLEServerCallbacks {
    void onConnect(NimBLEServer* pServer) {
        ESP_LOGI(TAG_BT, "Client connected");
        ESP_LOGI(TAG_BT, "Multi-connect support: start advertising");
        NimBLEDevice::startAdvertising();
    };
    /** Alternative onConnect() method to extract details of the connection.
     *  See: src/ble_gap.h for the details of the ble_gap_conn_desc struct.
     */
    void onConnect(NimBLEServer* pServer, ble_gap_conn_desc* desc) {
        Serial.print("Client address: ");
        ESP_LOGI(TAG_BT, "%s", NimBLEAddress(desc->peer_ota_addr).toString().c_str());
        /** We can use the connection handle here to ask for different connection parameters.
         *  Args: connection handle, min connection interval, max connection interval
         *  latency, supervision timeout.
         *  Units; Min/Max Intervals: 1.25 millisecond increments.
         *  Latency: number of intervals allowed to skip.
         *  Timeout: 10 millisecond increments, try for 5x interval time for best results.
         */
        pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
    };
    void onDisconnect(NimBLEServer* pServer) {
        ESP_LOGI(TAG_BT, "Client disconnected - start advertising");
        NimBLEDevice::startAdvertising();
    };
    void onMTUChange(uint16_t MTU, ble_gap_conn_desc* desc) {
        Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
    };

    /********************* Security handled here **********************
    ****** Note: these are the same return values as defaults ********/
    uint32_t onPassKeyRequest() {
        ESP_LOGI(TAG_BT, "Server Passkey Request");
        /** This should return a random 6 digit number for security
         *  or make your own static passkey as done here.
         */
        return 123456;
    };

    bool onConfirmPIN(uint32_t pass_key) {
        ESP_LOGI(TAG_BT, "Passkey: %d", pass_key);
        /** Return false if passkeys don't match. */
        return true;
    };

    void onAuthenticationComplete(ble_gap_conn_desc* desc) {
        /** Check that encryption was successful, if not we disconnect the client */
        if (!desc->sec_state.encrypted) {
            NimBLEDevice::getServer()->disconnect(desc->conn_handle);
            ESP_LOGI(TAG_BT, "Encrypt connection failed - disconnecting client");
            return;
        }
        ESP_LOGI(TAG_BT, "Starting BLE work!");
    };
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks {
    void onRead(NimBLECharacteristic* pCharacteristic) {
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onRead(), value: ");
        ESP_LOGI(TAG_BT, "%s", pCharacteristic->getValue().c_str());
    };

    void onWrite(NimBLECharacteristic* pCharacteristic) {
        Serial.print(pCharacteristic->getUUID().toString().c_str());
        Serial.print(": onWrite(), value: ");
        ESP_LOGI(TAG_BT, "%s", pCharacteristic->getValue().c_str());
    };
    /** Called before notification or indication is sent,
     *  the value can be changed here before sending if desired.
     */
    void onNotify(NimBLECharacteristic* pCharacteristic) {
        ESP_LOGI(TAG_BT, "Sending notification to clients");
    };


    /** The status returned in status is defined in NimBLECharacteristic.h.
     *  The value returned in code is the NimBLE host return code.
     */
    void onStatus(NimBLECharacteristic* pCharacteristic, Status status, int code) {
        String str = ("Notification/Indication status code: ");
        str += status;
        str += ", return code: ";
        str += code;
        str += ", ";
        str += NimBLEUtils::returnCodeToString(code);
        ESP_LOGI(TAG_BT, "%s", str.c_str());
    };

    void onSubscribe(NimBLECharacteristic* pCharacteristic, ble_gap_conn_desc* desc, uint16_t subValue) {
        String str = "Client ID: ";
        str += desc->conn_handle;
        str += " Address: ";
        str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
        if (subValue == 0) {
            str += " Unsubscribed to ";
        } else if (subValue == 1) {
            str += " Subscribed to notfications for ";
        } else if (subValue == 2) {
            str += " Subscribed to indications for ";
        } else if (subValue == 3) {
            str += " Subscribed to notifications and indications for ";
        }
        str += std::string(pCharacteristic->getUUID()).c_str();

        ESP_LOGI(TAG_BT, "%s", str.c_str());
    };
};

/** Handler class for descriptor actions */
class DescriptorCallbacks : public NimBLEDescriptorCallbacks {
    void onWrite(NimBLEDescriptor* pDescriptor) {
        std::string dscVal = pDescriptor->getValue();
        Serial.print("Descriptor witten value:");
        ESP_LOGI(TAG_BT, "%s", dscVal.c_str());
    };

    void onRead(NimBLEDescriptor* pDescriptor) {
        Serial.print(pDescriptor->getUUID().toString().c_str());
        ESP_LOGI(TAG_BT, " Descriptor read");
    };
};


/** Define callback instances globally to use for multiple Charateristics \ Descriptors */
static DescriptorCallbacks dscCallbacks;
static CharacteristicCallbacks chrCallbacks;

#define DBG_TAG "DEBUG"
#define GATTS_TAG "GATT"

// Bluetooth configuration
#define NUM_BLE_APPS 1
#define CONFIG_APP_ID 0

static NimBLECharacteristic* pLEDchar;

// Idle LED blinking...
void led_task(void* args) {
    while (1) {
        // gpio_set_level(TRACKER_LED_RED, 1);
        // gpio_set_level(TRACKER_LED_GRN, 1);
        // vTaskDelay(500);
        // gpio_set_level(TRACKER_LED_RED, 0);
        // gpio_set_level(TRACKER_LED_GRN, 1);
        // vTaskDelay(500);
        // gpio_set_level(TRACKER_LED_RED, 0);
        // gpio_set_level(TRACKER_LED_GRN, 0);
        // vTaskDelay(500);
        // gpio_set_level(TRACKER_LED_RED, 1);
        // gpio_set_level(TRACKER_LED_GRN, 0);
        // vTaskDelay(500);

        if (pLEDchar != NULL && pLEDchar->getValue().length() >= 2) {
            gpio_set_level(TRACKER_LED_RED, pLEDchar->getValue()[0] != '0');
            gpio_set_level(TRACKER_LED_GRN, pLEDchar->getValue()[1] != '0');
        }
        vTaskDelay(100);
        // ESP_LOGI(DBG_TAG, "Cycle!");
    }
}

extern "C" void app_main() {
    initArduino();

    gpio_reset_pin(TRACKER_LED_RED);
    gpio_reset_pin(TRACKER_LED_GRN);
    gpio_set_direction(TRACKER_LED_RED, GPIO_MODE_OUTPUT);
    gpio_set_direction(TRACKER_LED_GRN, GPIO_MODE_OUTPUT);

    esp_log_level_set("*", ESP_LOG_VERBOSE);

    xTaskCreate(led_task, "led_blinky_task", 1024 * 2, NULL, configMAX_PRIORITIES, NULL);

    ESP_LOGI(TAG_BT, "Starting NimBLE Server");

    /** sets device name */
    NimBLEDevice::init("ESP32 RocketTracker");

    /** Optional: set the transmit power, default is 3db */
    NimBLEDevice::setPower(ESP_PWR_LVL_P9); /** +9db */

    NimBLEDevice::setSecurityAuth(BLE_SM_PAIR_AUTHREQ_SC);

    pServer = NimBLEDevice::createServer();
    pServer->setCallbacks(new ServerCallbacks());

    NimBLEService* pLEDService = pServer->createService("FE01");
    pLEDchar = pLEDService->createCharacteristic(
        "FF01",
        NIMBLE_PROPERTY::READ |
        NIMBLE_PROPERTY::WRITE
    );

    pLEDchar->setValue("00");
    pLEDchar->setCallbacks(&chrCallbacks);

    /** 2904 descriptors are a special case, when createDescriptor is called with
     *  0x2904 a NimBLE2904 class is created with the correct properties and sizes.
     *  However we must cast the returned reference to the correct type as the method
     *  only returns a pointer to the base NimBLEDescriptor class.
     */
    NimBLE2904* pLED2904 = (NimBLE2904*)pLEDchar->createDescriptor("2904");
    pLED2904->setFormat(NimBLE2904::FORMAT_UTF8);
    pLED2904->setCallbacks(&dscCallbacks);


    /** Start the services when finished creating all Characteristics and Descriptors */
    pLEDService->start();

    NimBLEAdvertising* pAdvertising = NimBLEDevice::getAdvertising();
    /** Add the services to the advertisment data **/
    pAdvertising->addServiceUUID(pLEDService->getUUID());
    /** If your device is battery powered you may consider setting scan response
     *  to false as it will extend battery life at the expense of less data sent.
     */
    pAdvertising->setScanResponse(true);
    pAdvertising->start();

    ESP_LOGI(TAG_BT, "Advertising Started");

    while (1) {
        if (pServer->getConnectedCount()) {
            NimBLEService* pSvc = pServer->getServiceByUUID("BAAD");
            if (pSvc) {
                NimBLECharacteristic* pChr = pSvc->getCharacteristic("F00D");
                if (pChr) {
                    pChr->notify(true);
                }
            }
        }
        vTaskDelay(500);
    }
}