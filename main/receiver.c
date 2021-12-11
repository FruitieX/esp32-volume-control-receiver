#include <esp_now.h>

typedef struct struct_message
{
    long value;
} struct_message;

struct_message myData;

void OnDataRecv(const uint8_t *mac, const uint8_t *incomingData, int len)
{
    memcpy(&myData, incomingData, sizeof(myData));

    printf("Bytes received: %d\n", len);
    printf("Value: %d\n", myData.value);
}

void app_main(void)
{
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    if (esp_now_init() != ESP_OK)
    {
        printf("Error initializing ESP-NOW\n");
        return;
    }

    esp_now_register_recv_cb(OnDataRecv);
}
