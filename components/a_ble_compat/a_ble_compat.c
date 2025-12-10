#include <string.h>
#include "esp_err.h"
#include "esp_gap_ble_api.h"

// Compatibility shim: provide legacy esp_ble_gap_start_advertising so code
// calling the old API can link. This stub currently performs no advertising
// and returns success. If you need real advertising/provisioning, replace
// this implementation to map fields into the ext-adv API properly.

esp_err_t esp_ble_gap_start_advertising(esp_ble_adv_params_t *adv_params)
{
    (void)adv_params;
    return ESP_OK;
}
