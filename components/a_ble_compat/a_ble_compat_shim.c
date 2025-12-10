#include "esp_err.h"
#include "esp_gap_ble_api.h"
#include <string.h>

// Compatibility shim for legacy esp_ble_gap_start_advertising API.
// Translates to extended advertising APIs so protocomm (or other legacy callers)
// can link and the project can build. This provides a minimal translation;
// for proper runtime behavior you may want to map fields from `adv_params`
// into the relevant ext-adv structures.
esp_err_t esp_ble_gap_start_advertising(esp_ble_adv_params_t *adv_params)
{
    esp_ble_gap_ext_adv_t ext_adv;
    memset(&ext_adv, 0, sizeof(ext_adv));

    // A minimal call using one advertising set. For many applications this
    // will be sufficient to satisfy probing/build; extend the mapping from
    // adv_params to ext_adv as needed for actual BLE behavior.
    (void)adv_params;
    return esp_ble_gap_ext_adv_start(1, &ext_adv);
}
