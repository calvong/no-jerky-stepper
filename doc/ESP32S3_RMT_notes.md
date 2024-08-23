# ESP32-S3 Remote Control Transceiver (RMT) Notes

This is a collection of notes on the usage of the Espressif RMT periphereal.

## Reference
ESP32-S3 RMT API documentations > [link](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/peripherals/rmt.html#_CPPv412rmt_transmit20rmt_channel_handle_t20rmt_encoder_handle_tPKv6size_tPK21rmt_transmit_config_t)

## Notes of RMT TX

### Transimt workflow
Before the transmission, a new RMT encoder needs to be created to encode the date to be sent. A custom encoder function can be created serving the following purposes:
1. encode/convert user data into the [`rmt_symbol_word_t`](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/peripherals/rmt.html#_CPPv417rmt_symbol_word_t) format
2. pass the custom `encode`, `del`, `reset` functions to the *base* `rmt_encoder_t`
3. also pass some user data to the custom encoding function (lowest level) 


Then, [`rmt_transmit()`](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/peripherals/rmt.html#_CPPv412rmt_transmit20rmt_channel_handle_t20rmt_encoder_handle_tPKv6size_tPK21rmt_transmit_config_t) is called to transimt the data. It has the following parameters: `[rmt_channel_handle_t tx_channel, rmt_encoder_handle_t encoder, const void *payload, size_t payload_bytes, const rmt_transmit_config_t *config]`

At the lowest level, a custom data encoding function is created. It takes `[rmt_encoder_t *encoder, rmt_channel_handle_t channel, const void *primary_data, size_t data_size, rmt_encode_state_t *ret_state]` as parameters. The `const void *payload` from [`rmt_transmit()`](https://docs.espressif.com/projects/esp-idf/en/v5.3/esp32s3/api-reference/peripherals/rmt.html#_CPPv412rmt_transmit20rmt_channel_handle_t20rmt_encoder_handle_tPKv6size_tPK21rmt_transmit_config_t) can be passed to `const void *primary_data` here.
