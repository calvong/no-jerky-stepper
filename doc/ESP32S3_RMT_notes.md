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

## Streaming with the simple encoder (IDF >= 5.3)

The first version of this library converted a whole precomputed curve to symbols, then created
and deleted a copy encoder for every 48-symbol chunk. That needed memory proportional to the
move length and could not change a move once it had started. (It also had a bug:
`increase_allocated_curve_memory_check()` realloc'ed a pointer passed by value, so the caller
kept writing into freed memory.)

The current driver (`src/platform/esp32/rmt_step_channel.cpp`) uses
`rmt_new_simple_encoder()`, as FastAccelStepper's RMT v2 driver does:

- One `rmt_transmit()` per motion, with a dummy 1-byte payload. The driver calls our callback
  `size_t cb(data, size, symbols_written, symbols_free, symbols, &done, arg)` from its ISR
  whenever channel memory frees up (ping-pong halves of 24 symbols on the S3). The callback
  pulls step commands from a lock-free queue and writes symbols directly into RMT memory.
- The callback must write at least one symbol whenever `symbols_free >= 1` unless it sets
  `done`. Then the driver never uses its overflow buffer, and encoding is at most
  `mem_block_symbols` (48) ahead of playback. The DIR-change drain relies on this bound.
- Returning 0 with `*done = true` ends the transaction; the driver appends the end marker.
  `on_trans_done` then tells the planner task, which can start the next transaction.
- Symbol durations must be 1..32767 ticks: 0 is the end marker. Long low times are split
  into low-only symbols of up to 65534 ticks.
- `rmt_disable()` aborts a running transaction (used for immediate stop). `rmt_enable()`
  makes the channel usable again.
- Sync manager: `rmt_new_sync_manager()` needs all member channels enabled. Before each
  synchronous start, call `rmt_sync_reset()` and then `rmt_transmit()` on every member; the
  hardware starts them together.
