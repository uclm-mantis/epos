# EPOS-CAN

EPOS-CAN is a lightweight CANopen library for Espressif microcontrollers using ESP-IDF. It started as a helper library for Maxon EPOS drives, but the CANopen core is now generic and can be reused with other CANopen devices. The project currently provides:

- A generic CANopen core on top of ESP-IDF TWAI.
- Client-side SDO, NMT and PDO helpers.
- An asynchronous handler mechanism for arbitrary COB-IDs.
- A synchronous waiting API for specific CANopen frames.
- A generated client API from an object dictionary.
- A generated SDO server API from a server-side object dictionary.
- A serial/TCP console inspired by CiA 309-3.

Although the project is still evolving, it is already usable for real CANopen applications on ESP32-class devices.

## Hardware requirements

| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

Typical setup:

- Espressif ESP32 development board.
- SN65HVD230 or equivalent CAN transceiver.
- A CANopen device such as a Maxon EPOS controller, or any custom CANopen node.

## Installation

### ESP-IDF

#### Method 1: Managed Component

Add the component to `main/idf_component.yml`:

```yaml
dependencies:
  epos:
    git: https://github.com/uclm-mantis/epos.git
```

Then build:

```bash
idf.py build
```

#### Method 2: Git submodule

```bash
cd your-project
git submodule add https://github.com/uclm-mantis/epos.git components/epos
git submodule update --init --recursive
```

### Arduino IDE

#### Method 1: Install from ZIP (Recommended)

1. Download the repository as a ZIP file:
   - Go to [https://github.com/uclm-mantis/epos](https://github.com/uclm-mantis/epos)
   - Click the green **Code** button
   - Select **Download ZIP**

2. In Arduino IDE, go to **Sketch → Include Library → Add .ZIP Library...**

3. Select the downloaded `epos-main.zip` file

4. The library will be installed and appear in **Sketch → Include Library → EposCAN**

5. Include in your sketch:

```cpp
#include <EposCAN.h>
```

#### Method 2: Manual Installation

1. Copy this repository to your Arduino libraries directory:
   - **Windows**: `Documents\Arduino\libraries\EposCAN\`
   - **macOS**: `~/Documents/Arduino/libraries/EposCAN/`
   - **Linux**: `~/Arduino/libraries/EposCAN/`

2. Restart Arduino IDE

3. The library will appear in **Sketch → Include Library → EposCAN**


## Quick start

The CANopen core lives in `canopen.h` / `canopen.c`.

```c
#include "canopen.h"

void app_main(void)
{
    canopen_init_cfg_t cfg = canopen_init_default();
    cfg.enable_dump_msg = true;

    ESP_ERROR_CHECK(canopen_initialize(&cfg));

    // Example: reset node 2 and place it in operational state
    ESP_ERROR_CHECK(nmt_reset_node(2));
    vTaskDelay(pdMS_TO_TICKS(1000));
    ESP_ERROR_CHECK(nmt_start_remote_node(2));
}
```

## Project structure

- `canopen.h`, `canopen.c`: generic CANopen core.
- `canopen_client.h`, `canopen_client.c`: generated client-side API from `client_od.h`.
- `canopen_server.h`, `canopen_server.c`: generated SDO server-side API from `server_od.h`.
- `canopen_console.h`, `canopen_console.c`: interactive console.
- `canopen_types.h`: low-level CANopen frame layouts and constants.
- `client_od.h`: list of client-side object dictionaries to include.
- `server_od.h`: list of server-side object dictionaries to include.
- `epos_types.h`: optional typed views for Maxon EPOS objects.

## CANopen core API

### Initialization and shutdown

```c
static inline canopen_init_cfg_t canopen_init_default(void);
esp_err_t canopen_initialize(const canopen_init_cfg_t *cfg);
void canopen_request_shutdown(void);
esp_err_t canopen_wait_shutdown(void);
```

`canopen_init_cfg_t` contains:

```c
typedef struct {
    int can_tx_pin;
    int can_rx_pin;
    unsigned max_delay_ms;
    bool enable_dump_msg;
} canopen_init_cfg_t;
```

Notes:

- `canopen_initialize()` installs and starts the TWAI driver, creates the RX/TX/dispatch tasks and initializes the internal queues and mutexes.
- `canopen_request_shutdown()` asks the running CANopen tasks to stop.
- `canopen_wait_shutdown()` blocks until shutdown is processed, stops TWAI and releases internal resources.

### Core send functions

```c
esp_err_t canopen_send(const twai_message_t *msg);
esp_err_t canopen_post(const twai_message_t *msg);
```

- `canopen_send()` is synchronous with respect to enqueuing and transmit task acknowledgement.
- `canopen_post()` only enqueues the frame for transmission and returns immediately.

Use `canopen_post()` when you are implementing asynchronous services, for example inside the SDO server.

### Timeouts and tracing

```c
TickType_t canopen_get_max_delay_ms(void);
void canopen_max_delay_ms(unsigned delay);

bool canopen_is_dump_enabled(void);
void canopen_dump_enabled(bool enable);
```

- `canopen_max_delay_ms()` sets the timeout used by synchronous operations such as SDO transfers and `canopen_wait_until()`.
- `canopen_dump_enabled()` enables frame tracing through the ESP log.

## NMT API

```c
esp_err_t nmt(uint8_t cs, uint8_t n);
static inline esp_err_t nmt_enter_preoperational(uint8_t node);
static inline esp_err_t nmt_reset_communication(uint8_t node);
static inline esp_err_t nmt_reset_node(uint8_t node);
static inline esp_err_t nmt_start_remote_node(uint8_t node);
static inline esp_err_t nmt_stop_remote_node(uint8_t node);
```

Example:

```c
ESP_ERROR_CHECK(nmt_reset_node(2));
vTaskDelay(pdMS_TO_TICKS(1000));
ESP_ERROR_CHECK(nmt_start_remote_node(2));
```

## SDO client API

### Low-level functions

```c
esp_err_t sdo_download(uint32_t cob_id,
                       uint16_t index,
                       uint8_t subindex,
                       void *value,
                       size_t size);

esp_err_t sdo_upload(uint32_t cob_id,
                     uint16_t index,
                     uint8_t subindex,
                     void *ret);
```

These functions support expedited transfers and segmented transfers.

Example:

```c
uint8_t node = 2;
uint32_t sdo = 0x600 + node;

uint16_t controlword = 0x000F;
ESP_ERROR_CHECK(sdo_download(sdo, 0x6040, 0x00, &controlword, sizeof(controlword)));

uint16_t statusword;
ESP_ERROR_CHECK(sdo_upload(sdo, 0x6041, 0x00, &statusword));
```

### Generated client API

`canopen_client.h` generates typed getters and setters from the object dictionaries included by `client_od.h`. The generated functions are thin wrappers over `sdo_upload()` and `sdo_download()`.

Typical usage:

```c
#include "canopen_client.h"

ESP_ERROR_CHECK(set_controlword(2, 0));
ESP_ERROR_CHECK(set_controlword(2, 6));
ESP_ERROR_CHECK(set_controlword(2, 7));
ESP_ERROR_CHECK(set_controlword(2, 0x0F));

int16_t current;
ESP_ERROR_CHECK(get_current_actual_value(2, &current));
```

For Maxon EPOS objects, `epos_types.h` provides typed bitfield views such as `ControlWord_t`, `StatusWord_t` and `NMT_HB_state_t`.

Example:

```c
ControlWord_t cw = {
    .enable_voltage = 1,
    .quickstop = 1,
    .switch_on = 1,
};
ESP_ERROR_CHECK(set_controlword(2, cw.value));
```

## Asynchronous CANopen reception

The core provides two complementary mechanisms for receiving asynchronous traffic such as TPDOs, heartbeats or EMCY.

### Synchronous wait for a specific COB-ID

```c
esp_err_t canopen_wait_until(uint32_t cobid, void *ret);
```

This function blocks until a frame with the requested COB-ID arrives, or until timeout. When `ret` is not `NULL`, the first 8 data bytes are copied into the provided buffer. The implementation registers a temporary handler internally and removes it on completion or timeout.

Example:

```c
typedef struct __attribute__((packed)) {
    int32_t velocity;
    int32_t position;
} pdo1_t;

pdo1_t pdo;
ESP_ERROR_CHECK(canopen_wait_until(0x180 + 2, &pdo));
```

### Callback-based handlers

```c
typedef struct canopen_handler_entry *canopen_handler_handle_t;

typedef void (*canopen_handler_fn)(uint32_t cobid, void *data, void *context);

esp_err_t canopen_register_handler(uint32_t cobid,
                                   canopen_handler_fn handler_fn,
                                   void *context,
                                   canopen_handler_handle_t *out_handle);

esp_err_t canopen_unregister_handler(canopen_handler_handle_t handle);
```

Use this API when a task must react to multiple asynchronous COB-IDs without dedicating one waiting task per identifier.

Example:

```c
static void heartbeat_handler(uint32_t cobid, void *data, void *context)
{
    NMT_HB_state_t *state = (NMT_HB_state_t *)data;
    uint8_t *last = (uint8_t *)context;
    *last = *state;
}

uint8_t last_state = 0;
canopen_handler_handle_t hb_handle;
ESP_ERROR_CHECK(canopen_register_handler(0x700 + 2,
                                         heartbeat_handler,
                                         &last_state,
                                         &hb_handle));
```

## PDO helper API

The library now includes a higher-level PDO API for common client-side operations. The implementation is in `canopen.c`, and the associated declarations are expected in `canopen.h` for applications using the new helpers. The functionality currently implemented includes configuration, transmission, subscription and payload packing.

### PDO configuration model

A PDO is described by:

- Node ID.
- Direction: RX or TX.
- PDO number: 1 to 4.
- Optional COB-ID override.
- Transmission type.
- Optional inhibit time for TPDO.
- Mapping table encoded as standard 32-bit mapping entries.

The code uses the standard default COB-ID bases:

- TPDO1..4: `0x180`, `0x280`, `0x380`, `0x480` + node ID.
- RPDO1..4: `0x200`, `0x300`, `0x400`, `0x500` + node ID.

Communication and mapping parameter indices are derived automatically:

- TPDO communication: `0x1800`..`0x1803`
- RPDO communication: `0x1400`..`0x1403`
- TPDO mapping: `0x1A00`..`0x1A03`
- RPDO mapping: `0x1600`..`0x1603`

### `canopen_pdo_configure()`

```c
esp_err_t canopen_pdo_configure(uint8_t node,
                                canopen_pdo_dir_t dir,
                                uint8_t pdo_num,
                                const canopen_pdo_cfg_t *cfg);
```

Behaviour:

1. Validates that `pdo_num` is in the range 1..4.
2. Validates the mapping array:
   - at most 8 mapped entries,
   - no zero-sized entries,
   - total payload not larger than 64 bits.
3. Disables the PDO by setting the valid bit in the COB-ID.
4. Clears the mapping count.
5. Writes each mapping entry.
6. Restores the mapping count.
7. Programs transmission type and, for TPDO, inhibit time.
8. Re-enables the PDO.

This is the recommended way to configure RPDO/TPDO mappings from application code.

### `canopen_pdo_send()`

```c
esp_err_t canopen_pdo_send(uint8_t node,
                           uint8_t rpdo_num,
                           uint32_t cob_id_override,
                           const void *data,
                           size_t len);
```

Sends one RPDO frame.

- `rpdo_num` must be in 1..4.
- `len` must be at most 8.
- If `cob_id_override == 0`, the default RPDO COB-ID is used.

Example:

```c
uint8_t data[2] = { 0x34, 0x12 };
ESP_ERROR_CHECK(canopen_pdo_send(2, 1, 0, data, sizeof(data)));
```

### `canopen_pdo_subscribe()`

```c
esp_err_t canopen_pdo_subscribe(uint8_t node,
                                uint8_t tpdo_num,
                                uint32_t cob_id_override,
                                canopen_handler_fn fn,
                                void *context,
                                canopen_handler_handle_t *out);
```

Registers a handler for one TPDO using the default or overridden COB-ID.

Example:

```c
static void tpdo1_handler(uint32_t cobid, void *data, void *context)
{
    (void)cobid;
    memcpy(context, data, 8);
}

uint8_t latest_pdo[8];
canopen_handler_handle_t tpdo_handle;
ESP_ERROR_CHECK(canopen_pdo_subscribe(2, 1, 0,
                                      tpdo1_handler,
                                      latest_pdo,
                                      &tpdo_handle));
```

### PDO payload builder helpers

```c
void canopen_pdo_payload_clear(canopen_pdo_payload_t *p);
esp_err_t canopen_pdo_payload_put_u8(canopen_pdo_payload_t *p, uint8_t v);
esp_err_t canopen_pdo_payload_put_u16(canopen_pdo_payload_t *p, uint16_t v);
esp_err_t canopen_pdo_payload_put_u32(canopen_pdo_payload_t *p, uint32_t v);
esp_err_t canopen_pdo_payload_put_i32(canopen_pdo_payload_t *p, int32_t v);
```

These helpers append little-endian scalar values to a `canopen_pdo_payload_t` buffer and prevent overflow beyond 8 bytes.

Example:

```c
canopen_pdo_payload_t p;
canopen_pdo_payload_clear(&p);
ESP_ERROR_CHECK(canopen_pdo_payload_put_u16(&p, 0x1234));
ESP_ERROR_CHECK(canopen_pdo_payload_put_i32(&p, -25));
ESP_ERROR_CHECK(canopen_pdo_send(2, 1, 0, p.data, p.len));
```

### Generated PDO mapping enums

`canopen_client.h` also generates enums with standard mapping values for objects flagged as PDO-mappable in `client_od.h`.

- `pdo_tx_mapping_object_t`
- `pdo_rx_mapping_object_t`

Each enum value encodes the standard CANopen mapping word:

```c
(index << 16) | (subindex << 8) | (sizeof(type) << 3)
```

This lets you configure PDO mappings without hardcoding magic numbers.

## SDO server API

The server-side support is implemented in `canopen_server.h` / `canopen_server.c`. It provides a generated object dictionary with typed getter/setter callbacks and a multi-instance SDO server bound to node IDs.

### Server object dictionary callbacks

The user writes typed callbacks referenced from `server_od.h`:

```c
canopen_od_status_t on_get_my_value(uint32_t *value);
canopen_od_status_t on_set_my_value(uint32_t value);
```

The generated wrappers adapt those typed callbacks to the generic dispatcher API.

### Server-side OD status values

```c
typedef enum {
    CANOPEN_OD_OK = 0,
    CANOPEN_OD_UNSUPPORTED_ACCESS,
    CANOPEN_OD_WRITEONLY,
    CANOPEN_OD_READONLY,
    CANOPEN_OD_NO_SUCH_OBJECT,
    CANOPEN_OD_NO_SUCH_SUBINDEX,
    CANOPEN_OD_TYPE_MISMATCH,
    CANOPEN_OD_INVALID_VALUE,
    CANOPEN_OD_DATA_TOO_LONG,
    CANOPEN_OD_DATA_TOO_SHORT,
    CANOPEN_OD_HW_ERROR,
} canopen_od_status_t;
```

These are converted to CiA 301 SDO abort codes by:

```c
uint32_t canopen_od_status_to_abort_code(canopen_od_status_t st);
```

### Lifecycle

```c
esp_err_t canopen_server_start(uint8_t node_id);
esp_err_t canopen_server_stop(uint8_t node_id);
```

- `canopen_server_start()` registers the SDO request handler for `0x600 + node_id` and responds through `0x580 + node_id`.
- `canopen_server_stop()` unregisters the handler and releases the instance slot.

Up to 8 concurrent server instances are currently supported.

### Direct OD access

```c
canopen_od_status_t canopen_server_od_get(uint16_t index,
                                          uint8_t subindex,
                                          void *value,
                                          size_t size);

canopen_od_status_t canopen_server_od_set(uint16_t index,
                                          uint8_t subindex,
                                          const void *value,
                                          size_t size);

const canopen_server_od_entry_t *canopen_server_find(uint16_t index,
                                                     uint8_t subindex);
```

These functions are useful both inside and outside the SDO protocol.

### Explicit dispatcher

```c
esp_err_t canopen_server_dispatch_sdo(uint32_t cobid_req,
                                      const void *req,
                                      size_t req_size,
                                      uint32_t *cobid_resp,
                                      void *resp,
                                      size_t *resp_size);
```

This function is reserved for future decoupling of the SDO server from the CAN handler registration. In the current implementation it returns `ESP_ERR_NOT_SUPPORTED`, because stateful transfers are handled by registered server instances instead.

### Protocol coverage

The current server implementation supports:

- SDO initiate download, expedited and segmented.
- SDO download segments.
- SDO initiate upload, expedited and segmented.
- SDO upload segments.

SDO block upload/download is not yet implemented.

## Console API

The console implementation lives in `canopen_console.h` / `canopen_console.c`. It uses the generated client object dictionary and exposes an interactive CiA 309-style interface.

### Initialization

```c
void canopen_console_register_commands(void);
void canopen_console_init(const canopen_console_cfg_t *cfg);
```

Configuration:

```c
typedef struct {
    int rx_buffer_size;
    int tx_buffer_size;
    bool dumb_mode;
    bool enable_usb_console;
    bool enable_tcp_console;
} canopen_console_cfg_t;
```

Default configuration enables the USB console and disables the TCP console. The TCP console listens on port 3344.

### Supported commands

The console provides, among others:

```text
[[net] node] r[ead] <index> <subindex> [<datatype>]
[[net] node] r[ead] <symbol> [<datatype>]
[[net] node] w[rite] <index> <subindex> <datatype> <value>
[[net] node] w[rite] <symbol> <value>
[[net] node] start
[[net] node] stop
[[net] node] preop[erational]
[[net] node] reset node|comm[unication]
[net] set network|node|sdo_timeout|sdo_block|dump <value>
[net] get network|node|sdo_timeout|sdo_block|dump
about [datatype|object|<symbol prefix>]
exit
```

Notes:

- Symbolic object access comes from the generated object dictionary.
- Completion and history are provided by `linenoise`.
- Because of argument parsing rules, negative numeric literals in some cases need the `--` separator, for example:

```text
w modes_of_operation -- -3
```

Unlike CANopenNode or similar CANopen implementations, we do not aim at producing a shadow copy of the object dictionary in the controller. You may, nonetheless, replicate as much as you want of the object dictionary using a somewhat standard callback mechanism.

Once started, EPOS are usually in *stopped* state, probably with some active faults.  The usual sequence to reset an EPOS2 to an operational state is:

```
set node 2
```

Sets target of the following commands to *id* number 2.  This is the identifier set in the dip switch of the EPOS.

```
> reset node
```

This triggers a transition to the *Initialisation* state. After a while the EPOS transitions automatically to a *Preoperational* state. In this precise moment an NMT booutup message is received.  You may see the trace in the console if tracing of events is active (it is active by default).

```
I (2624560) EPOS: Skip 00000702 (2 bytes) 00 00 00 00 00 00 00 00
```

COB-ID 0x702 signals an NMT heartbeat. Payload with a 0 means the EPOS is in *bootup* state. From now on the EPOS is able to handle SDO requests. Let us activate the device writing into the *controlword* object (`index=0x6040, subindex=0`).

```
> w controlword 0
```

In a fully compliant CiA console you would write `w 0x6040 0 u16 0`. This is not needed in EPOS-CAN because the console already knows the object dictionary of the EPOS device. Of course you may use the standard syntax and you may also use the help feature to find the right index, subindex and type of an object:

```
> help contr
Objects starting with contr:
  2220 00 u16 RW controller_structure
  6040 00 u16 RW controlword
```

Now let us complete the activation sequence:

```
> w controlword 6
> w controlword 7
> w controlword 0xf
```

Note that you may us the cursor *up* key to recall previous command and then edit that command to issue the new one.

If everything is right your EPOS will have a steady green led signaling it is ready.

From now on you will receive traces of PDO notifications from the EPOS.

Now you may issue the commands needed to operate the motor. For example, let us configure the motor in current mode:

```
> w modes_of_operation -- -3
```

Note the `--` before the `-3`. This is the only known departure from CiA 309. It is needed to communicate that no user options are added after that point. Therefore `-3` should not be interpreted as a flag but as a literal. You may also use `0xfd` if you want fully conformant interaction.

Now we may set the desired current:

```
> w current_mode_setting_value 500
```

And see the actual current value:

```
> r current_actual_value
```

And finally stop it:

```
> w current_mode_setting_value 0
```

### Minimal console setup

```c
#include "canopen.h"
#include "canopen_console.h"

void app_main(void)
{
    ESP_ERROR_CHECK(canopen_initialize(NULL));
    canopen_console_register_commands();
    canopen_console_init(NULL);
}
```

## Building APIs from EDS / OD files

The project can be used beyond Maxon EPOS devices. CANopen Electronic Data Sheet (EDS files, CiA 306-1) provide a full description of the capabilities of a device. You may use `eds_to_od.py` python script to extract the information from the EDS into a simple object dictionary table. Then, you may use `extract.py` python script to automatically generate an API customized for your device. 

- `client_od.h` is intended to aggregate one or more client-side object dictionary headers.
- `server_od.h` is intended to aggregate one or more server-side object dictionary headers.
- The generated macros in `canopen_client.h` and `canopen_server.h` turn those OD entries into typed client and server APIs.

This makes it practical to generate support for custom CANopen devices from EDS-derived tables.



## Acknowledgements

This project is partially funded by EU (Erasmus+ 2022-1-ES01- KA220-HED-000089155), JCCM and FEDER (SBPLY/21/180501/000238) and by NATO (SPS.MYP.G6001) research grants.

Developed by UCLM [Mantis Research Group](https://uclm-mantis.github.io/). 

Contact: [mantis@on.uclm.es](mailto:mantis@on.uclm.es).

### Contributors

* Fernando Castillo García
* Antonio González Rodríguez
* Sergio Juárez Pérez
* Andrea Martín Parra
* David Rodríguez Rosa
* Raúl Cuadros Tardío
* Juan Sánchez Medina
* Francisco Moya Fernández

## Reference

* [CANopen CiA 301 (login required)](https://www.can-cia.org/cia-groups/technical-documents)
* [ESP32](https://www.espressif.com/sites/default/files/documentation/esp32_datasheet_en.pdf)
* [SN65HVD230](https://www.ti.com/lit/ds/symlink/sn65hvd230.pdf)
* [EPOS2 Firmware Specification](https://www.maxongroup.es/medias/sys_master/root/8834321186846/EPOS2-Firmware-Specification-En.pdf)
