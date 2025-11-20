# EPOS-CAN

EPOS-CAN is a lightweight library for managing Maxon EPOS devices using Espressif microcontrollers with ESP-IDF and Arduino IDE. It allows interaction with PDO, SDO (upload and download), and NMT protocols. It features a serial console compatible with CiA 309-3 and includes numerous functionalities to make the interaction more user-friendly.

Although it is still a work in progress, we believe it has reached a quite useable state and therefore we are making it public.

## Hardware requirements

| Supported Targets | ESP32 | ESP32-C3 | ESP32-C6 | ESP32-H2 | ESP32-P4 | ESP32-S2 | ESP32-S3 |
| ----------------- | ----- | -------- | -------- | -------- | -------- | -------- | -------- |

* Espressif ESP32 dev module
* SN65HVD230 or equivalent transceiver
* Maxon EPOS Drive Controller

## Installation

### ESP-IDF

#### Method 1: Managed Component (Recommended)

Add the component as a dependency using the ESP-IDF Component Manager. This method automatically manages updates and dependencies.

1. Create or edit `main/idf_component.yml` in your project:

```yaml
dependencies:
  epos:
    git: https://github.com/uclm-mantis/epos.git
```

2. Build your project:

```bash
idf.py build
```

3. The component will be automatically downloaded to `managed_components/epos/`

4. Include in your code:

```c
#include "epos.h"
```

#### Method 2: Git Submodule

Add the repository as a git submodule to your project's components directory:

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

## Usage

### Programmatic Usage

#### ESP-IDF Example (High-Level Motor API)

The high-level API from `epos_motor.h` provides an object-oriented interface for motor control:

```c
#include "epos.h"
#include "epos_motor.h"

void app_main(void)
{
    // Initialize EPOS with console enabled
    epos_initialize(true);
    
    // Create motor instance for node 2
    Motor_t* motor = motor_new(2);
    
    // Reset and activate motor
    motor_reset(motor);
    
    // Configure profile parameters
    MotorProfile_t profile = {
        .velocity = 1000,
        .acceleration = 5000,
        .deceleration = 5000,
        .type = PPM_TRAPEZOIDAL
    };
    motor_set_profile(motor, &profile);
    
    // Move to absolute position
    motor_profile_position_absolute(motor, 10000);
    
    // Wait until target is reached
    motor_wait_target_reached(motor);
    
    printf("Movement completed!\n");
}
```

#### ESP-IDF Example (EDS-Generated API)

The EDS API provides type-safe functions generated from the EPOS object dictionary:

```c
#include "epos.h"

void app_main(void)
{
    // Initialize EPOS with console enabled
    epos_initialize(true);
    
    // Set target node
    uint8_t node = 2;
    
    // Reset node
    nmt_reset_node(node);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Activate the device
    set_controlword(node, 0);
    set_controlword(node, 6);
    set_controlword(node, 7);
    set_controlword(node, 0x0f);
    
    // Set current mode
    set_modes_of_operation(node, -3);
    
    // Set desired current (500 mA)
    set_current_mode_setting_value(node, 500);
    
    // Read actual current
    int16_t current;
    get_current_actual_value(node, &current);
    printf("Actual current: %d mA\n", current);
}
```

#### CANopen API

The CANopen API provides direct access to CANopen communication services (SDO, NMT, PDO, EMCY). This example shows SDO usage:

```c
#include "epos.h"

void app_main(void)
{
    // Initialize EPOS with console enabled
    epos_initialize(true);
    
    // Set target node
    uint8_t node = 2;
    uint32_t cob_id = 0x600 + node;  // SDO client-to-server
    
    // Reset node using NMT
    nmt_reset_node(node);
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Activate the device using SDO download
    uint16_t controlword;
    
    controlword = 0;
    sdo_download(cob_id, 0x6040, 0, &controlword, sizeof(controlword));
    
    controlword = 6;
    sdo_download(cob_id, 0x6040, 0, &controlword, sizeof(controlword));
    
    controlword = 7;
    sdo_download(cob_id, 0x6040, 0, &controlword, sizeof(controlword));
    
    controlword = 0x0f;
    sdo_download(cob_id, 0x6040, 0, &controlword, sizeof(controlword));
    
    // Set current mode (index 0x6060, subindex 0)
    int8_t mode = -3;  // Current mode
    sdo_download(cob_id, 0x6060, 0, &mode, sizeof(mode));
    
    // Set desired current (index 0x2030, subindex 0) - 500 mA
    int16_t current_setting = 500;
    sdo_download(cob_id, 0x2030, 0, &current_setting, sizeof(current_setting));
    
    // Read actual current (index 0x6078, subindex 0)
    int16_t current_actual;
    sdo_upload(cob_id, 0x6078, 0, &current_actual);
    printf("Actual current: %d mA\n", current_actual);
}
```

For detailed documentation on each CANopen service, see the sections below.

#### Arduino Example

```cpp
#include <EposCAN.h>

EPOS::CAN can;
EPOS::Motor motor(1);  // Node ID 1

void setup()
{
    Serial.begin(115200);
    
    // Initialize CAN with console enabled
    can.begin(true);
    
    // Reset and activate motor
    motor.reset();
    
    // Example: Profile position mode
    motor.profile_position_relative(32000);
}

void loop()
{
    if (motor.is_target_reached()) {
        Serial.println("Target reached!");
        delay(1000);
    }
}
```

### Console Usage

The EPOS-CAN library includes a powerful serial console for interactive CANopen communication.

* Connect with your board using a Monitor. Sometimes the ESP-IDF builtin monitor on Windows is not recognized as an ANSI capable monitor and the console thread falls back to dumb mode. You may either press the reset button in the development board or use a different monitor application (such as *putty*).
* The console implements full line editing, completion and history using `linenoise`.

#### Basic commands


The console is almost compatible with CiA 309 as implemented by CANopenNode. Please, note that we do not have direct access to CiA 309, we do not aim at a fully compliant implementation. Nonetheless, although we implement significant improvements over a basic CiA 309, the only known minor departure from CiA 309 has to do with negative numbers. See below.

The console has a builtin help system:

```
> help
Available commands:
  [[net] node] r[ead] <index> <subindex> [<datatype>]
  [[net] node] r[ead] <symbol> [<datatype>]
  [[net] node] w[rite] <index> <subindex> <datatype> <value>
  [[net] node] w[rite] <symbol> <value>
  [[net] node] start
  [[net] node] stop
  [[net] node] preop[erational]
  [[net] node] reset node|comm[unication]
  [net] set network|node|sdo_timeout|sdo_block <value>
  help [datatype|object|<symbol prefix>]
```

This help is not automatically generated by the argument parser library because we need to implement *net* and *node* handling by hand before they are passed to the actual command. Therefore it does not document extended commands.  In particular, the project comes with a WIP compatibility layer for a former library (FC) as an example of how a user may add a custom command set.

Although CiA 309 allows multiple CAN networks connected to the same controller, we are only using devices with a single TWAI peripheral. We are aware of the development of dual TWAI Espressif chips, but we haven't got one yet.

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

Nothe that you may us the cursor *up* key to recall previous command and then edit that command to issue the new one.

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

## CANopen API Reference

The library provides direct access to CANopen communication services. This section documents the low-level CANopen API.

### NMT (Network Management)

NMT services control the state of CANopen nodes:

```c
#include "epos.h"

// Reset node (triggers boot-up)
esp_err_t nmt_reset_node(uint8_t node);

// Reset communication only
esp_err_t nmt_reset_communication(uint8_t node);

// Start remote node (transition to Operational)
esp_err_t nmt_start_remote_node(uint8_t node);

// Stop remote node
esp_err_t nmt_stop_remote_node(uint8_t node);

// Enter pre-operational state
esp_err_t nmt_enter_preoperational(uint8_t node);
```

Example:

```c
uint8_t node = 2;

// Reset and wait for boot-up
nmt_reset_node(node);
vTaskDelay(pdMS_TO_TICKS(1000));

// Start node (enter Operational state)
nmt_start_remote_node(node);
```

### SDO (Service Data Object)

For each object in the object dictionary you have a `setter` if it is writeable and a `getter` if it is readable.  Syntax of a setter is as follows:

```C
esp_err_t ec = set_controlword(n, 7);
```

First argument is the Node ID, second argument is the value to be set. The type of `value` is given in the object dictionary (see `object_dictionary.h`). It should return `ESP_OK` if everything goes as expected.

A bitwise description of the types required for a given object is also available in `epos_types.h`. For example, type `ControlWord_t` allows bit-level manipulation of the control word. The above example may also be written as:

```C
esp_err_t ec = set_controlword(n, (ControlWord_t){.enable_voltage = 1, .quickstop = 1, .switch_on = 1});
```

Getters are similar but second argument is a pointer to the location of the actual value.  For example, you may read the actual current value (in mA) as follows:

```C
int16_t current;
esp_err_t ec = get_current_actual_value(node, &current);
```

Sometimes you may also require bit-level manipulation of the return value. Use the types provided in `epos_types.h` as follows:

```C
StatusWord_t status;
esp_err_t ec = get_statusword(n, (uint16_t*)&status);
if (status.target_reached) {
    // ...
}
```

SDO interaction is synchronous. If there is an ongoing SDO request while the running thread starts another SDO request then the running thread would block until a response is received for the ongoing SDO request.  There is a setable parameter `sdo_timeout` to adjust the maximum waiting time before a TIMEOUT error is returned.

#### Low-Level SDO Functions

For direct CANopen communication:

```c
// Download (write) data to object dictionary
esp_err_t sdo_download(uint32_t cob_id, uint16_t index, uint8_t subindex, 
                       void* value, size_t size);

// Upload (read) data from object dictionary
esp_err_t sdo_upload(uint32_t cob_id, uint16_t index, uint8_t subindex, 
                     void* ret);
```

Example:

```c
uint8_t node = 2;
uint32_t cob_id = 0x600 + node;

// Write controlword (0x6040)
uint16_t controlword = 0x0f;
sdo_download(cob_id, 0x6040, 0, &controlword, sizeof(controlword));

// Read statusword (0x6041)
uint16_t statusword;
sdo_upload(cob_id, 0x6041, 0, &statusword);
```

### PDO (Process Data Object), NMT Heartbeats, EMCY

There are two ways to receive asynchronous notifications such as NMT heartbeats, PDO notifications or EMCY errors. The simplest interface is the synchronous interface.  Yoy may wait for a specific notification. For example, supose you are waiting for actual velocity updates from a periodic PDO with COB-ID 0x182 (0x180 + node ID).

```C
typedef struct __attribute__((packed)) {
    int32_t velocity;
    // ... remaining mapped objects
} PDO1_t;


for(;;) {
    PDO1_t pdo;
    esp_err_t ec = epos_wait_until(0x180 + n, &pdo);
    // use new pdo.velocity value
}
```

This is by far the easiest use case but sometimes you need to react to whatever notification it arrives and you may not be able to use one thread per COB-ID.  Then you can use a simple callback mechanism. For example, assume you will handle NMT heartbeat notifications in the same thread you wait for a PDO message:

```C
void my_heartbeat_handler(uint32_t cobid, void* msg, void* context)
{
    NMT_HB_state_t* state = (NMT_HB_state_t*) msg;
    uint8_t* current_state = (uint8_t*) context;
    *current_state = *state; 
}
```

Registering the callback is straightforward:

```C
uint8_t current_state;
epos_register_canopen_handler(0x700 + n, &my_heartbeat_handler, &current_state);
```

## Build from CANopen EDS

Although this project started as a library to support Maxon controllers programming from Espressif microcontrollers, it may also be used to build custom libraries to support other CANopen devices.

CANopen Electronic Data Sheet (EDS files, CiA 306-1) provide a full description of the capabilities of a device. You may use `eds_to_od.py` python script to extract the information from the EDS into a simple object dictionary table. Then, you may use `extract.py` python script to automatically generate an API customized for your device. 

## Acknowledgements

This project is partially funded by EU (Erasmus+ 2022-1-ES01- KA220-HED-000089155), JCCM and FEDER (SBPLY/21/180501/000238) and by NATO (SPS.MYP.G6001) research grants.

Developed by UCLM [Mantis Research Group](https://uclm-mantis.github.io/). 

Contact: [mantis@on.uclm.es](mailto:mantis@on.uclm.es).

### Contributors:

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
