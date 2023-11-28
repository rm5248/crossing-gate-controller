#include <ACAN2515.h>
#include <lcc.h>
#include <lcc-common-internal.h>
#include <lcc-datagram.h>
#include <lcc-event.h>
#include <lcc-memory.h>

#include "crossing-gate-structs.h"

// Size per route is:
// 4x sensor inputs(align to 64 bytes)
// 8x switch inputs(align to 64 bytes)
#define MEMORY_SIZE_ROUTE_EEPROM (12 * 64)
#define NUM_ROUTES 8

// 1x other space(align to 64 bytes)
#define MAX_MEMORY_BYTES ((NUM_ROUTES * MEMORY_SIZE_ROUTE_EEPROM) + 64)

// STM32
// #include "stm32f401xe.h"

const char cdi[] PROGMEM = { "<?xml version='1.0'?> \
<cdi xmlns:xsi='http://www.w3.org/2001/XMLSchema-instance' xsi:noNamespaceSchemaLocation='http://openlcb.org/schema/cdi/1/1/cdi.xsd'> \
<identification> \
<manufacturer>Snowball Creek</manufacturer> \
<model>Crossing Gate Controller</model> \
<hardwareVersion>1.0</hardwareVersion> \
<softwareVersion>0.1</softwareVersion> \
</identification> \
<acdi/> \
<!-- \
	<segment space='251'> \
		<name>Node ID</name> \
		<group> \
			<name>Your name and description for this node</name> \
			<string size='63'> \
				<name>Node Name</name> \
			</string> \
			<string size='64' offset='1'> \
				<name>Node Description</name> \
			</string> \
		</group>	 \
	</segment> \
--> \
<segment space='253'> \
<name>Routes</name> \
<group replication='8'> \
<name>Route</name> \
<repname>Route</repname> \
<group replication='4'> \
<name>Sensor Inputs</name> \
<repname>Sensor Inputs</repname> \
<int size='1'> \
<name>GPIO Number</name> \
<description>GPIO number to use as input to this sensor on this route.  Set to 0 to disable and use EventIDs.</description> \
</int> \
<int size='1'> \
<name>GPIO Type</name> \
<description>GPIO Type(analog or normal)</description> \
<map> \
<relation> \
<property>0</property> \
<value>GPIO</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Analog GPIO</value> \
</relation> \
</map> \
</int> \
<int size='1'> \
<name>Polarity</name> \
<description>GPIO polarity</description> \
<map> \
<relation> \
<property>0</property> \
<value>High</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Low</value> \
</relation> \
</map> \
</int> \
<!-- 1 byte padding here --> \
<int size='2' offset='1'> \
<name>Analog Value</name> \
<description>If using an analog pin, values above this will be 'on'</description> \
<min>0</min> \
<max>1023</max> \
<default>250</default> \
</int> \
<eventid> \
<name>Event Id ON</name> \
<description>If not using GPIO, event ID to indicate that this sensor is on</description> \
</eventid> \
<eventid> \
<name>Event Id OFF</name> \
<description>If not using GPIO, event ID to indicate that this sensor is off</description> \
</eventid> \
</group> \
<!-- Switch inputs --> \
<group replication='8'> \
<name>Switch Inputs</name> \
<repname>Switch Input</repname> \
<int size='1'> \
<name>GPIO Number</name> \
<description>GPIO number to use as input to this sensor on this route.  Set to 0 to disable and use EventIDs.</description> \
</int> \
<int size='1'> \
<name>Polarity</name> \
<description>GPIO polarity</description> \
<map> \
<relation> \
<property>0</property> \
<value>High</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Low</value> \
</relation> \
</map> \
</int> \
<int size='1'> \
<name>Route Posistion</name> \
<description>The posistion this switch needs to be in for this route to be valid</description> \
<map> \
<relation> \
<property>0</property> \
<value>Normal</value> \
</relation> \
<relation> \
<property>1</property> \
<value>Reverse</value> \
</relation> \
</map> \
</int> \
<eventid> \
<name>Event Id Normal</name> \
<description>If not using GPIO, event ID to indicate that this switch is normal</description> \
</eventid> \
<eventid> \
<name>Event Id Reverse</name> \
<description>If not using GPIO, event ID to indicate that this switch is reversed</description> \
</eventid> \
</group> \
</group> \
<!-- The next page contains other information about the system. --> \
<!-- The first two bytes are for meta-info that the software uses, so this offset must be two more than the route replication --> \
<int size='2' offset='44'> \
<name>Timeout</name> \
<description>Timeout value for system to reset and become 'inactive'.  This value is in seconds</description> \
<min>0</min> \
<max>120</max> \
<default>25</default> \
</int> \
</segment> \
</cdi>" };

static const byte MCP2515_CS  = 9 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  2 ; // INT output of MCP2515 (adapt to your design)
static const byte EEPROM_CS = 10;

static const byte EEPROM_WRITE_ENABLE = 0x6;
static const byte EEPROM_READ_STATUS_REGISTER = 0x5;
static const byte EEPROM_WRITE_STATUS_REGISTER = 0x1;
static const byte EEPROM_READ_MEMORY_ARRAY = 0x3;
static const byte EEPROM_WRITE_MEMORY_ARRAY = 0x2;
static const byte EEPROM_WRITE_DISABLE = 0x4;

static const int LCC_UNIQUE_ID_ADDR = 0x6000;

// The CAN controller.  This example uses the ACAN2515 library from Pierre Molinaro:
// https://github.com/pierremolinaro/acan2515
// This particular example also uses the SparkFun CAN-BUS Shield:
// https://www.sparkfun.com/products/13262
ACAN2515 can (MCP2515_CS, SPI, MCP2515_INT) ;

static const uint32_t QUARTZ_FREQUENCY = 16UL * 1000UL * 1000UL ; // 16 MHz

static lcc_context* ctx;
CANMessage frame ;
struct lcc_can_frame lcc_frame;
unsigned long claim_alias_time;
static uint32_t gBlinkLedDate = 0 ;
static uint64_t unique_id;

enum GateFlashState{
  FLASH_OFF,
  FLASH_ON,
};

enum GateFlashState gate_flash;
unsigned long timeout_millis = 25000;
struct route crossing_routes[NUM_ROUTES];
int blink_val = 0;

int found_eeprom = 0;

void find_eeprom(){
  Serial.println("Looking for EEPROM");

  delay(5);

  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_WRITE_ENABLE);
  digitalWrite(EEPROM_CS, HIGH);

  delay(5);

  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_READ_STATUS_REGISTER);
  int read_status_reg = SPI.transfer(0xFF);
  digitalWrite(EEPROM_CS, HIGH);

  delay(5);

  Serial.print("status reg: ");
  Serial.println(read_status_reg);

  if(read_status_reg != 0xFF &&
     read_status_reg & (0x01 << 1)){
    // WEL bit is set, so we are talking with the EEPROM!
    // Let's go and disable it again
    found_eeprom = 1;
    digitalWrite(EEPROM_CS, LOW);
    SPI.transfer(EEPROM_WRITE_DISABLE);
    digitalWrite(EEPROM_CS, HIGH);
    Serial.println("Found EEPROM!");
  }
}

void eeprom_read(int offset, void* data, int numBytes){
  digitalWrite(EEPROM_CS, LOW);
  delay(2);
  SPI.transfer(EEPROM_READ_MEMORY_ARRAY);

  SPI.transfer((offset & 0xFF00) >> 8);
  SPI.transfer((offset & 0x00FF) >> 0);

  uint8_t* u8_data = data;
  while(numBytes > 0){
    numBytes--;
    *u8_data = SPI.transfer(0xFF); // dummy byte
    u8_data++;
  }

  digitalWrite(EEPROM_CS, HIGH);
}

// Note: only handles one page at a time.  Anything more than a page is a bug.
// Page size: 64 bytes
void eeprom_write(int offset, void* data, int numBytes){
  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_WRITE_ENABLE);
  digitalWrite(EEPROM_CS, HIGH);

  delay(5);

  digitalWrite(EEPROM_CS, LOW);
  SPI.transfer(EEPROM_WRITE_MEMORY_ARRAY);
  SPI.transfer((offset & 0xFF00) >> 8);
  SPI.transfer((offset & 0x00FF) >> 0);

  uint8_t* u8_data = data;
  while(numBytes > 0){
    numBytes--;
    SPI.transfer(*u8_data); // data byte
    u8_data++;
  }

  digitalWrite(EEPROM_CS, HIGH);

  int numTimes = 0;
  while(numTimes < 10){
    // Read until the write in progress bit is 0
    numTimes++;
    delay(1);

    digitalWrite(EEPROM_CS, LOW);
    SPI.transfer(EEPROM_READ_STATUS_REGISTER);
    int read_status_reg = SPI.transfer(0xFF);
    digitalWrite(EEPROM_CS, HIGH);

    if(read_status_reg & 0x01 == 0){
      break;
    }
  }
}

void display_freeram() {
  Serial.print(F("- SRAM left: "));
  Serial.println(freeRam());
}

int freeRam() {
  extern int __heap_start,*__brkval;
  int v;
  return (int)&v - (__brkval == 0  
    ? (int)&__heap_start : (int) __brkval);  
}

/**
 * This is a callback function that is called by liblcc in order to write a frame out to the CAN bus.
 */
int lcc_write(struct lcc_context*, struct lcc_can_frame* lcc_frame){
  frame.id = lcc_frame->can_id;
  frame.len = lcc_frame->can_len;
  frame.rtr = false;
  frame.ext = true;
  memcpy(frame.data, lcc_frame->data, 8);
  if(can.tryToSend (frame)){
    // Serial.println(F("Send frame OK"));
    // Serial.println(frame.id, HEX);
    return LCC_OK;
  }

  Serial.println(F("Unable to send frame!"));
  return LCC_ERROR_TX;
}

/**
 * Determine if a route is occupied for a flash state.
 * The route is occupied if:
 *  - The pre-island is occupied
 *  - the island is occupied
 *
 * Once the train passes the island, it is no longer considered occupied for flashing purposes
 */
int is_route_occupied_for_flash(struct route* route){
  if(route->current_train.location == LOCATION_PRE_ISLAND_OCCUPIED ||
    route->current_train.location == LOCATION_ISLAND_OCCUPIED_INCOMING ||
    route->current_train.location == LOCATION_ISLAND_OCCUPIED){
    return 1;
  }

  return 0;
}

void handle_gate_flash(){
  enum GateFlashState expectedGateFlashState = FLASH_OFF;

  for(int x = 0; x < sizeof(crossing_routes) / sizeof(crossing_routes[0]); x++){
    if(is_route_occupied_for_flash(&crossing_routes[x])){
      expectedGateFlashState = FLASH_ON;
    }
  }

  if(expectedGateFlashState != gate_flash){
    gate_flash = expectedGateFlashState;

    if(gate_flash == FLASH_ON){
      digitalWrite(5, 1);
    }else{
      digitalWrite(5, 0);
    }
  }
}

void handle_route_ltr(struct route* route, int left_input, int left_island_input, int right_island_input, int right_input){
  if(left_island_input == 1 && 
    route->current_train.location == LOCATION_PRE_ISLAND_OCCUPIED){
    route->current_train.location = LOCATION_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("Island occupied incoming"));
  }else if(right_island_input == 1 &&
    route->current_train.location == LOCATION_ISLAND_OCCUPIED_INCOMING){
    route->current_train.location = LOCATION_ISLAND_OCCUPIED;
    Serial.println(F("island occupied"));
  }else if(right_island_input == 0 &&
    route->current_train.location == LOCATION_ISLAND_OCCUPIED){
    route->current_train.location = LOCATION_POST_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("post island occupied incoming"));
  }else if(right_input == 1 &&
    route->current_train.location == LOCATION_POST_ISLAND_OCCUPIED_INCOMING){
    route->current_train.location = LOCATION_POST_ISLAND_OCCUPIED;
    Serial.println(F("post island occupied"));
  }else if(right_input == 0 &&
    route->current_train.location == LOCATION_POST_ISLAND_OCCUPIED){
    Serial.println(F("train out LTR"));
    route->current_train.location = LOCATION_UNOCCUPIED;
    route->current_train.direction = DIRECTION_UNKNOWN;
  }
}

void handle_route_rtl(struct route* route, int left_input, int left_island_input, int right_island_input, int right_input){
  if(right_island_input == 1 && 
    route->current_train.location == LOCATION_PRE_ISLAND_OCCUPIED){
    route->current_train.location = LOCATION_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("Island occupied incoming"));
  }else if(left_island_input == 1 &&
    route->current_train.location == LOCATION_ISLAND_OCCUPIED_INCOMING){
    route->current_train.location = LOCATION_ISLAND_OCCUPIED;
    Serial.println(F("island occupied"));
  }else if(left_island_input == 0 &&
    route->current_train.location == LOCATION_ISLAND_OCCUPIED){
    route->current_train.location = LOCATION_POST_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("post island occupied incoming"));
  }else if(left_input == 1 &&
    route->current_train.location == LOCATION_POST_ISLAND_OCCUPIED_INCOMING){
    route->current_train.location = LOCATION_POST_ISLAND_OCCUPIED;
    Serial.println(F("post island occupied"));
  }else if(left_input == 0 &&
    route->current_train.location == LOCATION_POST_ISLAND_OCCUPIED){
    Serial.println(F("train out RTL"));
    route->current_train.location = LOCATION_UNOCCUPIED;
    route->current_train.direction = DIRECTION_UNKNOWN;
  }
}

void handle_single_route(struct route* route){
  for(int x = 0; x < 4; x++){
    if(!sensor_input_valid(&route->inputs[0])){
      return;
    }
  }

  int left_input = sensor_input_value(&route->inputs[0]);
  int left_island_input = sensor_input_value(&route->inputs[1]);
  int right_island_input = sensor_input_value(&route->inputs[2]);
  int right_input = sensor_input_value(&route->inputs[3]);
  unsigned long millis_diff = millis() - route->current_train.incoming_millis;

  // First check to see if this is a new train coming into the route
  if((left_input || right_input) &&
    route->current_train.location == LOCATION_UNOCCUPIED){
      // There is a new train coming into the route.
      // Let's see if this route is valid or not
      for(int x = 0; x < sizeof(route->switch_inputs) / sizeof(route->switch_inputs[0]); x++){
        if(switch_input_value(&route->switch_inputs[x]) != route->switch_inputs[x].route_position){
          // The switch is not set to the right position for this route to be active
          return;
        }
      }

      // All switches are in the correct position for this route.
      // This route now has a train in it
      route->current_train.incoming_millis = millis();
      route->current_train.location = LOCATION_PRE_ISLAND_OCCUPIED;
      if(left_input){
        route->current_train.direction = DIRECTION_LTR;
        Serial.println(F("Incoming train LTR"));
      }else{
        route->current_train.direction = DIRECTION_RTL;
        Serial.println(F("Incoming train RTL"));
      }

    return;
  }

  if(route->current_train.direction == DIRECTION_RTL){
    handle_route_rtl(route, left_input, left_island_input, right_island_input, right_input);
  }else if(route->current_train.direction == DIRECTION_LTR){
    handle_route_ltr(route, left_input, left_island_input, right_island_input, right_input);
  }

  if(millis_diff > timeout_millis &&
    route->current_train.location != LOCATION_UNOCCUPIED){
    Serial.println(F("timeout"));
    route->current_train.location = LOCATION_UNOCCUPIED;
    route->current_train.direction = DIRECTION_UNKNOWN;
  }
}

static void write_defaults_to_eeprom(){
  // Note: all values in LCC are defined as big-endian, so we will store it that way as well.
  uint16_t eeprom_info = __builtin_bswap16(1);
  uint16_t eeprom_offset = 0;
  uint16_t timeout_value = __builtin_bswap16(25);

  eeprom_write(MAX_MEMORY_BYTES - 64, &eeprom_info, 2);
  eeprom_write(MAX_MEMORY_BYTES - 62, &timeout_value, 2);

  for(int x = 0; x < NUM_ROUTES; x++){
    for(int sensor_input = 0; sensor_input < 4; sensor_input++){
      uint8_t bytes[64] = {0};
      struct sensor_input_eeprom* sensor_eeprom = (sensor_input_eeprom*)bytes;
      sensor_eeprom->analog_value = __builtin_bswap16(250);

      eeprom_write(eeprom_offset, bytes, sizeof(bytes));

      eeprom_offset += 64;
    }

    for(int switch_input = 0; switch_input < 8; switch_input++){
      uint8_t bytes[64] = {0};

      eeprom_write(eeprom_offset, bytes, sizeof(bytes));

      eeprom_offset += 64;
    }
  }
}

static void load_from_eeprom(){
  uint16_t eeprom_offset = 0;
  struct sensor_input_eeprom sensor_eeprom;
  struct switch_input_eeprom switch_eeprom;

  memset(crossing_routes, 0, sizeof(crossing_routes));

  // Load some of our default values.
  // If the EEPROM is blank, we first go and write out sane default values
  uint16_t eeprom_info;
  eeprom_read(MAX_MEMORY_BYTES - 64, &eeprom_info, 2);

  if(eeprom_info == 0xFFFF){
    // EEPROM is blank - write out default values
    Serial.println("EEPROM blank - initializing");
    write_defaults_to_eeprom();
  }

  // Re-read our stuff
  eeprom_read(MAX_MEMORY_BYTES - 64, &eeprom_info, 2);
  eeprom_info = __builtin_bswap16(eeprom_info);
  Serial.print("EEPROM Version ");
  Serial.println(eeprom_info);

  uint16_t default_timeout;
  eeprom_read(MAX_MEMORY_BYTES - 62, &default_timeout, 2);
  default_timeout = __builtin_bswap16(default_timeout);
  timeout_millis = default_timeout * 1000;

  for(int x = 0; x < NUM_ROUTES; x++){
    for(int sensor_input = 0; sensor_input < 4; sensor_input++){
      eeprom_read(eeprom_offset, &sensor_eeprom, sizeof(sensor_eeprom));
      crossing_routes[x].inputs[sensor_input].analog_value = __builtin_bswap16(sensor_eeprom.analog_value);
      crossing_routes[x].inputs[sensor_input].event_id_off = __builtin_bswap64(sensor_eeprom.event_id_off);
      crossing_routes[x].inputs[sensor_input].event_id_on = __builtin_bswap64(sensor_eeprom.event_id_on);
      crossing_routes[x].inputs[sensor_input].gpio = sensor_eeprom.gpio_number;
      if(sensor_eeprom.gpio_type == 1){
        crossing_routes[x].inputs[sensor_input].flags |= FLAG_USE_ANALOG;
      }
      if(sensor_eeprom.polarity){
        crossing_routes[x].inputs[sensor_input].flags |= FLAG_POLARITY_ACTIVE_LOW;
      }

      eeprom_offset += 64;
    }

    for(int switch_input = 0; switch_input < 8; switch_input++){
      eeprom_read(eeprom_offset, &switch_eeprom, sizeof(switch_eeprom));
      crossing_routes[x].switch_inputs[switch_input].event_id_normal = __builtin_bswap64(switch_eeprom.event_id_normal);
      crossing_routes[x].switch_inputs[switch_input].event_id_reverse = __builtin_bswap64(switch_eeprom.event_id_reverse);
      crossing_routes[x].switch_inputs[switch_input].gpio = switch_eeprom.gpio_number;
      crossing_routes[x].switch_inputs[switch_input].polarity = switch_eeprom.polarity;
      crossing_routes[x].switch_inputs[switch_input].route_position = switch_eeprom.route_posistion;

      eeprom_offset += 64;
    }
  }
}

//
// Memory read/write
//

void mem_address_space_information_query(struct lcc_memory_context* ctx, uint16_t alias, uint8_t address_space){
  if(address_space == 251){
    lcc_memory_respond_information_query(ctx, alias, 1, address_space, 64 + 64, 0, 0);
  }else if(address_space == 253){
    // basic config space
    lcc_memory_respond_information_query(ctx, alias, 1, address_space, MAX_MEMORY_BYTES, 0, 0);
  }else{
    // This memory space does not exist: return an error
    lcc_memory_respond_information_query(ctx, alias, 0, address_space, 0, 0, 0);
  }
}

void mem_address_space_read(struct lcc_memory_context* ctx, uint16_t alias, uint8_t address_space, uint32_t starting_address, uint8_t read_count){
  Serial.print("Read space ");
  Serial.print(address_space);
  Serial.print(" starting addr " );
  Serial.print(starting_address);
  Serial.print(" read count ");
  Serial.print(read_count);
  Serial.println();
  if(address_space == 251){
    // This space is what we use for node name/description.
    // The data for this space starts at offset 0x4000
    uint8_t buffer[64];
    eeprom_read(starting_address + 0x4000, buffer, read_count);

    // For any blank data, we will read 0xFF
    for(int x = 0; x < sizeof(buffer); x++){
      if(buffer[x] == 0xFF){
        buffer[x] = 0x00;
      }
    }

    lcc_memory_respond_read_reply_ok(ctx, alias, address_space, starting_address, buffer, read_count);
  }else if(address_space == 253){
    // Basic config space
    if((starting_address + read_count) > MAX_MEMORY_BYTES){
      Serial.println("too much memory??");
      // trying to read too much memory
      lcc_memory_respond_read_reply_fail(ctx, alias, address_space, 0, 0, NULL);
      return;
    }

    uint8_t buffer[64];
    eeprom_read(starting_address, buffer, read_count);

    lcc_memory_respond_read_reply_ok(ctx, alias, address_space, starting_address, buffer, read_count);
  }else{
    Serial.println("READ FAIL");
    lcc_memory_respond_read_reply_fail(ctx, alias, address_space, 0, 0, NULL);
    return;
  }

  Serial.println("DOne read");
}

void mem_address_space_write(struct lcc_memory_context* ctx, uint16_t alias, uint8_t address_space, uint32_t starting_address, void* data, int data_len){
  if(address_space == 251){
    eeprom_write(starting_address, data, data_len);

    lcc_memory_respond_write_reply_ok(ctx, alias, address_space, starting_address);
  }else if(address_space == 253){
    eeprom_write(starting_address, data, data_len);

    load_from_eeprom();

    lcc_memory_respond_write_reply_ok(ctx, alias, address_space, starting_address);
  }else{
    lcc_memory_respond_write_reply_fail(ctx, alias, address_space, starting_address, 0, NULL);
    return;
  }
}

void setup () {
  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

  Serial.println("Crossing gate controller starting up");

  SPI.begin();

  pinMode(EEPROM_CS, OUTPUT);
  digitalWrite(EEPROM_CS, HIGH);

  find_eeprom();

  gate_flash = FLASH_OFF;

  display_freeram();

  unique_id = 0;
  eeprom_read(LCC_UNIQUE_ID_ADDR, &unique_id, 8);

  Serial.print("ID: ");
  for(int x = 0; x < 8; x++){
    uint8_t* as_u8 = (uint8_t*)&unique_id;
    Serial.print(as_u8[x], HEX);
  }
  Serial.println();

  load_from_eeprom();

  // Create an LCC context that determines our communications
  ctx = lcc_context_new();

  // Set the unique identifier that this node will use
  lcc_context_set_unique_identifer(ctx, unique_id);

  // Set the callback function that will be called to write  frame out to the bus
  lcc_context_set_write_function(ctx, lcc_write);

  // Set simple node information that is handled by the 'simple node information protocol'
  lcc_context_set_simple_node_information(ctx,
                                        "Snowball Creek",
                                        "Crossing gate CTL",
                                        "1.0",
                                        "0.1");

  // Optional: create other contexts to handle other parts of LCC communication
  // Contexts:
  // * Datagram - allows transfers of datagrams to/from the device
  // * Event - event producer/consumer
  // * Memory -  memory read/writing on the node.  Requires a datagram context to exist
  // All contexts are owned by the parent lcc_context and are not free'd by the caller
  // lcc_datagram_context_new(ctx);
  struct lcc_event_context* evt_ctx = lcc_event_new(ctx);
  lcc_datagram_context_new(ctx);
  struct lcc_memory_context* mem_ctx = lcc_memory_new(ctx);

  lcc_memory_set_cdi(mem_ctx, cdi, sizeof(cdi), LCC_MEMORY_CDI_FLAG_ARDUINO_PROGMEM);
  lcc_memory_set_memory_functions(mem_ctx, 
    mem_address_space_information_query,
    mem_address_space_read,
    mem_address_space_write);

  uint64_t event_id = unique_id << 16;
  lcc_event_add_event_produced(evt_ctx, event_id);
  lcc_event_add_event_produced(evt_ctx, event_id + 1);
  lcc_event_add_event_produced(evt_ctx, event_id + 2);

  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;

  pinMode(4, INPUT);
  pinMode(5, OUTPUT);
  pinMode(6, OUTPUT);

  // Input sensors for the track
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(2, INPUT);
  pinMode(3, INPUT);
  // pinMode(A4, INPUT);
  // pinMode(A5, INPUT);


  SPI.begin () ;
  display_freeram();
  Serial.println (F("Configure ACAN2515")) ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  // We need to lower the transmit and receive buffer size(at least on the Uno), as otherwise
  // the ACAN2515 library will allocate too much memory
  settings.mReceiveBufferSize = 4;
  settings.mTransmitBuffer0Size = 12;
  const uint16_t errorCode = can.begin (settings, [] { can.isr () ; }) ;
  if (errorCode != 0) {
    Serial.print ("Configuration error 0x") ;
    Serial.println (errorCode, HEX) ;
  }

  // Generate an LCC alias and request it.
  // Note that generating an alias is a two-part step: you must generate the alias,
  // wait at least 200ms, and then claim the alias
  int val = lcc_context_generate_alias(ctx);
  if(val != LCC_OK){
    Serial.println(F("ERROR: Can't generate alias!"));
    Serial.print("Error code: ");
    Serial.println(val);
    while(1){}
  }
  
  claim_alias_time = millis() + 220;

  display_freeram();
}

void loop() {
  if (can.available ()) {
    // If we have an incoming CAN frame, turn it into an LCC frame and push it to liblcc
    can.receive (frame) ;
    lcc_frame.can_id = frame.id;
    lcc_frame.can_len = frame.len;
    memcpy(&lcc_frame.data, frame.data, 8);
    lcc_context_incoming_frame(ctx, &lcc_frame);
  }

  if (gBlinkLedDate < millis ()) {
    gBlinkLedDate += 1000 ;
    digitalWrite (LED_BUILTIN, blink_val) ;
    blink_val = !blink_val;
    // Serial.print(F("blink: "));
    // Serial.println(millis());
  }

  if(millis() >= claim_alias_time &&
    lcc_context_current_state(ctx) == LCC_STATE_INHIBITED){
    int stat = lcc_context_claim_alias(ctx);
    if(stat != LCC_OK){
      // If we were unable to claim our alias, we need to generate a new one and start over
      lcc_context_generate_alias(ctx);
      claim_alias_time = millis() + 220;
    }else{
      Serial.print(F("Claimed alias "));
      Serial.println(lcc_context_alias(ctx), HEX);
    }
  }

  // Read the value of pin 4 - if it changes state, send an event
  // int currentVal = !!digitalRead(4);
  // if(currentVal != inputValue){
  //   inputValue = currentVal;
  //   uint64_t event_id = lcc_context_unique_id(ctx) << 16llu;

  //   if(currentVal == 0){
  //     lcc_event_produce_event(lcc_context_get_event_context(ctx), event_id);
  //   }else{
  //     lcc_event_produce_event(lcc_context_get_event_context(ctx), event_id + 1);
  //   }

  //   // Light up LED with current status
  //   digitalWrite(7, currentVal);
  // }

  for(int x = 0; x < sizeof(crossing_routes) / sizeof(crossing_routes[0]); x++){
    handle_single_route(&crossing_routes[x]);
  }
  handle_gate_flash();
}
