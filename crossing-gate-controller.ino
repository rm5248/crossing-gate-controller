#include <ACAN2515.h>
#include <lcc.h>
#include <lcc-common-internal.h>
#include <lcc-datagram.h>
#include <lcc-event.h>

static const byte MCP2515_CS  = 10 ; // CS input of MCP2515 (adapt to your design) 
static const byte MCP2515_INT =  2 ; // INT output of MCP2515 (adapt to your design)

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
int inputValue = 0;

enum TrackState{
  TRACK_UNOCCUPIED,
  PRE_ISLAND_OCCUPIED,
  ISLAND_OCCUPIED_INCOMING,
  ISLAND_OCCUPIED,
  POST_ISLAND_OCCUPIED_INCOMING,
  POST_ISLAND_OCCUPIED,
};

enum Direction{
  DIRECTION_UNKNOWN,
  DIRECTION_LTR,
  DIRECTION_RTL,
};

struct TrackInformation{
  // The furthest left input going into the crossing
  int left_input;
  // The input on the left side of the island
  int left_island_input;
  // The input on the right side of the island
  int right_island_input;
  // The furthest right input going into the crossing
  int right_input;
  enum TrackState current_state;
  enum Direction current_direction;
  // When the train came in, so we can timeout
  unsigned long incoming_millis;
};

enum GateFlashState{
  FLASH_OFF,
  FLASH_ON,
};

struct TrackInformation track1;
struct TrackInformation track2;
enum GateFlashState gate_flash;
unsigned long timeout_millis = 25000;

/**
 * This is a callback function that is called by liblcc in order to write a frame out to the CAN bus.
 */
void lcc_write(struct lcc_context*, struct lcc_can_frame* lcc_frame){
  frame.id = lcc_frame->can_id;
  frame.len = lcc_frame->can_len;
  frame.rtr = false;
  frame.ext = true;
  memcpy(frame.data, lcc_frame->data, 8);
  if(can.tryToSend (frame)){
    Serial.println(F("Send frame OK"));
    Serial.println(frame.id, HEX);
  }
}

/**
 * Determine if a track is occupied for a flash state.
 * The track is occupied if:
 *  - The pre-island is occupied
 *  - the island is occupied
 *
 * Once the train passes the island, it is no longer considered occupied for flashing purposes
 */
int is_track_occupied_for_flash(struct TrackInformation* track){
  if(track->current_state == PRE_ISLAND_OCCUPIED ||
    track->current_state == ISLAND_OCCUPIED_INCOMING ||
    track->current_state == ISLAND_OCCUPIED){
    return 1;
  }

  return 0;
}

void handle_gate_flash(){
  enum GateFlashState expectedGateFlashState = FLASH_OFF;

  if(is_track_occupied_for_flash(&track1) ||
    is_track_occupied_for_flash(&track2)){
      expectedGateFlashState = FLASH_ON;
  }

  if(expectedGateFlashState != gate_flash){
    gate_flash = expectedGateFlashState;

    if(gate_flash == FLASH_ON){
      digitalWrite(8, 1);
    }else{
      digitalWrite(8, 0);
    }
  }
}

void handle_ltr(struct TrackInformation* track, int left_input, int left_island_input, int right_island_input, int right_input){
  if(left_island_input == 1 && 
    track->current_state == PRE_ISLAND_OCCUPIED){
    track->current_state = ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("Island occupied incoming"));
  }else if(right_island_input == 1 &&
    track->current_state == ISLAND_OCCUPIED_INCOMING){
    track->current_state = ISLAND_OCCUPIED;
    Serial.println(F("island occupied"));
  }else if(right_island_input == 0 &&
    track->current_state == ISLAND_OCCUPIED){
    track->current_state = POST_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("post island occupied incoming"));
  }else if(right_input == 1 &&
    track->current_state == POST_ISLAND_OCCUPIED_INCOMING){
    track->current_state = POST_ISLAND_OCCUPIED;
    Serial.println(F("post island occupied"));
  }else if(right_input == 0 &&
    track->current_state == POST_ISLAND_OCCUPIED){
    Serial.println(F("train out LTR"));
    track->current_state = TRACK_UNOCCUPIED;
    track->current_direction = DIRECTION_UNKNOWN;
  }
}

void handle_rtl(struct TrackInformation* track, int left_input, int left_island_input, int right_island_input, int right_input){
  if(right_island_input == 1 && 
    track->current_state == PRE_ISLAND_OCCUPIED){
    track->current_state = ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("Island occupied incoming"));
  }else if(left_island_input == 1 &&
    track->current_state == ISLAND_OCCUPIED_INCOMING){
    track->current_state = ISLAND_OCCUPIED;
    Serial.println(F("island occupied"));
  }else if(left_island_input == 0 &&
    track->current_state == ISLAND_OCCUPIED){
    track->current_state = POST_ISLAND_OCCUPIED_INCOMING;
    Serial.println(F("post island occupied incoming"));
  }else if(left_input == 1 &&
    track->current_state == POST_ISLAND_OCCUPIED_INCOMING){
    track->current_state = POST_ISLAND_OCCUPIED;
    Serial.println(F("post island occupied"));
  }else if(left_input == 0 &&
    track->current_state == POST_ISLAND_OCCUPIED){
    Serial.println(F("train out RTL"));
    track->current_state = TRACK_UNOCCUPIED;
    track->current_direction = DIRECTION_UNKNOWN;
  }
}

void handle_track(struct TrackInformation* track){
  int left_input = !digitalRead(track->left_input);
  int left_island_input = !digitalRead(track->left_island_input);
  int right_island_input = !digitalRead(track->right_island_input);
  int right_input = !digitalRead(track->right_input);
  unsigned long millis_diff = millis() - track->incoming_millis;

  if(left_input == 1 && track->current_state == TRACK_UNOCCUPIED){
    // Incoming train, left to right
    track->current_state = PRE_ISLAND_OCCUPIED;
    track->current_direction = DIRECTION_LTR;
    track->incoming_millis = millis();
    Serial.println(F("Incoming train LTR"));
    return;
  }else if(right_input == 1 && track->current_state == TRACK_UNOCCUPIED){
    // Incoming train, right to left
    track->current_state = PRE_ISLAND_OCCUPIED;
    track->current_direction = DIRECTION_RTL;
    track->incoming_millis = millis();
    Serial.println(F("Incoming train RTL"));
    return;
  }

  if(track->current_direction == DIRECTION_LTR){
    handle_ltr(track, left_input, left_island_input, right_island_input, right_input);
  }else if(track->current_direction == DIRECTION_RTL){
    handle_rtl(track, left_input, left_island_input, right_island_input, right_input);
  }

  if(millis_diff > timeout_millis &&
    track->current_state != TRACK_UNOCCUPIED){
    track->current_state = TRACK_UNOCCUPIED;
    track->current_direction = DIRECTION_UNKNOWN;
  }
}

void setup () {
  track1.left_input = A4;
  track1.left_island_input = A3;
  track1.right_island_input = A2;
  track1.right_input = A1;
  track1.current_state = TRACK_UNOCCUPIED;
  track1.current_direction = DIRECTION_UNKNOWN;

  track2.left_input = A5;
  track2.left_island_input = A3;
  track2.right_island_input = A2;
  track2.right_input = A0;
  track2.current_state = TRACK_UNOCCUPIED;
  track2.current_direction = DIRECTION_UNKNOWN;

  gate_flash = FLASH_OFF;

  // Define a unique ID for your node.  The generation of this unique ID can be
  // found in the LCC specifications, specifically the unique identifiers standard
  uint64_t unique_id = 0x040032405022llu;

  // Create an LCC context that determines our communications
  ctx = lcc_context_new();

  // Set the unique identifier that this node will use
  lcc_context_set_unique_identifer(ctx, unique_id);

  // Set the callback function that will be called to write  frame out to the bus
  lcc_context_set_write_function(ctx, lcc_write);

  // Set simple node information that is handled by the 'simple node information protocol'
  lcc_context_set_simple_node_information(ctx,
                                        "Manufacturer",
                                        "Model",
                                        "HWVersion",
                                        "SWVersion");

  // Optional: create other contexts to handle other parts of LCC communication
  // Contexts:
  // * Datagram - allows transfers of datagrams to/from the device
  // * Event - event producer/consumer
  // * Memory -  memory read/writing on the node.  Requires a datagram context to exist
  // All contexts are owned by the parent lcc_context and are not free'd by the caller
  lcc_datagram_context_new(ctx);
  struct lcc_event_context* evt_ctx = lcc_event_new(ctx);

  uint64_t event_id = unique_id << 16;
  lcc_event_add_event_produced(evt_ctx, event_id);
  lcc_event_add_event_produced(evt_ctx, event_id + 1);
  lcc_event_add_event_produced(evt_ctx, event_id + 2);


  pinMode (LED_BUILTIN, OUTPUT) ;
  digitalWrite (LED_BUILTIN, HIGH) ;

  // This particular code uses the SparkFun CAN-BUS shield, where pins 7 and 8
  // are LED outputs.
  // Pin 4 is used as a sample digital input that will generate LCC events when it
  // changes state.  Make sure to put a pull-down on this pin, and you can then trigger
  // events by connecting and disconnecting it from the +5v rail.
  pinMode(4, INPUT);
  pinMode(7, OUTPUT);
  pinMode(8, OUTPUT);

  // Input sensors for the track
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(A4, INPUT);
  pinMode(A5, INPUT);

  Serial.begin (9600) ;
  while (!Serial) {
    delay (50) ;
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
  }

  SPI.begin () ;
  Serial.println ("Configure ACAN2515") ;
  ACAN2515Settings settings (QUARTZ_FREQUENCY, 125UL * 1000UL) ; // CAN bit rate 125 kb/s
  settings.mRequestedMode = ACAN2515Settings::NormalMode;
  // We need to lower the transmit and receive buffer size(at least on the Uno), as otherwise
  // the ACAN2515 library will allocate too much memory
  settings.mReceiveBufferSize = 4;
  settings.mTransmitBuffer0Size = 8;
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
    while(1){}
  }
  
  claim_alias_time = millis() + 220;
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
    digitalWrite (LED_BUILTIN, !digitalRead (LED_BUILTIN)) ;
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
  handle_track(&track1);
  handle_track(&track2);
  handle_gate_flash();
}
