#ifndef CROSSING_GATE_STRUCTS_H
#define CROSSING_GATE_STRUCTS_H

#include <stdint.h>
#include <debounce.h>

#define FLAG_POLARITY (0x01 << 0)
#define FLAG_USE_ANALOG (0x01 << 1)
#define FLAG_POLARITY_ACTIVE_HIGH 0
#define FLAG_POLARITY_ACTIVE_LOW 1

#define ROUTE_SWITCH_INPUT_NORMAL 0
#define ROUTE_SWITCH_INPUT_REVERSE 1

// Layout of the sensor input in EEPROM
struct sensor_input_eeprom{
  uint8_t gpio_number;
  uint8_t gpio_type;
  uint8_t polarity;
  uint8_t padding;
  uint16_t analog_value;
  uint64_t event_id_on;
  uint64_t event_id_off;
};

struct switch_input_eeprom{
  uint8_t gpio_number;
  uint8_t polarity;
  uint8_t route_posistion;
  uint8_t padding;
  uint64_t event_id_normal;
  uint64_t event_id_reverse;
};

struct sensor_input{
	uint8_t gpio;
	uint8_t flags;
  uint16_t analog_value;
  Button debouncer;

	// If using event IDs:
	uint64_t event_id_on;
	uint64_t event_id_off;
	uint8_t is_on;
};

struct switch_input{
	uint8_t gpio;
	uint8_t polarity;
	/* what posistion does this switch need to be in for the route to be valid(normal or reverse) */
	uint8_t route_position;

	// If using event IDs:
  uint8_t current_pos;
	uint64_t event_id_normal;
	uint64_t event_id_reverse;
};

enum Direction{
  DIRECTION_UNKNOWN,
  DIRECTION_LTR,
  DIRECTION_RTL,
};

enum TrainLocation{
  LOCATION_UNOCCUPIED,
  LOCATION_PRE_ISLAND_OCCUPIED,
  LOCATION_ISLAND_OCCUPIED_INCOMING,
  LOCATION_ISLAND_OCCUPIED,
  LOCATION_POST_ISLAND_OCCUPIED_INCOMING,
  LOCATION_POST_ISLAND_OCCUPIED,
};

struct train{
	unsigned long last_seen_millis;
	enum Direction direction;
	enum TrainLocation location;
};

struct route{
	struct sensor_input inputs[4];
	struct switch_input switch_inputs[8];
  struct train current_train;
};

/**
 * Determine the current value of the sensor input.  Value will be either 0(for inactive) or 1(for active)
 */
int sensor_input_value(struct sensor_input* input);

/**
 * For the given sensor input, notify that input that an event ID has come in.
 * If this sensor input cares about the event ID, it will update itself.
 */
void sensor_input_handle_event(struct sensor_input* input, uint64_t event_id);

/**
 * Check to see if the sensor input is valid.  A valid sensor input has either a GPIO or both event IDs set.
 */
int sensor_input_valid(struct sensor_input* input);

/**
 * Determine the current value of the given switch.  Value will be either
 * 0(for normal) or 1(for reverse)
 */
int switch_input_value(struct switch_input* input);

void switch_input_handle_event(struct switch_input* input, uint64_t event_id);

#endif /* CROSSING_GATE_STRUCTS_H */
