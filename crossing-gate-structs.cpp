#include "crossing-gate-structs.h"
#include <Arduino.h>

int sensor_input_value(struct sensor_input* input){
	int val;

	if(input->gpio > 0){
		val = digitalRead(input->gpio);
		if(input->polarity == POLARITY_ACTIVE_LOW){
			val = !val;
		}

		return val;
	}

	return input->is_on;
}

void sensor_input_handle_event(struct sensor_input* input, uint64_t event_id){
	if(input->event_id_on == event_id){
		input->is_on = 1;
	}else if(input->event_id_off == event_id){
		input->is_on = 0;
	}
}

int sensor_input_valid(struct sensor_input* input){
  if(input->gpio > 0){
    return 1;
  }

  if(input->event_id_off > 0 &&
    input->event_id_on > 0){
      return 1;
    }

  return 0;
}

int switch_input_value(struct switch_input* input){
  if(input->gpio > 0){
      int val = digitalRead(input->gpio);
      if(input->polarity == POLARITY_ACTIVE_LOW){
        val = !val;
      }
      return val;
  }

  return input->current_pos;
}

void switch_input_handle_event(struct switch_input* input, uint64_t event_id){
	if(input->event_id_reverse == event_id){
		input->current_pos = 1;
	}else if(input->event_id_normal == event_id){
		input->current_pos = 0;
	}
}
