#include "crossing-gate-structs.h"
#include <Arduino.h>

static int gpioToAnalog(int num){
  switch(num){
    case 0: return A0;
    case 1: return A1;
    case 2: return A2;
    case 3: return A3;
    case 4: return A4;
    case 5: return A5;
    case 6: return A6;
    case 7: return A7;
  }
}

int sensor_input_value(struct sensor_input* input){
	int val;
  int polarity = input->flags & FLAG_POLARITY;

	if(input->gpio > 0){

    if(input->flags & FLAG_USE_ANALOG){
      val = analogRead(gpioToAnalog(input->gpio));

      if(val >= input->analog_value){
        val = 1;
      }else{
        val = 0;
      }
    }else{
      val = digitalRead(input->gpio);
    }

    if(polarity == FLAG_POLARITY_ACTIVE_LOW){
      val = !val;
    }

    // if(val){
    //   if(input->flags & FLAG_USE_ANALOG){
    //     Serial.print("Analog ");
    //   }else{
    //     Serial.print("GPIO ");
    //   }
    //   Serial.print(input->gpio);
    //   Serial.println(" ACTIVE");
    // }

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
      if(input->polarity == FLAG_POLARITY_ACTIVE_LOW){
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
