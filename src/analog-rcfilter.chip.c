// https://wokwi.com/projects/409325290405496833
// custom-chip-RCfilter.ino
// An implementation of an RC filter in a Wokwi custom chip
// See https://docs.wokwi.com/chips-api for API
// 
// SPDX-License-Identifier: MIT
// Copyright (C) 2024 David Forrest

// Connections:
// 0-5V pot wiper attached to A0
// pin 3 PWM out to RC filter IN
// RC filter custom chip OUT to A3

#include "wokwi-api.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
const uint32_t interval = 1000; // microseconds

const bool VERBOSE = false;

typedef struct {
  pin_t pin_out, pin_in;
  uint32_t R_attr, C_attr, Rexp_attr, Cexp_attr ;
  float inVoltage, voltage;
  float tau, frac;
  float r,rexp,c,cexp;
} chip_state_t;

static void chip_timer_event(void *user_data);

void chip_init(void) {
  chip_state_t *chip = malloc(sizeof(chip_state_t));
  chip->pin_in = pin_init("IN", ANALOG);
  chip->pin_out = pin_init("OUT", ANALOG);
  chip->R_attr = attr_init_float("R", 10.0);
  chip->Rexp_attr = attr_init_float("Rexp", 2.0);
  chip->C_attr = attr_init_float("C", 1.0);
  chip->Cexp_attr = attr_init_float("Cexp", -6.0);
  chip->voltage = 0;

  const timer_config_t timer_config = {
    .callback = chip_timer_event,
    .user_data = chip,
  };
  timer_t timer_id = timer_init(&timer_config);
  timer_start(timer_id, interval, true);
}

void chip_timer_event(void *user_data) {
  static uint32_t count = 0;
  chip_state_t *chip = (chip_state_t*)user_data;
  chip->inVoltage =pin_adc_read(chip->pin_in);

   float r = attr_read_float(chip->R_attr);
   float rexp = attr_read_float(chip->Rexp_attr);
   float c = attr_read_float(chip->C_attr);
   float cexp = attr_read_float(chip->Cexp_attr);
   if( r != chip->r 
      || rexp != chip->rexp 
      || c != chip->c
      || cexp != chip->cexp){
        chip->r = r;
        chip->rexp = rexp;
        chip->c = c;
        chip->cexp = cexp;
        chip->tau = r*c*pow(10,rexp+cexp); 
        float t_tau = interval*1e-6 / chip->tau;
        chip->frac = 1-exp(-t_tau);
     if(VERBOSE){
        printf("Tau:%f s", chip->tau);
  //    printf(" t/tau: %f",t_tau);
  //    printf(" exp(-t/tau): %f",chip->frac);
        printf(" %f %f V\n", chip->inVoltage, chip->voltage);
     }
  }
  if(isnan(chip->voltage) || chip->voltage > 1024 || chip->voltage < 0){
    chip->voltage=chip->inVoltage;
  }  
  if(chip->tau > 1e-8){
    chip->voltage+= (chip->inVoltage - chip->voltage) * chip->frac;
    if(VERBOSE && count++ > 1000){
      printf("Tau:%f %f %f V\n", chip->tau,chip->inVoltage,chip->voltage);
      count = 0;
    }
  } else {
    chip->voltage = chip->inVoltage;
  }
  pin_dac_write(chip->pin_out, chip->voltage);
}
