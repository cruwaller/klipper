#ifndef __STEPPER_H
#define __STEPPER_H

#include <stdint.h> // uint8_t

uint_fast8_t stepper_event(struct timer *t);
struct stepper *stepper_oid_lookup(uint8_t oid);
void stepper_stop(struct stepper *s);

struct endstop;
void stepper_set_endstop(struct endstop *e, uint8_t oid);
void stepper_endstop_enable(struct stepper *s, uint8_t enable);

#endif // stepper.h
