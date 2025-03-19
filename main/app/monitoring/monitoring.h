#ifndef __MONITORING_H__
#define __MONITORING_H__

#include <stdint.h>

// creates binary semaphore which deferrs isr handling to monitoring task 
void semaphore_init();

// interrupt handler for lora packet reception event
void lora_isr();

// main communication task that occurs at each device poll
void monitoring();

#endif
