#include "stm32f30x_it.h"
#include <sdk/debug.h>

void NMI_Handler(void){
	debug_error(NMI_ERROR);
}

void HardFault_Handler(void){
	debug_error(HARDFAULT_ERROR);
}

void MemManage_Handler(void){
	debug_error(MEMMANAGE_ERROR);
}

void BusFault_Handler(void){
	debug_error(BUSFAULT_ERROR);
}

void UsageFault_Handler(void){
	debug_error(USAGEFAULT_HANDLER);
}

void SVC_Handler(void){
	debug_error(SVC_ERROR);
}

void DebugMon_Handler(void){
	debug_error(DEBUGMON_ERROR);
}

void PendSV_Handler(void){
	debug_error(PENDSV_ERROR);
}
