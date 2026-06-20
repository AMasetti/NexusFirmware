#pragma once
// Single-leg balance on right stance leg (ENABLE_SINGLE_LEG_BALANCE=1)
#if ENABLE_SINGLE_LEG_BALANCE
void task_single_leg_balance(void* arg);
void task_serial_console(void* arg);
#endif
