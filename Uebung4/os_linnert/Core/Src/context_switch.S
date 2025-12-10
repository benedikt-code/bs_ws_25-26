.syntax unified
.cpu cortex-m4
.thumb

.global PendSV_Handler
.extern pick_next
.extern os_get_sp
.extern os_set_current
.extern os_get_current
.extern os_store_sp

PendSV_Handler:
    CPSID   I      // (CPSID - Change Processor State, I - disable interrupts)
    MRS     R0, PSP     // (MRS - Move from Special Register, PSP - Process Stack Pointer)
    STMDB   R0!, {R4-R11}   // (STMDB - Store Multiple Decrement Before, R0! - pointer with writeback)

    // save old PSP into sp_table[current]
    // R0 currently == PSP after pushing R4-R11
    // We need to preserve the PSP across calls; use R4 (callee-saved) as temporary to hold it.
    MOV     R4, R0          // copy PSP into R4 so we can call C helpers (R0..R3 may be clobbered by BL)
    BL      os_get_current  // get previous 'current' index; return value in R0
    CMP     R0, #-1
    BLT     .Lskip_store    // if no previous thread, skip storing
    // call os_store_sp(prev_index, sp)
    MOV     R1, R4          // second argument (sp) in R1, first arg (idx) already in R0
    BL      os_store_sp
.Lskip_store:

    // call scheduler pick_next()
    BL      pick_next       // (BL - Branch with Link)
    // r0 contains next index or -1
    CMP     R0, #-1
    BEQ     .Lno_thread     // (BEQ - Branch if Equal)

    // set current via os_set_current(r0)
    MOV     R1, R0
    BL      os_set_current

    // get sp for new current: call os_get_sp(current)
    MOV     R0, R1
    BL      os_get_sp
    // r0 = new sp
    LDMIA   R0!, {R4-R11}       // (LDMIA - Load Multiple Increment After, R0! - pointer with writeback)
    MSR     PSP, R0
    CPSIE   I           // (CPSIE - Change Processor State, I - enable interrupts)
    BX      LR

.Lno_thread:
    // no runnable threads: just re-enable interrupts and return
    CPSIE   I
    BX      LR          // (BX - Branch and Exchange, LR - Link Register)
