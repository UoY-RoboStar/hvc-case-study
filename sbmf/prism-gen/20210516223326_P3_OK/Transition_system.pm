// This model is automatically generated from the RoboChart model 
// by the translator (translate2prism v20210223) plugin in RoboTool
// on 2021-05-03 22:05:06
// 
// All changes made will be lost after regeneration. 
 

// PRISM Output

dtmc

const int State_Ramp = (0);
const int State_Init = (1);
const int State_Wait24Vpower = (2);
const int State_ClosedLoop = (3);
const int State_ErrorMode = (4);
const int Power_On = (0);
const int Power_Off = (1);
const int Action_70 = (3);
const int Action_72 = (5);
const int Action_77 = (10);
const int Action_78 = (11);
const int Action_80 = (13);
const int Action_81 = (14);
const int Action_82 = (15);
const int Action_83 = (16);
const int ext_pow24VStatus_75 = (8);
const int ext_pow24VStatus_76 = (9);
const int int_ActualHV_74 = (7);
const int int_ActualHV_79 = (12);
const int int_overLimit_73 = (6);
const int int_underLimit_71 = (4);
const int mod_sys_ctrl_ref1_stm_ref0_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE = (0);
const int mod_sys_ctrl_ref1_stm_ref0_TERMINATED_11 = (7);
const int mod_sys_ctrl_ref1_stm_ref0_i0 = (2);
const int mod_sys_ctrl_ref1_stm_ref0_s0 = (1);
const int mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead = (3);
const int mod_sys_ctrl_ref1_stm_ref0_s0_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead = (6);
const int mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead = (5);
const int mod_sys_ctrl_ref1_stm_ref0_s0_Waiting = (2);
const int mod_sys_ctrl_ref1_stm_ref0_s0_f0 = (1);
const int mod_sys_ctrl_ref1_stm_ref0_s0_i0 = (4);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t0 = (8);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t1 = (7);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t10 = (11);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t2 = (10);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t3 = (6);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t4 = (4);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t5 = (3);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t6 = (9);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t7 = (5);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t8 = (12);
const int mod_sys_ctrl_ref1_stm_ref0_s0_t9 = (13);
const int mod_sys_ctrl_ref1_stm_ref0_t0 = (2);
const int mod_sys_ctrl_ref1_stm_ref0_t1 = (1);
const int Action_67 = (5);
const int Action_68 = (6);
const int ActualHV_1_69 = (7);
const int int_ActualHV_66 = (2);
const int mod_sys_ctrl_ref3_stm0_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref3_stm0_LOCK_FREE = (0);
const int mod_sys_ctrl_ref3_stm0_TERMINATED_11 = (8);
const int mod_sys_ctrl_ref3_stm0_i0 = (3);
const int mod_sys_ctrl_ref3_stm0_s0 = (4);
const int mod_sys_ctrl_ref3_stm0_s0_entering = (1);
const int mod_sys_ctrl_ref3_stm0_t0 = (1);
const int mod_sys_ctrl_ref3_stm0_t1 = (2);
const int Action_29 = (6);
const int Action_30 = (7);
const int Action_33 = (4);
const int Action_34 = (5);
const int Action_35 = (7);
const int Action_36 = (8);
const int Action_37 = (2);
const int Action_38 = (4);
const int Action_39 = (8);
const int Action_40 = (17);
const int Action_41 = (18);
const int Action_42 = (19);
const int Action_43 = (20);
const int Action_44 = (21);
const int Action_45 = (22);
const int Action_46 = (2);
const int Action_48 = (4);
const int Action_50 = (6);
const int Action_51 = (10);
const int Action_53 = (8);
const int Action_55 = (10);
const int Action_56 = (11);
const int Action_57 = (12);
const int Action_58 = (2);
const int Action_59 = (11);
const int Action_61 = (8);
const int Action_63 = (10);
const int Action_64 = (7);
const int ext_pow24VStatus_31 = (8);
const int ext_pow24VStatus_32 = (9);
const int ext_setPoint_52 = (11);
const int ext_setPoint_60 = (3);
const int ext_setPoint_65 = (8);
const int int_ActualHV_54 = (9);
const int int_ActualHV_62 = (9);
const int int_overLimit_47 = (3);
const int int_underLimit_49 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop = (14);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_entering = (6);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_i0 = (8);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_self_13 = (16);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2 = (9);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_i0 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_j0 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1 = (7);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1_entering = (2);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t0 = (25);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t1 = (24);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t2 = (26);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3 = (10);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3_entering = (4);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4 = (7);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4_entering = (3);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t0 = (20);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t1 = (18);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t2 = (21);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t3 = (19);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t4 = (22);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t5 = (23);
const int mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_to_loop_14 = (17);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode = (11);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering = (3);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_f0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_i0 = (7);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_j0 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1_entering = (2);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t0 = (14);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t2 = (15);
const int mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t4 = (13);
const int mod_sys_ctrl_ref0_stm_ref0_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_Init = (9);
const int mod_sys_ctrl_ref0_stm_ref0_Init_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_Init_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_Init_i0 = (8);
const int mod_sys_ctrl_ref0_stm_ref0_Init_loop_15 = (9);
const int mod_sys_ctrl_ref0_stm_ref0_Init_loop_self_16 = (12);
const int mod_sys_ctrl_ref0_stm_ref0_Init_si0 = (7);
const int mod_sys_ctrl_ref0_stm_ref0_Init_si0_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_Init_t0 = (11);
const int mod_sys_ctrl_ref0_stm_ref0_Init_to_loop_17 = (10);
const int mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE = (0);
const int mod_sys_ctrl_ref0_stm_ref0_Ramping = (13);
const int mod_sys_ctrl_ref0_stm_ref0_Ramping_entering = (5);
const int mod_sys_ctrl_ref0_stm_ref0_TERMINATED_11 = (23);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower = (15);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_entering = (7);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_i0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_self_10 = (30);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0_entering = (2);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t0 = (27);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t1 = (29);
const int mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_to_loop_11 = (28);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE = (0);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_TERMINATED_11 = mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0;
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_i0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_t0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_t1 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_checkLimits_t2 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE = (0);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11 = mod_sys_ctrl_ref0_stm_ref0_disableHV_f0;
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_f0 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_i0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_s0 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_s0_entering = (2);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_s1 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_s1_entering = (1);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_t0 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_t1 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_t2 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_disableHV_t3 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_i0 = (10);
const int mod_sys_ctrl_ref0_stm_ref0_sp1_4 = (9);
const int mod_sys_ctrl_ref0_stm_ref0_sp1_7 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_sp_pj_3 = (16);
const int mod_sys_ctrl_ref0_stm_ref0_sp_pj_6 = (12);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE = (0);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_TERMINATED_11 = mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0;
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0 = (5);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_i0 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_19 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_22 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_18 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_21 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_t0 = (3);
const int mod_sys_ctrl_ref0_stm_ref0_t0 = (4);
const int mod_sys_ctrl_ref0_stm_ref0_t1 = (1);
const int mod_sys_ctrl_ref0_stm_ref0_t2 = (8);
const int mod_sys_ctrl_ref0_stm_ref0_t3 = (7);
const int mod_sys_ctrl_ref0_stm_ref0_t4 = (6);
const int mod_sys_ctrl_ref0_stm_ref0_t5 = (2);
const int mod_sys_ctrl_ref0_stm_ref0_t6 = (3);
const int Action_85 = (5);
const int Action_87 = (7);
const int ext_pow24VStatus_84 = (2);
const int ext_pow24_1_88 = (8);
const int ext_pow24_2_86 = (6);
const int mod_sys_ctrl_ref2_stm0_INACTIVE_10 = (0);
const int mod_sys_ctrl_ref2_stm0_LOCK_FREE = (0);
const int mod_sys_ctrl_ref2_stm0_TERMINATED_11 = (9);
const int mod_sys_ctrl_ref2_stm0_i0 = (3);
const int mod_sys_ctrl_ref2_stm0_s0 = (4);
const int mod_sys_ctrl_ref2_stm0_s0_entering = (1);
const int mod_sys_ctrl_ref2_stm0_t0 = (1);
const int mod_sys_ctrl_ref2_stm0_t1 = (2);
const int Exit_Sub_ACT = (4);
const int Exit_NONE = (0);
const int Exit_ACT_Parent = (1);
const int Exit_EXITED = (3);
const int Exit_ACT_Trans = (2);
const int Exit_Sub_ACT_Waiting = (5);
const int Exit_Sub_EXITED = (6);

// For the asynchronous connection on the event [int_DisableHV] of [ctrl_ref1] ==> the event [int_DisableHV] of [ctrl_ref0]
module BUF_int_DisableHV_16
    BUF_int_DisableHV_16_f : bool init false;

    [OUT_BUF__int_DisableHV_16] true -> (BUF_int_DisableHV_16_f'=true);
    [IN_BUF__int_DisableHV_16] (BUF_int_DisableHV_16_f=true) -> (BUF_int_DisableHV_16_f'=false);
endmodule

// For the asynchronous connection on the event [ext_pow24_1] of [ctrl_ref2] ==> the event [ext_pow24VStatus] of [ctrl_ref0]
module BUF_ext_pow24_1_17
    BUF_ext_pow24_1_17_f : bool init false;
    EVT__IN_BUF__ext_pow24_1_17 : [0..1];
    FIN__OUT_BUF__ext_pow24_1_17 : bool init true;

    [OUT_BUF__ext_pow24_1_17] ((FIN__IN_BUF__ext_pow24_1_17=true))&((FIN__OUT_BUF__ext_pow24_1_17=true)) -> (BUF_ext_pow24_1_17_f'=false)&(FIN__OUT_BUF__ext_pow24_1_17'=false);
    [] (FIN__OUT_BUF__ext_pow24_1_17=false) -> (EVT__IN_BUF__ext_pow24_1_17'=EVT__OUT_BUF__ext_pow24_1_17)&(FIN__OUT_BUF__ext_pow24_1_17'=true)&(BUF_ext_pow24_1_17_f'=true);
    [IN_BUF__ext_pow24_1_17] (BUF_ext_pow24_1_17_f=true) -> (BUF_ext_pow24_1_17_f'=false);
endmodule

// For the asynchronous connection on the event [int_underLimit] of [ctrl_ref0] ==> the event [int_underLimit] of [ctrl_ref1]
module BUF_int_underLimit_20
    BUF_int_underLimit_20_f : bool init false;
    EVT__IN_BUF__int_underLimit_20 : [0..2];
    FIN__OUT_BUF__int_underLimit_20 : bool init true;

    [OUT_BUF__int_underLimit_20] ((FIN__IN_BUF__int_underLimit_20=true))&((FIN__OUT_BUF__int_underLimit_20=true)) -> (FIN__OUT_BUF__int_underLimit_20'=false)&(BUF_int_underLimit_20_f'=false);
    [] (FIN__OUT_BUF__int_underLimit_20=false) -> (FIN__OUT_BUF__int_underLimit_20'=true)&(EVT__IN_BUF__int_underLimit_20'=EVT__OUT_BUF__int_underLimit_20)&(BUF_int_underLimit_20_f'=true);
    [IN_BUF__int_underLimit_20] (BUF_int_underLimit_20_f=true) -> (BUF_int_underLimit_20_f'=false);
endmodule

// For the state machine [Watchdog] in the controller [ctrl1]
module mod_sys_ctrl_ref1_stm_ref0
    FIN__IN_BUF__ActualHV_1_19 : bool init true;
    mod_sys_ctrl_ref1_stm_ref0_overLimit : [0..2];
    mod_sys_ctrl_ref1_stm_ref0_underLimit : [0..2];
    mod_sys_ctrl_ref1_stm_ref0_ActualHV : [0..2];
    mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8 : [0..17] init mod_sys_ctrl_ref1_stm_ref0_s0_INACTIVE_10;
    mod_sys_ctrl_ref1_stm_ref0_power : [0..1];
    FIN__IN_BUF__int_underLimit_20 : bool init true;
    FIN__IN_BUF__ext_pow24_2_25 : bool init true;
    mod_sys_ctrl_ref1_stm_ref0_exit_9 : [0..6] init Exit_NONE;
    FIN__IN_BUF__int_overLimit_22 : bool init true;
    mod_sys_ctrl_ref1_stm_ref0_lock_7 : [0..13] init mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE;
    mod_sys_ctrl_ref1_stm_ref0_scpc_8 : [0..6] init mod_sys_ctrl_ref1_stm_ref0_i0;
    mod_sys_ctrl_ref1_stm_ref0_s0_exit_9 : [0..6] init Exit_NONE;

    
    // Step 7: make sure the composite source state [s0] has been exited.
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_t1)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref1_stm_ref0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0)&(mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_i0);
    
    // Step 4/2: exit command of the state [f0]
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_f0)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t7] from a state [ActualHVRead] to [PowerAndActualHVRead].Step 1: trigger an exit of the state [ActualHVRead]
    [IN_BUF__ext_pow24_2_25] ((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((FIN__IN_BUF__ext_pow24_2_25=true))&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=ext_pow24VStatus_76)&(FIN__IN_BUF__ext_pow24_2_25'=false)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t7);
    
    // The transition [t5] from a state [Waiting] to [ActualHVRead].Step 1: trigger an exit of the state [Waiting]
    [IN_BUF__ActualHV_1_19] ((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_Waiting))&(((FIN__IN_BUF__ActualHV_1_19=true))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)))) -> (FIN__IN_BUF__ActualHV_1_19'=false)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=int_ActualHV_74)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t5);
    
    // The transition [t1] from a state [s0] to [s0].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_t1)&(mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_ACT_Trans);
    
    // The transition [t9] from a state [PowerAndActualHVRead] to [f0].Step 1: trigger an exit of the state [PowerAndActualHVRead]
    [] ((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead))&(mod_sys_ctrl_ref1_stm_ref0_ActualHV>mod_sys_ctrl_ref1_stm_ref0_overLimit))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_83)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t9);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_77)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE);
    
    // The transition [t4] from a state [Waiting] to [PowerStatusRead].Step 1: trigger an exit of the state [Waiting]
    [IN_BUF__ext_pow24_2_25] ((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_Waiting))&(((FIN__IN_BUF__ext_pow24_2_25=true))&(((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)))) -> (FIN__IN_BUF__ext_pow24_2_25'=false)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=ext_pow24VStatus_75)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t4);
    
    // Step 6: if the composite state [s0] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref1_stm_ref0_exit_9=Exit_Sub_ACT_Waiting)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref1_stm_ref0_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t6] from a state [PowerStatusRead] to [PowerAndActualHVRead].Step 1: trigger an exit of the state [PowerStatusRead]
    [IN_BUF__ActualHV_1_19] ((FIN__IN_BUF__ActualHV_1_19=true))&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead))&(((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=int_ActualHV_79)&(FIN__IN_BUF__ActualHV_1_19'=false)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t6);
    
    // Step 3: trigger an exit of the composite state [s0]
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_exit_9'=Exit_Sub_ACT_Waiting)&(mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_ACT_Parent);
    
    // The transition [t0] from [i0] to [s0].
    [] (mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_i0) -> (mod_sys_ctrl_ref1_stm_ref0_scpc_8'=Action_72)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_t0);
    
    // The transition [t1] from a state [ActualHVRead] to [f0].Step 1: trigger an exit of the state [ActualHVRead]
    [] (mod_sys_ctrl_ref1_stm_ref0_ActualHV>mod_sys_ctrl_ref1_stm_ref0_overLimit)&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead))&(((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_78)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t1);
    
    // The transition [t6] from a state [PowerStatusRead] to [PowerAndActualHVRead].Step 1: trigger an exit of the state [PowerStatusRead]
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=int_ActualHV_79)) -> (mod_sys_ctrl_ref1_stm_ref0_ActualHV'=EVT__IN_BUF__ActualHV_1_19)&(FIN__IN_BUF__ActualHV_1_19'=true)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead);
    
    // The transition [t10] from a state [PowerAndActualHVRead] to [f0].Step 1: trigger an exit of the state [PowerAndActualHVRead]
    [] ((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&(((mod_sys_ctrl_ref1_stm_ref0_power=Power_Off))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead)))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_81)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t10);
    
    // The transition [t8] from a state [PowerAndActualHVRead] to [f0].Step 1: trigger an exit of the state [PowerAndActualHVRead]
    [] (mod_sys_ctrl_ref1_stm_ref0_ActualHV<mod_sys_ctrl_ref1_stm_ref0_underLimit)&(((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)))) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t8)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_82);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_82))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0);
    
    // The transition [t5] from a state [Waiting] to [ActualHVRead].Step 1: trigger an exit of the state [Waiting]
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=int_ActualHV_74)) -> (FIN__IN_BUF__ActualHV_1_19'=true)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead)&(mod_sys_ctrl_ref1_stm_ref0_ActualHV'=EVT__IN_BUF__ActualHV_1_19)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE);
    
    // Step 5: check if all substates of the composite state [s0] are exited
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_EXITED)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_INACTIVE_10);
    
    // Step 4/2: exit command of the state [PowerAndActualHVRead]
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_EXITED);
    
    // Step 2: the state [s0] is going to exit by setting its substates to exit firstly.
    [] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&(((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_ACT);
    
    // The transition [t2] from a state [PowerStatusRead] to [f0].Step 1: trigger an exit of the state [PowerStatusRead]
    [] ((mod_sys_ctrl_ref1_stm_ref0_power=Power_Off))&(((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead)))) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t2)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_80);
    [] (mod_sys_ctrl_ref1_stm_ref0_scpc_8=int_overLimit_73) -> (mod_sys_ctrl_ref1_stm_ref0_overLimit'=EVT__IN_BUF__int_overLimit_22)&(FIN__IN_BUF__int_overLimit_22'=true)&(mod_sys_ctrl_ref1_stm_ref0_scpc_8'=Action_70);
    
    // Step 4/2: exit command of the state [PowerStatusRead]
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t0] from [i0] to [Waiting].
    [] ((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_i0))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_Waiting);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_83)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_80)) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0);
    
    // Step 4/2: exit command of the state [Waiting]
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_Waiting)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_EXITED);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_81)) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0);
    
    // Step 4/2: exit command of the state [ActualHVRead]
    [] (((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead)))&((mod_sys_ctrl_ref1_stm_ref0_s0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_exit_9'=Exit_Sub_EXITED);
    [OUT_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=Action_78)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_f0)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE);
    
    // The transition [t4] from a state [Waiting] to [PowerStatusRead].Step 1: trigger an exit of the state [Waiting]
    [] ((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=ext_pow24VStatus_75))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE)&(FIN__IN_BUF__ext_pow24_2_25'=true)&(mod_sys_ctrl_ref1_stm_ref0_power'=EVT__IN_BUF__ext_pow24_2_25);
    
    // The transition [t7] from a state [ActualHVRead] to [PowerAndActualHVRead].Step 1: trigger an exit of the state [ActualHVRead]
    [] ((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=ext_pow24VStatus_76))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0)) -> (mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead)&(mod_sys_ctrl_ref1_stm_ref0_power'=EVT__IN_BUF__ext_pow24_2_25)&(FIN__IN_BUF__ext_pow24_2_25'=true)&(mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE);
    [IN_BUF__int_underLimit_20] ((FIN__IN_BUF__int_underLimit_20=true))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=Action_70)) -> (mod_sys_ctrl_ref1_stm_ref0_scpc_8'=int_underLimit_71)&(FIN__IN_BUF__int_underLimit_20'=false);
    [IN_BUF__int_overLimit_22] ((FIN__IN_BUF__int_overLimit_22=true))&((mod_sys_ctrl_ref1_stm_ref0_scpc_8=Action_72)) -> (mod_sys_ctrl_ref1_stm_ref0_scpc_8'=int_overLimit_73)&(FIN__IN_BUF__int_overLimit_22'=false);
    
    // The transition [t3] from a state [ActualHVRead] to [f0].Step 1: trigger an exit of the state [ActualHVRead]
    [] (mod_sys_ctrl_ref1_stm_ref0_ActualHV<mod_sys_ctrl_ref1_stm_ref0_underLimit)&(((mod_sys_ctrl_ref1_stm_ref0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0))&(((mod_sys_ctrl_ref1_stm_ref0_lock_7=mod_sys_ctrl_ref1_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8=mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead)))) -> (mod_sys_ctrl_ref1_stm_ref0_lock_7'=mod_sys_ctrl_ref1_stm_ref0_s0_t3)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=Action_77);
    [] (mod_sys_ctrl_ref1_stm_ref0_scpc_8=int_underLimit_71) -> (mod_sys_ctrl_ref1_stm_ref0_underLimit'=EVT__IN_BUF__int_underLimit_20)&(FIN__IN_BUF__int_underLimit_20'=true)&(mod_sys_ctrl_ref1_stm_ref0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0)&(mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8'=mod_sys_ctrl_ref1_stm_ref0_s0_i0);
endmodule

// For the state machine [stm0] in the controller [ctrl3]
module mod_sys_ctrl_ref3_stm0
    mod_sys_ctrl_ref3_stm0_exit_9 : [0..6] init Exit_NONE;
    EVT__OUT_BUF__ActualHV_1_19 : [0..2];
    FIN__IN_BUF__int_ActualHV_27 : bool init true;
    mod_sys_ctrl_ref3_stm0_scpc_8 : [0..7] init mod_sys_ctrl_ref3_stm0_i0;
    mod_sys_ctrl_ref3_stm0_ActualHV : [0..2];
    mod_sys_ctrl_ref3_stm0_lock_7 : [0..2] init mod_sys_ctrl_ref3_stm0_LOCK_FREE;
    EVT__RP__ActualHV_2_21 : [0..2];

    [OUT_BUF__ActualHV_1_19] (mod_sys_ctrl_ref3_stm0_scpc_8=Action_68) -> (mod_sys_ctrl_ref3_stm0_scpc_8'=ActualHV_1_69)&(EVT__OUT_BUF__ActualHV_1_19'=mod_sys_ctrl_ref3_stm0_ActualHV);
    
    // Step 4/2: exit command of the state [s0]
    [] ((mod_sys_ctrl_ref3_stm0_scpc_8=mod_sys_ctrl_ref3_stm0_s0))&((mod_sys_ctrl_ref3_stm0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref3_stm0_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t0] from [i0] to [s0].
    [] (mod_sys_ctrl_ref3_stm0_scpc_8=mod_sys_ctrl_ref3_stm0_i0) -> (mod_sys_ctrl_ref3_stm0_scpc_8'=mod_sys_ctrl_ref3_stm0_s0_entering);
    [] ((mod_sys_ctrl_ref3_stm0_scpc_8=ActualHV_1_69))&((FIN__OUT_BUF__ActualHV_1_19=true)) -> (mod_sys_ctrl_ref3_stm0_scpc_8'=Action_67);
    [] (mod_sys_ctrl_ref3_stm0_scpc_8=int_ActualHV_66) -> (mod_sys_ctrl_ref3_stm0_scpc_8'=mod_sys_ctrl_ref3_stm0_s0)&(mod_sys_ctrl_ref3_stm0_ActualHV'=EVT__IN_BUF__int_ActualHV_27)&(FIN__IN_BUF__int_ActualHV_27'=true)&(mod_sys_ctrl_ref3_stm0_lock_7'=mod_sys_ctrl_ref3_stm0_LOCK_FREE);
    [RP__ActualHV_2_21] (mod_sys_ctrl_ref3_stm0_scpc_8=Action_67) -> (mod_sys_ctrl_ref3_stm0_scpc_8'=mod_sys_ctrl_ref3_stm0_s0_entering)&(EVT__RP__ActualHV_2_21'=mod_sys_ctrl_ref3_stm0_ActualHV);
    [IN_BUF__int_ActualHV_27] ((mod_sys_ctrl_ref3_stm0_scpc_8=mod_sys_ctrl_ref3_stm0_s0_entering))&((FIN__IN_BUF__int_ActualHV_27=true)) -> (FIN__IN_BUF__int_ActualHV_27'=false)&(mod_sys_ctrl_ref3_stm0_scpc_8'=int_ActualHV_66);
    
    // The transition [t1] from a state [s0] to [s0].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref3_stm0_scpc_8=mod_sys_ctrl_ref3_stm0_s0))&((mod_sys_ctrl_ref3_stm0_lock_7=mod_sys_ctrl_ref3_stm0_LOCK_FREE)) -> (mod_sys_ctrl_ref3_stm0_lock_7'=mod_sys_ctrl_ref3_stm0_t1)&(mod_sys_ctrl_ref3_stm0_scpc_8'=Action_68);
endmodule

// For the asynchronous connection on the event [ext_pow24_2] of [ctrl_ref2] ==> the event [ext_pow24VStatus] of [ctrl_ref1]
module BUF_ext_pow24_2_25
    BUF_ext_pow24_2_25_f : bool init false;
    EVT__IN_BUF__ext_pow24_2_25 : [0..1];
    FIN__OUT_BUF__ext_pow24_2_25 : bool init true;

    [OUT_BUF__ext_pow24_2_25] ((FIN__IN_BUF__ext_pow24_2_25=true))&((FIN__OUT_BUF__ext_pow24_2_25=true)) -> (BUF_ext_pow24_2_25_f'=false)&(FIN__OUT_BUF__ext_pow24_2_25'=false);
    [] (FIN__OUT_BUF__ext_pow24_2_25=false) -> (BUF_ext_pow24_2_25_f'=true)&(EVT__IN_BUF__ext_pow24_2_25'=EVT__OUT_BUF__ext_pow24_2_25)&(FIN__OUT_BUF__ext_pow24_2_25'=true);
    [IN_BUF__ext_pow24_2_25] (BUF_ext_pow24_2_25_f=true) -> (BUF_ext_pow24_2_25_f'=false);
endmodule


// For the asynchronous connection on the event [int_ActualHV] of [ctrl_ref0] ==> the event [int_ActualHV] of [ctrl_ref3]
module BUF_int_ActualHV_27
    BUF_int_ActualHV_27_f : bool init false;
    EVT__IN_BUF__int_ActualHV_27 : [0..2];
    FIN__OUT_BUF__int_ActualHV_27 : bool init true;

    [OUT_BUF__int_ActualHV_27] ((FIN__IN_BUF__int_ActualHV_27=true))&((FIN__OUT_BUF__int_ActualHV_27=true)) -> (FIN__OUT_BUF__int_ActualHV_27'=false)&(BUF_int_ActualHV_27_f'=false);
    [] (FIN__OUT_BUF__int_ActualHV_27=false) -> (FIN__OUT_BUF__int_ActualHV_27'=true)&(BUF_int_ActualHV_27_f'=true)&(EVT__IN_BUF__int_ActualHV_27'=EVT__OUT_BUF__int_ActualHV_27);
    [IN_BUF__int_ActualHV_27] (BUF_int_ActualHV_27_f=true) -> (BUF_int_ActualHV_27_f'=false);
endmodule

// For the asynchronous connection on the event [int_overLimit] of [ctrl_ref0] ==> the event [int_overLimit] of [ctrl_ref1]
module BUF_int_overLimit_22
    BUF_int_overLimit_22_f : bool init false;
    EVT__IN_BUF__int_overLimit_22 : [0..2];
    FIN__OUT_BUF__int_overLimit_22 : bool init true;

    [OUT_BUF__int_overLimit_22] ((FIN__IN_BUF__int_overLimit_22=true))&((FIN__OUT_BUF__int_overLimit_22=true)) -> (BUF_int_overLimit_22_f'=false)&(FIN__OUT_BUF__int_overLimit_22'=false);
    [] (FIN__OUT_BUF__int_overLimit_22=false) -> (FIN__OUT_BUF__int_overLimit_22'=true)&(EVT__IN_BUF__int_overLimit_22'=EVT__OUT_BUF__int_overLimit_22)&(BUF_int_overLimit_22_f'=true);
    [IN_BUF__int_overLimit_22] (BUF_int_overLimit_22_f=true) -> (BUF_int_overLimit_22_f'=false);
endmodule

// For the state machine [stm0] in the controller [ctrl2]
module mod_sys_ctrl_ref2_stm0
    FIN__RP__ext_pow24VStatus_28 : bool init true;
    mod_sys_ctrl_ref2_stm0_power : [0..1];
    mod_sys_ctrl_ref2_stm0_lock_7 : [0..2] init mod_sys_ctrl_ref2_stm0_LOCK_FREE;
    EVT__OUT_BUF__ext_pow24_2_25 : [0..1];
    EVT__OUT_BUF__ext_pow24_1_17 : [0..1];
    mod_sys_ctrl_ref2_stm0_scpc_8 : [0..8] init mod_sys_ctrl_ref2_stm0_i0;
    mod_sys_ctrl_ref2_stm0_exit_9 : [0..6] init Exit_NONE;
    
    // Step 4/2: exit command of the state [s0]
    [] ((mod_sys_ctrl_ref2_stm0_scpc_8=mod_sys_ctrl_ref2_stm0_s0))&((mod_sys_ctrl_ref2_stm0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref2_stm0_exit_9'=Exit_Sub_EXITED);
    [] ((FIN__OUT_BUF__ext_pow24_2_25=true))&((mod_sys_ctrl_ref2_stm0_scpc_8=ext_pow24_2_86)) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=mod_sys_ctrl_ref2_stm0_s0_entering);
    [] ((FIN__OUT_BUF__ext_pow24_1_17=true))&((mod_sys_ctrl_ref2_stm0_scpc_8=ext_pow24_1_88)) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=Action_85);
    [OUT_BUF__ext_pow24_1_17] (mod_sys_ctrl_ref2_stm0_scpc_8=Action_87) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=ext_pow24_1_88)&(EVT__OUT_BUF__ext_pow24_1_17'=mod_sys_ctrl_ref2_stm0_power);
    [OUT_BUF__ext_pow24_2_25] (mod_sys_ctrl_ref2_stm0_scpc_8=Action_85) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=ext_pow24_2_86)&(EVT__OUT_BUF__ext_pow24_2_25'=mod_sys_ctrl_ref2_stm0_power);
    
    // The transition [t0] from [i0] to [s0].
    [] (mod_sys_ctrl_ref2_stm0_scpc_8=mod_sys_ctrl_ref2_stm0_i0) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=mod_sys_ctrl_ref2_stm0_s0_entering);
    
    // The transition [t1] from a state [s0] to [s0].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref2_stm0_scpc_8=mod_sys_ctrl_ref2_stm0_s0))&((mod_sys_ctrl_ref2_stm0_lock_7=mod_sys_ctrl_ref2_stm0_LOCK_FREE)) -> (mod_sys_ctrl_ref2_stm0_lock_7'=mod_sys_ctrl_ref2_stm0_t1)&(mod_sys_ctrl_ref2_stm0_scpc_8'=Action_87);
    [RP__ext_pow24VStatus_28] ((mod_sys_ctrl_ref2_stm0_scpc_8=mod_sys_ctrl_ref2_stm0_s0_entering))&((FIN__RP__ext_pow24VStatus_28=true)) -> (mod_sys_ctrl_ref2_stm0_scpc_8'=ext_pow24VStatus_84)&(FIN__RP__ext_pow24VStatus_28'=false);
    [] (mod_sys_ctrl_ref2_stm0_scpc_8=ext_pow24VStatus_84) -> (mod_sys_ctrl_ref2_stm0_lock_7'=mod_sys_ctrl_ref2_stm0_LOCK_FREE)&(FIN__RP__ext_pow24VStatus_28'=true)&(mod_sys_ctrl_ref2_stm0_power'=EVT__RP__ext_pow24VStatus_28)&(mod_sys_ctrl_ref2_stm0_scpc_8'=mod_sys_ctrl_ref2_stm0_s0);
endmodule

// For the asynchronous connection on the event [ActualHV_1] of [ctrl_ref3] ==> the event [int_ActualHV] of [ctrl_ref1]
module BUF_ActualHV_1_19
    BUF_ActualHV_1_19_f : bool init false;
    EVT__IN_BUF__ActualHV_1_19 : [0..2];
    FIN__OUT_BUF__ActualHV_1_19 : bool init true;

    [OUT_BUF__ActualHV_1_19] ((FIN__IN_BUF__ActualHV_1_19=true))&((FIN__OUT_BUF__ActualHV_1_19=true)) -> (FIN__OUT_BUF__ActualHV_1_19'=false)&(BUF_ActualHV_1_19_f'=false);
    [] (FIN__OUT_BUF__ActualHV_1_19=false) -> (EVT__IN_BUF__ActualHV_1_19'=EVT__OUT_BUF__ActualHV_1_19)&(FIN__OUT_BUF__ActualHV_1_19'=true)&(BUF_ActualHV_1_19_f'=true);
    [IN_BUF__ActualHV_1_19] (BUF_ActualHV_1_19_f=true) -> (BUF_ActualHV_1_19_f'=false);
endmodule


// For the state machine [State_machine] in the controller [ctrl0]
module mod_sys_ctrl_ref0_stm_ref0
    mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_setPoint : [0..2];
    mod_sys_ctrl_ref0_stm_ref0_scpc_8 : [0..22] init mod_sys_ctrl_ref0_stm_ref0_i0;
    mod_sys_ctrl_ref0_stm_ref0_disableHV_arg : bool;
    mod_sys_ctrl_ref0_stm_ref0_lock_7 : [0..30] init mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE;
    mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8 : [0..11] init mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_errorFlag : bool;
    EVT__OUT_BUF__int_overLimit_22 : [0..2];
    mod_sys_ctrl_ref0_stm_ref0_Init_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_ActualHV : [0..2];
    EVT__RP__int_pwmSignal_18 : [0..1];
    mod_sys_ctrl_ref0_stm_ref0_power : [0..1];
    mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8 : [0..11] init mod_sys_ctrl_ref0_stm_ref0_Init_INACTIVE_10;
    EVT__RP__ext_setPoint_24 : [0..2];
    mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8 : [0..13] init mod_sys_ctrl_ref0_stm_ref0_ErrorMode_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7 : [0..4] init mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE;
    mod_sys_ctrl_ref0_stm_ref0_underLimit : [0..2];
    mod_sys_ctrl_ref0_stm_ref0_errorAck : bool init false;
    mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8 : [0..8] init mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_overLimit : [0..2];
    FIN__IN_BUF__ext_pow24_1_17 : bool init true;
    mod_sys_ctrl_ref0_stm_ref0_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_res : bool init false;
    mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7 : [0..3] init mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE;
    mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8 : [0..6] init mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8 : [0..9] init mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_power : [0..1];
    mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8 : [0..10] init mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10;
    mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8 : [0..10] init mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_INACTIVE_10;
    EVT__OUT_BUF__int_underLimit_20 : [0..2];
    mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9 : [0..6] init Exit_NONE;
    EVT__RP__currentState_26 : [0..4];
    EVT__OUT_BUF__int_ActualHV_27 : [0..2];
    FIN__RP__ext_setPoint_23 : bool init true;
    mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9 : [0..6] init Exit_NONE;
    mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7 : [0..3] init mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE;
    mod_sys_ctrl_ref0_stm_ref0_supplyLim : bool init false;
    mod_sys_ctrl_ref0_stm_ref0_lim : bool init false;

    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_si0_entering)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=Action_50)&(mod_sys_ctrl_ref0_stm_ref0_overLimit'=#CALLFUNC!overLimitF$(mod_sys_ctrl_ref0_stm_ref0_setPoint+(2))#);
    
    // The transition [t0] from a state [s1] to [s2].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1))) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=Action_56)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t0);
    
    // Call operation [disableHV]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1_entering)))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=true)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1_entering)) -> (mod_sys_ctrl_ref0_stm_ref0_lim'=false)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=Action_58);
    
    // Call operation [checkLimits]
    [] ((((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0_entering)))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_i0);
    [OUT_BUF__int_underLimit_20] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=Action_48)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=int_underLimit_49)&(EVT__OUT_BUF__int_underLimit_20'=mod_sys_ctrl_ref0_stm_ref0_underLimit);
    
    // Step 7: make sure the composite source state [Wait24Vpower] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_t2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_43)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_NONE);
    [RP__currentState_26] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Ramping_entering) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Ramping)&(EVT__RP__currentState_26'=State_Ramp)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // Call operation [disableHV]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=Action_58)))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=true);
    
    // The transition [t0] from [i0] to [si0].
    [] ((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_i0))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=Action_64)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t0);
    
    // The transition [t1] from a state [si0] to [s1].Step 1: trigger an exit of the state [si0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((((mod_sys_ctrl_ref0_stm_ref0_setPoint!=(0)))|((mod_sys_ctrl_ref0_stm_ref0_lim=true)))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)))) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1_entering)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t1);
    [] (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=Action_34) -> (mod_sys_ctrl_ref0_stm_ref0_lim'=true)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0);
    
    // The transition [t4] from a state [s2] to [j0].Step 1: trigger an exit of the state [s2]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t4)&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=Action_53);
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_44) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering)&(mod_sys_ctrl_ref0_stm_ref0_res'=false);
    
    // Step 4/2: exit command of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9'=Exit_Sub_EXITED);
    
    // Step 4/2: exit command of the state [f0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_f0))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9'=Exit_Sub_EXITED);
    
    // Wait for operation [disableHV] to exit.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_39))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_i0);
    
    // Step 6: if the composite state [s2] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT_Waiting))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_EXITED)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_NONE);
    
    // The transition [t3] from a state [s0] to [s1].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s0))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_t3)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1_entering);
    
    // The transition [to_loop_17] from a state [si0] to [loop_15].Step 1: trigger an exit of the state [si0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_si0))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_loop_15);
    
    // Step 2: the state [ClosedLoop] is going to exit by setting its substates to exit firstly.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_ACT);
    [OUT_BUF__int_ActualHV_27] ((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=Action_53))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=int_ActualHV_54)&(EVT__OUT_BUF__int_ActualHV_27'=mod_sys_ctrl_ref0_stm_ref0_ActualHV);
    
    // The transition [t3] from a state [s4] to [s3].Step 1: trigger an exit of the state [s4]
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lim=false)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=Action_59)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t3);
    
    // The transition [t0] from [i0] to [s4].
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_i0))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4_entering);
    
    // Step 5: check if all substates of the composite state [s2] are exited
    [] (((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_EXITED);
    
    // Step 3: trigger an exit of the composite state [Wait24Vpower]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_ACT_Waiting)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_ACT_Parent);
    
    // Step 4/2: exit command of the state [s4]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_EXITED);
    
    // Step 4/2: exit command of the state [s1]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1)))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_Sub_EXITED);
    
    // Step 4/2: exit command of the state [s1]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_Sub_EXITED);
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_42) -> (mod_sys_ctrl_ref0_stm_ref0_res'=false)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering);
    
    // Wait for operation [supplyVoltCheck] to exit.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0_entering)))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0);
    
    // Step 7: make sure the composite source state [Wait24Vpower] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_t3))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_entering);
    
    // Step 4/2: exit command of the state [s2]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2)))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_Sub_EXITED);
    
    // Call operation [supplyVoltCheck]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4_entering)))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_i0);
    
    // Step 4/2: exit command of the state [s0]
    [] ((((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_Sub_EXITED);
    [RP__ext_setPoint_23] ((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=Action_64))&(((FIN__RP__ext_setPoint_23=true))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))) -> (FIN__RP__ext_setPoint_23'=false)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=ext_setPoint_65);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=Action_55)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_f0)&(mod_sys_ctrl_ref0_stm_ref0_errorAck'=true)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // Step 3: trigger an exit of the composite state [s2]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_ACT_Waiting)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_ACT_Parent);
    
    // Step 7: make sure the composite source state [Wait24Vpower] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_sp1_4)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_sp_pj_3);
    
    // From the probabilistic junction [sp_pj_6]
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_sp_pj_6) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_41);
    
    // The transition [sp1_19] from a state [s0] to [sp_pj_18].Step 1: trigger an exit of the state [s0]
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=ext_pow24VStatus_32) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_18)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_power'=EVT__IN_BUF__ext_pow24_1_17)&(FIN__IN_BUF__ext_pow24_1_17'=true);
    [RP__int_pwmSignal_18] (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s0_entering) -> (EVT__RP__int_pwmSignal_18'=Power_Off)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_s0)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE);
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=int_ActualHV_62))&((FIN__OUT_BUF__int_ActualHV_27=true)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0_entering);
    [] ((FIN__OUT_BUF__int_underLimit_20=true))&(((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=int_underLimit_49))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=Action_46);
    
    // Step 6: if the composite state [ErrorMode] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT_Waiting)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_EXITED)&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_NONE);
    
    // The transition [sp1_7] from a state [ClosedLoop] to [sp_pj_6].Step 1: trigger an exit of the state [ClosedLoop]
    [IN_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_sp1_7);
    
    // Step 6: if the composite state [Init] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT_Waiting)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_EXITED);
    
    // Step 5: check if all substates of the composite state [Init] are exited
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_EXITED);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2_entering)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2)&(mod_sys_ctrl_ref0_stm_ref0_ActualHV'=(0))&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1_entering)) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_setPoint'=(0))&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1);
    
    // The transition [t2] from a state [s2] to [s1].Step 1: trigger an exit of the state [s2]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lim=true))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t2);
    
    // The transition [t0] from [i0] to [s1].
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_i0))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1_entering);
    
    // Step 4/2: exit command of the state [s3]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_EXITED);
    
    // Step 5: check if all substates of the composite state [Wait24Vpower] are exited
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_EXITED);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=Action_57)) -> (mod_sys_ctrl_ref0_stm_ref0_errorAck'=false)&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1_entering);
    [RP__currentState_26] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_entering) -> (EVT__RP__currentState_26'=State_ClosedLoop)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_i0);
    
    // The transition [t4] from a state [s3] to [s2].Step 1: trigger an exit of the state [s3]
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3))&((((mod_sys_ctrl_ref0_stm_ref0_lim=false))&(true))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_i0)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t4)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2);
    
    // Wait for operation [checkLimits] to exit.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3_entering)))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3);
    
    // The transition [t2] from a state [s1] to [j0].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1)))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t2)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=Action_63);
    
    // Wait for operation [disableHV] to exit.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_37))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_i0);
    
    // Step 6: if the composite state [ClosedLoop] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT_Waiting)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_EXITED)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_NONE);
    
    // Step 4/2: exit command of the state [si0]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_Sub_EXITED);
    
    // Wait for operation [disableHV] to exit.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_41))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_40)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10);
    
    // The transition [t5] from a junction [j0] to [f0].
    [] (((mod_sys_ctrl_ref0_stm_ref0_setPoint=(0)))&((mod_sys_ctrl_ref0_stm_ref0_ActualHV=(0))))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_j0))) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=Action_55);
    
    // Wait for operation [disableHV] to exit.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_45))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_44);
    [OUT_BUF__int_overLimit_22] ((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=Action_46))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init)) -> (EVT__OUT_BUF__int_overLimit_22'=mod_sys_ctrl_ref0_stm_ref0_overLimit)&(mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=int_overLimit_47);
    
    // The transition [sp1_4] from a state [Wait24Vpower] to [sp_pj_3].Step 1: trigger an exit of the state [Wait24Vpower]
    [IN_BUF__int_DisableHV_16] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_sp1_4);
    
    // The transition [t1] from a junction [j0] to [s1].
    [] (((mod_sys_ctrl_ref0_stm_ref0_setPoint!=(0)))|((mod_sys_ctrl_ref0_stm_ref0_ActualHV!=(0))))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_j0))) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1_entering);
    
    // Step 2: the state [Wait24Vpower] is going to exit by setting its substates to exit firstly.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&(((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_Sub_ACT);
    
    // The transition [t2] from a state [s0] to [f0].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_ActualHV<mod_sys_ctrl_ref0_stm_ref0_underLimit)|(mod_sys_ctrl_ref0_stm_ref0_ActualHV>mod_sys_ctrl_ref0_stm_ref0_overLimit))&(((mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7=mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0))) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=Action_34)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_t2);
    
    // Step 3: trigger an exit of the composite state [Init]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_ACT_Parent)&(mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_ACT_Waiting);
    [RP__int_pwmSignal_18] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=Action_59))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3_entering)&(EVT__RP__int_pwmSignal_18'=Power_On);
    
    // Step 5: check if all substates of the composite state [ErrorMode] are exited
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_EXITED);
    
    // The transition [t0] from a state [Ramping] to [Init].Step 1: trigger an exit of the state [Ramping]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Ramping))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t0)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_entering);
    
    // Step 6: if the composite state [Wait24Vpower] has been exited and its parent state required exit, then its parent state's substate has exited too
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_EXITED)))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT_Waiting)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_EXITED)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_NONE);
    [] (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=Action_35) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_f0)&(mod_sys_ctrl_ref0_stm_ref0_res'=false)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE);
    
    // Step 7: make sure the composite source state [ClosedLoop] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_t4)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_42);
    
    // The transition [t1] from a state [s1] to [f0].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1))&(((mod_sys_ctrl_ref0_stm_ref0_disableHV_arg=true))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=Action_36)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_t1);
    
    // The transition [t2] from a state [Wait24Vpower] to [ErrorMode].Step 1: trigger an exit of the state [Wait24Vpower]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&(((mod_sys_ctrl_ref0_stm_ref0_res=true))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t2);
    
    // Step 7: make sure the composite source state [Init] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_t1)))&((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_entering)&(mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_NONE);
    
    // The transition [t1] from a state [s4] to [s1].Step 1: trigger an exit of the state [s4]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&(((mod_sys_ctrl_ref0_stm_ref0_lim=true))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1_entering)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t1);
    
    // Step 4/2: exit command of the state [s1]
    [] ((((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_Sub_EXITED);
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=Action_50)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=Action_48)&(mod_sys_ctrl_ref0_stm_ref0_underLimit'=#CALLFUNC!underLimitF$(mod_sys_ctrl_ref0_stm_ref0_setPoint-(2))#);
    
    // The transition [loop_self_13] from a state [loop_12] to [loop_12].Step 1: trigger an exit of the state [loop_12]
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12);
    
    // Step 4/2: exit command of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9'=Exit_Sub_EXITED);
    
    // Step 2: the state [Init] is going to exit by setting its substates to exit firstly.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&(((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_Sub_ACT);
    
    // Step 3: trigger an exit of the composite state [ClosedLoop]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_ACT_Waiting)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_ACT_Parent);
    
    // Step 4/2: exit command of the state [Ramping]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Ramping))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_EXITED);
    
    // Wait for operation [disableHV] to exit.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=Action_58)))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // The transition [t0] from [i0] to [s0].
    [] (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_i0) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0);
    
    // Step 4/2: exit command of the state [loop_12]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_EXITED);
    
    // The transition [loop_self_10] from a state [loop_9] to [loop_9].Step 1: trigger an exit of the state [loop_9]
    [] ((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9);
    
    // Call operation [disableHV]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_39))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=false);
    
    // Call operation [disableHV]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_38))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=false)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0);
    
    // The transition [t0] from [i0] to [si0].
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_i0)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=Action_51)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_Init_t0);
    
    // The transition [t0] from [i0] to [s0].
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_i0) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0);
    
    // Step 4/2: exit command of the state [f0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9'=Exit_Sub_EXITED);
    [] (((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=Action_63)) -> (mod_sys_ctrl_ref0_stm_ref0_ActualHV'=mod_sys_ctrl_ref0_stm_ref0_setPoint)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_j0);
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_43) -> (mod_sys_ctrl_ref0_stm_ref0_res'=false)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering);
    
    // The transition [t6] from a state [ErrorMode] to [Wait24Vpower].Step 1: trigger an exit of the state [ErrorMode]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(mod_sys_ctrl_ref0_stm_ref0_errorAck)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t6);
    
    // Wait for operation [disableHV] to exit.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_38))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_i0);
    [] ((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=ext_setPoint_52))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_si0_entering)&(mod_sys_ctrl_ref0_stm_ref0_setPoint'=EVT__RP__ext_setPoint_23)&(FIN__RP__ext_setPoint_23'=true);
    
    // The transition [t2] from [i0] to [s1].
    [] ((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_i0))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=Action_57)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t2);
    
    // The transition [sp1_22] from a state [s0] to [sp_pj_21].Step 1: trigger an exit of the state [s0]
    [IN_BUF__ext_pow24_1_17] ((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0))&(((FIN__IN_BUF__ext_pow24_1_17=true))&(((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE))&((EVT__IN_BUF__ext_pow24_1_17=Power_Off)))) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_22)&(FIN__IN_BUF__ext_pow24_1_17'=false)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=ext_pow24VStatus_31);
    [] (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=Action_33) -> (mod_sys_ctrl_ref0_stm_ref0_lim'=false)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0);
    
    // Call operation [disableHV]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_45))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=true)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0);
    
    // Step 2: the state [s2] is going to exit by setting its substates to exit firstly.
    [] (((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_Sub_ACT);
    [RP__currentState_26] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_38)&(EVT__RP__currentState_26'=State_ErrorMode);
    
    // Step 4/2: exit command of the state [f0]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_f0)))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_Sub_EXITED);
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=Action_29) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_lim'=true);
    
    // The transition [t3] from a state [Wait24Vpower] to [ClosedLoop].Step 1: trigger an exit of the state [Wait24Vpower]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((((mod_sys_ctrl_ref0_stm_ref0_setPoint=(0)))&((mod_sys_ctrl_ref0_stm_ref0_lim=false)))&((mod_sys_ctrl_ref0_stm_ref0_res=false)))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t3)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_ACT_Trans);
    [] ((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=ext_setPoint_65))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower)) -> (mod_sys_ctrl_ref0_stm_ref0_setPoint'=EVT__RP__ext_setPoint_23)&(FIN__RP__ext_setPoint_23'=true)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0_entering);
    [] (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=Action_36) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_f0)&(mod_sys_ctrl_ref0_stm_ref0_res'=true)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE);
    
    // Call operation [supplyVoltCheck]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0_entering)))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_i0);
    
    // From the probabilistic junction [sp_pj_18]
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_18) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=Action_30);
    
    // Step 4/2: exit command of the state [loop_15]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_loop_15)))&((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_Sub_EXITED);
    
    // Step 7: make sure the composite source state [ClosedLoop] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_sp1_7)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_sp_pj_6);
    
    // Step 4/2: exit command of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9'=Exit_Sub_EXITED);
    [] (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1_entering) -> (mod_sys_ctrl_ref0_stm_ref0_setPoint'=(0))&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE);
    
    // Step 3: trigger an exit of the composite state [ErrorMode]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_ACT_Parent)&(mod_sys_ctrl_ref0_stm_ref0_exit_9'=Exit_Sub_ACT_Waiting);
    [OUT_BUF__int_ActualHV_27] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=Action_61))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=int_ActualHV_62)&(EVT__OUT_BUF__int_ActualHV_27'=mod_sys_ctrl_ref0_stm_ref0_ActualHV);
    
    // The transition [t5] from a junction [j0] to [s0].
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_j0))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=Action_61);
    
    // Step 4/2: exit command of the state [si0]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_si0)))&((mod_sys_ctrl_ref0_stm_ref0_Init_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t2] from [i0] to [s0].
    [] (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_s0_entering);
    
    // Step 2: the state [ErrorMode] is going to exit by setting its substates to exit firstly.
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&(((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_ACT_Parent))|((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_ACT_Trans))) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_Sub_ACT);
    
    // Call operation [disableHV]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_41))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=true);
    [] ((FIN__OUT_BUF__int_overLimit_22=true))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=int_overLimit_47))) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_si0)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // Call operation [disableHV]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_37))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_arg'=false)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_i0);
    
    // Step 7: make sure the composite source state [ErrorMode] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_t6)))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9'=Exit_NONE)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_entering);
    
    // The transition [t1] from a state [s0] to [f0].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0))&(((mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7=mod_sys_ctrl_ref0_stm_ref0_checkLimits_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_ActualHV<=mod_sys_ctrl_ref0_stm_ref0_overLimit)&(mod_sys_ctrl_ref0_stm_ref0_ActualHV>=mod_sys_ctrl_ref0_stm_ref0_underLimit))) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=Action_33)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_t1);
    
    // Step 4/2: exit command of the state [s1]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_Sub_EXITED);
    
    // Step 4/2: exit command of the state [loop_9]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9)))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9'=Exit_Sub_EXITED);
    
    // Wait for operation [disableHV] to exit.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))&((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1_entering)))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_disableHV_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // The transition [t5] from [i0] to [Ramping].
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_i0) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Ramping_entering);
    [RP__ext_setPoint_23] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1_entering))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((FIN__RP__ext_setPoint_23=true)))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=ext_setPoint_60)&(FIN__RP__ext_setPoint_23'=false);
    [RP__ext_setPoint_23] ((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=Action_51))&(((FIN__RP__ext_setPoint_23=true))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=ext_setPoint_52)&(FIN__RP__ext_setPoint_23'=false);
    [RP__currentState_26] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_entering) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_39)&(EVT__RP__currentState_26'=State_Wait24Vpower);
    
    // Step 4/2: exit command of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s0))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9'=Exit_Sub_EXITED);
    
    // The transition [t5] from a state [s3] to [s1].Step 1: trigger an exit of the state [s3]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lim=true))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t5)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1_entering);
    
    // Wait for operation [supplyVoltCheck] to exit.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4_entering)))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_INACTIVE_10)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4);
    
    // Step 5: check if all substates of the composite state [ClosedLoop] are exited
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9=Exit_Sub_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_EXITED)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_INACTIVE_10);
    
    // Call operation [checkLimits]
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3_entering)))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10)) -> (mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_i0);
    
    // From the probabilistic junction [sp_pj_3]
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_sp_pj_3) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_45);
    
    // The transition [sp1_22] from a state [s0] to [sp_pj_21].Step 1: trigger an exit of the state [s0]
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=ext_pow24VStatus_31) -> (FIN__IN_BUF__ext_pow24_1_17'=true)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_power'=EVT__IN_BUF__ext_pow24_1_17)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_21);
    
    // Wait for operation [checkLimits] to exit.
    [] ((((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop)))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0_entering)))&((mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8=mod_sys_ctrl_ref0_stm_ref0_checkLimits_TERMINATED_11)) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0)&(mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_checkLimits_INACTIVE_10);
    
    // From the probabilistic junction [sp_pj_21]
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_21) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=Action_29);
    [] ((FIN__OUT_BUF__int_ActualHV_27=true))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=int_ActualHV_54))) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_j0);
    
    // Step 4/2: exit command of the state [f0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9=Exit_Sub_ACT)) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9'=Exit_Sub_EXITED);
    [] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=Action_40) -> (mod_sys_ctrl_ref0_stm_ref0_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_entering)&(mod_sys_ctrl_ref0_stm_ref0_res'=false);
    
    // The transition [t0] from a state [s1] to [f0].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8=mod_sys_ctrl_ref0_stm_ref0_disableHV_s1))&(((mod_sys_ctrl_ref0_stm_ref0_disableHV_arg=false))&((mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7=mod_sys_ctrl_ref0_stm_ref0_disableHV_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8'=Action_35)&(mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7'=mod_sys_ctrl_ref0_stm_ref0_disableHV_t0);
    
    // The transition [sp1_19] from a state [s0] to [sp_pj_18].Step 1: trigger an exit of the state [s0]
    [IN_BUF__ext_pow24_1_17] ((FIN__IN_BUF__ext_pow24_1_17=true))&(((EVT__IN_BUF__ext_pow24_1_17=Power_On))&(((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0)))) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=ext_pow24VStatus_32)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_19)&(FIN__IN_BUF__ext_pow24_1_17'=false);
    [RP__ext_setPoint_24] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ErrorMode))&((mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8=Action_56)) -> (mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2_entering)&(EVT__RP__ext_setPoint_24'=mod_sys_ctrl_ref0_stm_ref0_setPoint);
    [] (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8=Action_30) -> (mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0)&(mod_sys_ctrl_ref0_stm_ref0_lim'=false)&(mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7'=mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_LOCK_FREE);
    [RP__currentState_26] (mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_entering) -> (EVT__RP__currentState_26'=State_Init)&(mod_sys_ctrl_ref0_stm_ref0_scpc_8'=Action_37);
    
    // The transition [t4] from a state [ClosedLoop] to [ErrorMode].Step 1: trigger an exit of the state [ClosedLoop]
    [] ((mod_sys_ctrl_ref0_stm_ref0_res=true))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t4);
    
    // The transition [t1] from a state [s0] to [s1].Step 1: trigger an exit of the state [s0]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0)))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t1)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1_entering);
    
    // Step 7: make sure the composite source state [s2] has been exited.
    [] (((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t2))))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9=Exit_EXITED)) -> (mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1_entering)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9'=Exit_NONE);
    
    // The transition [t1] from a state [Init] to [Wait24Vpower].Step 1: trigger an exit of the state [Init]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init)) -> (mod_sys_ctrl_ref0_stm_ref0_Init_exit_9'=Exit_ACT_Trans)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_t1);
    
    // The transition [loop_self_16] from a state [loop_15] to [loop_15].Step 1: trigger an exit of the state [loop_15]
    [] ((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init))&(((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&((mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Init_loop_15))) -> (mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Init_loop_15)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    
    // The transition [to_loop_14] from a state [s1] to [loop_12].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))&((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1))) -> (mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12);
    
    // The transition [to_loop_11] from a state [s1] to [loop_9].Step 1: trigger an exit of the state [s1]
    [] ((mod_sys_ctrl_ref0_stm_ref0_lock_7=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE))&(((mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower))) -> (mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE);
    [] ((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8=ext_setPoint_60))&(((mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2))&((mod_sys_ctrl_ref0_stm_ref0_scpc_8=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop))) -> (mod_sys_ctrl_ref0_stm_ref0_setPoint'=EVT__RP__ext_setPoint_23)&(mod_sys_ctrl_ref0_stm_ref0_lock_7'=mod_sys_ctrl_ref0_stm_ref0_LOCK_FREE)&(mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8'=mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1)&(FIN__RP__ext_setPoint_23'=true);
endmodule

// For the robotic platform [rp_ref0]
module mod_sys_rp_ref0
    EVT__RP__ext_setPoint_23 : [0..2];
    EVT__RP__ext_pow24VStatus_28 : [0..1];

    [RP__ext_setPoint_23] true -> (EVT__RP__ext_setPoint_23'=(1));
    [RP__currentState_26] true -> true;
    [RP__ext_setPoint_23] true -> (EVT__RP__ext_setPoint_23'=(2));
    [RP__ext_setPoint_24] true -> true;
    [RP__int_pwmSignal_18] true -> true;
    [RP__ext_pow24VStatus_28] true -> (EVT__RP__ext_pow24VStatus_28'=(1));
    [RP__ext_setPoint_23] true -> (EVT__RP__ext_setPoint_23'=(0));
    [RP__ext_pow24VStatus_28] true -> (EVT__RP__ext_pow24VStatus_28'=(0));
    [RP__ActualHV_2_21] true -> true;
endmodule

// Renames of RoboChart elements (automatically generated and used by the assertion language and don't delete)
// renames of the STM [mod_sys::ctrl_ref0::stm_ref0] to [mod_sys_ctrl_ref0_stm_ref0]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::int_underLimit::OUT] to [OUT_BUF__int_underLimit_20]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::currentState::OUT] to [RP__currentState_26]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::ext_setPoint::OUT] to [RP__ext_setPoint_24]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::ext_setPoint::IN] to [RP__ext_setPoint_23]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::ext_pow24VStatus::IN] to [IN_BUF__ext_pow24_1_17]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::int_pwmSignal::OUT] to [RP__int_pwmSignal_18]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::int_DisableHV::IN] to [IN_BUF__int_DisableHV_16]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::int_ActualHV::OUT] to [OUT_BUF__int_ActualHV_27]
// renames of the event [mod_sys::ctrl_ref0::stm_ref0::int_overLimit::OUT] to [OUT_BUF__int_overLimit_22]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::errorFlag] to [mod_sys_ctrl_ref0_stm_ref0_errorFlag]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::setPoint] to [mod_sys_ctrl_ref0_stm_ref0_setPoint]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::res] to [mod_sys_ctrl_ref0_stm_ref0_res]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::lim] to [mod_sys_ctrl_ref0_stm_ref0_lim]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::supplyLim] to [mod_sys_ctrl_ref0_stm_ref0_supplyLim]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::errorAck] to [mod_sys_ctrl_ref0_stm_ref0_errorAck]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ActualHV] to [mod_sys_ctrl_ref0_stm_ref0_ActualHV]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::overLimit] to [mod_sys_ctrl_ref0_stm_ref0_overLimit]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::underLimit] to [mod_sys_ctrl_ref0_stm_ref0_underLimit]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::power] to [mod_sys_ctrl_ref0_stm_ref0_power]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::int_underLimit] to [EVT__OUT_BUF__int_underLimit_20]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::currentState] to [EVT__RP__currentState_26]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::ext_setPoint] to [EVT__RP__ext_setPoint_24]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::int_pwmSignal] to [EVT__RP__int_pwmSignal_18]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::int_ActualHV] to [EVT__OUT_BUF__int_ActualHV_27]
// renames of the channel variable [mod_sys::ctrl_ref0::stm_ref0::int_overLimit] to [EVT__OUT_BUF__int_overLimit_22]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::power] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_power]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::s0] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_s0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp_pj_21] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_21]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::f0] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_f0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp_pj_18] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp_pj_18]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::i0] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_i0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::t0] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp1_19] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_19]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp2_20] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp2_20]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp2_23] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp2_23]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::sp1_22] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_sp1_22]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck::lock_7] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck_lock_7]
// renames of the operation [mod_sys::ctrl_ref0::stm_ref0::supplyVoltCheck] to [mod_sys_ctrl_ref0_stm_ref0_supplyVoltCheck]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::checkLimits::i0] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::checkLimits::s0] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_s0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::checkLimits::f0] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_f0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::checkLimits::t0] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::checkLimits::t1] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::checkLimits::t2] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_t2]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::checkLimits::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::checkLimits::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::checkLimits::lock_7] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits_lock_7]
// renames of the operation [mod_sys::ctrl_ref0::stm_ref0::checkLimits] to [mod_sys_ctrl_ref0_stm_ref0_checkLimits]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::disableHV::s1] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_s1]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::disableHV::f0] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_f0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::disableHV::i0] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::disableHV::s0] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_s0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::disableHV::t0] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::disableHV::t3] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_t3]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::disableHV::t1] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::disableHV::t2] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_t2]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::disableHV::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::disableHV::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::disableHV::lock_7] to [mod_sys_ctrl_ref0_stm_ref0_disableHV_lock_7]
// renames of the operation [mod_sys::ctrl_ref0::stm_ref0::disableHV] to [mod_sys_ctrl_ref0_stm_ref0_disableHV]
// renames of the operation parameter [mod_sys::ctrl_ref0::stm_ref0::disableHV::arg] to a varaible [mod_sys_ctrl_ref0_stm_ref0_disableHV_arg]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Init] to [mod_sys_ctrl_ref0_stm_ref0_Init]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::sp_pj_3] to [mod_sys_ctrl_ref0_stm_ref0_sp_pj_3]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::sp_pj_6] to [mod_sys_ctrl_ref0_stm_ref0_sp_pj_6]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Ramping] to [mod_sys_ctrl_ref0_stm_ref0_Ramping]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::i0] to [mod_sys_ctrl_ref0_stm_ref0_i0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t0] to [mod_sys_ctrl_ref0_stm_ref0_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t5] to [mod_sys_ctrl_ref0_stm_ref0_t5]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::sp1_4] to [mod_sys_ctrl_ref0_stm_ref0_sp1_4]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t2] to [mod_sys_ctrl_ref0_stm_ref0_t2]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::sp2_8] to [mod_sys_ctrl_ref0_stm_ref0_sp2_8]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t3] to [mod_sys_ctrl_ref0_stm_ref0_t3]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t6] to [mod_sys_ctrl_ref0_stm_ref0_t6]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t1] to [mod_sys_ctrl_ref0_stm_ref0_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::t4] to [mod_sys_ctrl_ref0_stm_ref0_t4]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::sp2_5] to [mod_sys_ctrl_ref0_stm_ref0_sp2_5]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::sp1_7] to [mod_sys_ctrl_ref0_stm_ref0_sp1_7]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_exit_9]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Init::i0] to [mod_sys_ctrl_ref0_stm_ref0_Init_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Init::loop_15] to [mod_sys_ctrl_ref0_stm_ref0_Init_loop_15]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Init::si0] to [mod_sys_ctrl_ref0_stm_ref0_Init_si0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Init::to_loop_17] to [mod_sys_ctrl_ref0_stm_ref0_Init_to_loop_17]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Init::loop_self_16] to [mod_sys_ctrl_ref0_stm_ref0_Init_loop_self_16]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Init::t0] to [mod_sys_ctrl_ref0_stm_ref0_Init_t0]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::Init::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_Init_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::Init::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_Init_scpc_8]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::f0] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_f0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::i0] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::s2] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s2]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::s1] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_s1]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::j0] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_j0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::t1] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::t4] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t4]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::t2] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t2]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::t0] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::t5] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_t5]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ErrorMode::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_ErrorMode_scpc_8]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::loop_12] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_12]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::i0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s4] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s4]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s3] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s3]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s1] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t2] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t2]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::to_loop_14] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_to_loop_14]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t5] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t5]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t4] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t4]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::loop_self_13] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_loop_self_13]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t1] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::t3] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_t3]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_exit_9]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::i0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::j0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_j0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::s1] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s1]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::s0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_s0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::t0] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::t2] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t2]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::t5] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t5]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::t1] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_t1]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::s2::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_s2_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::ClosedLoop::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_ClosedLoop_scpc_8]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::s1] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_s1]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::loop_9] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_9]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::i0] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_i0]
// renames of the node [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::si0] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_si0]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::to_loop_11] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_to_loop_11]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::loop_self_10] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_loop_self_10]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::t1] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t1]
// renames of the transition [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::t0] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_t0]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::exit_9] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_exit_9]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::Wait24Vpower::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_Wait24Vpower_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::scpc_8] to [mod_sys_ctrl_ref0_stm_ref0_scpc_8]
// renames of the variable [mod_sys::ctrl_ref0::stm_ref0::lock_7] to [mod_sys_ctrl_ref0_stm_ref0_lock_7]
// renames of the STM [mod_sys::ctrl_ref3::stm0] to [mod_sys_ctrl_ref3_stm0]
// renames of the event [mod_sys::ctrl_ref3::stm0::int_ActualHV::IN] to [IN_BUF__int_ActualHV_27]
// renames of the event [mod_sys::ctrl_ref3::stm0::ActualHV_1::OUT] to [OUT_BUF__ActualHV_1_19]
// renames of the event [mod_sys::ctrl_ref3::stm0::ActualHV_2::OUT] to [RP__ActualHV_2_21]
// renames of the variable [mod_sys::ctrl_ref3::stm0::ActualHV] to [mod_sys_ctrl_ref3_stm0_ActualHV]
// renames of the channel variable [mod_sys::ctrl_ref3::stm0::ActualHV_1] to [EVT__OUT_BUF__ActualHV_1_19]
// renames of the channel variable [mod_sys::ctrl_ref3::stm0::ActualHV_2] to [EVT__RP__ActualHV_2_21]
// renames of the node [mod_sys::ctrl_ref3::stm0::s0] to [mod_sys_ctrl_ref3_stm0_s0]
// renames of the node [mod_sys::ctrl_ref3::stm0::i0] to [mod_sys_ctrl_ref3_stm0_i0]
// renames of the transition [mod_sys::ctrl_ref3::stm0::t1] to [mod_sys_ctrl_ref3_stm0_t1]
// renames of the transition [mod_sys::ctrl_ref3::stm0::t0] to [mod_sys_ctrl_ref3_stm0_t0]
// renames of the variable [mod_sys::ctrl_ref3::stm0::exit_9] to [mod_sys_ctrl_ref3_stm0_exit_9]
// renames of the variable [mod_sys::ctrl_ref3::stm0::scpc_8] to [mod_sys_ctrl_ref3_stm0_scpc_8]
// renames of the variable [mod_sys::ctrl_ref3::stm0::lock_7] to [mod_sys_ctrl_ref3_stm0_lock_7]
// renames of the STM [mod_sys::ctrl_ref1::stm_ref0] to [mod_sys_ctrl_ref1_stm_ref0]
// renames of the event [mod_sys::ctrl_ref1::stm_ref0::int_DisableHV::OUT] to [OUT_BUF__int_DisableHV_16]
// renames of the event [mod_sys::ctrl_ref1::stm_ref0::ext_pow24VStatus::IN] to [IN_BUF__ext_pow24_2_25]
// renames of the event [mod_sys::ctrl_ref1::stm_ref0::int_underLimit::IN] to [IN_BUF__int_underLimit_20]
// renames of the event [mod_sys::ctrl_ref1::stm_ref0::int_overLimit::IN] to [IN_BUF__int_overLimit_22]
// renames of the event [mod_sys::ctrl_ref1::stm_ref0::int_ActualHV::IN] to [IN_BUF__ActualHV_1_19]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::power] to [mod_sys_ctrl_ref1_stm_ref0_power]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::ActualHV] to [mod_sys_ctrl_ref1_stm_ref0_ActualHV]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::overLimit] to [mod_sys_ctrl_ref1_stm_ref0_overLimit]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::underLimit] to [mod_sys_ctrl_ref1_stm_ref0_underLimit]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0] to [mod_sys_ctrl_ref1_stm_ref0_s0]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::i0] to [mod_sys_ctrl_ref1_stm_ref0_i0]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::t1] to [mod_sys_ctrl_ref1_stm_ref0_t1]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::t0] to [mod_sys_ctrl_ref1_stm_ref0_t0]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::exit_9] to [mod_sys_ctrl_ref1_stm_ref0_exit_9]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::PowerStatusRead] to [mod_sys_ctrl_ref1_stm_ref0_s0_PowerStatusRead]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::ActualHVRead] to [mod_sys_ctrl_ref1_stm_ref0_s0_ActualHVRead]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::f0] to [mod_sys_ctrl_ref1_stm_ref0_s0_f0]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::Waiting] to [mod_sys_ctrl_ref1_stm_ref0_s0_Waiting]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::i0] to [mod_sys_ctrl_ref1_stm_ref0_s0_i0]
// renames of the node [mod_sys::ctrl_ref1::stm_ref0::s0::PowerAndActualHVRead] to [mod_sys_ctrl_ref1_stm_ref0_s0_PowerAndActualHVRead]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t0] to [mod_sys_ctrl_ref1_stm_ref0_s0_t0]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t6] to [mod_sys_ctrl_ref1_stm_ref0_s0_t6]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t2] to [mod_sys_ctrl_ref1_stm_ref0_s0_t2]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t5] to [mod_sys_ctrl_ref1_stm_ref0_s0_t5]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t3] to [mod_sys_ctrl_ref1_stm_ref0_s0_t3]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t7] to [mod_sys_ctrl_ref1_stm_ref0_s0_t7]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t1] to [mod_sys_ctrl_ref1_stm_ref0_s0_t1]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t8] to [mod_sys_ctrl_ref1_stm_ref0_s0_t8]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t9] to [mod_sys_ctrl_ref1_stm_ref0_s0_t9]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t4] to [mod_sys_ctrl_ref1_stm_ref0_s0_t4]
// renames of the transition [mod_sys::ctrl_ref1::stm_ref0::s0::t10] to [mod_sys_ctrl_ref1_stm_ref0_s0_t10]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::s0::exit_9] to [mod_sys_ctrl_ref1_stm_ref0_s0_exit_9]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::s0::scpc_8] to [mod_sys_ctrl_ref1_stm_ref0_s0_scpc_8]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::scpc_8] to [mod_sys_ctrl_ref1_stm_ref0_scpc_8]
// renames of the variable [mod_sys::ctrl_ref1::stm_ref0::lock_7] to [mod_sys_ctrl_ref1_stm_ref0_lock_7]
// renames of the STM [mod_sys::ctrl_ref2::stm0] to [mod_sys_ctrl_ref2_stm0]
// renames of the event [mod_sys::ctrl_ref2::stm0::ext_pow24VStatus::IN] to [RP__ext_pow24VStatus_28]
// renames of the event [mod_sys::ctrl_ref2::stm0::ext_pow24_2::OUT] to [OUT_BUF__ext_pow24_2_25]
// renames of the event [mod_sys::ctrl_ref2::stm0::ext_pow24_1::OUT] to [OUT_BUF__ext_pow24_1_17]
// renames of the variable [mod_sys::ctrl_ref2::stm0::power] to [mod_sys_ctrl_ref2_stm0_power]
// renames of the channel variable [mod_sys::ctrl_ref2::stm0::ext_pow24_2] to [EVT__OUT_BUF__ext_pow24_2_25]
// renames of the channel variable [mod_sys::ctrl_ref2::stm0::ext_pow24_1] to [EVT__OUT_BUF__ext_pow24_1_17]
// renames of the node [mod_sys::ctrl_ref2::stm0::i0] to [mod_sys_ctrl_ref2_stm0_i0]
// renames of the node [mod_sys::ctrl_ref2::stm0::s0] to [mod_sys_ctrl_ref2_stm0_s0]
// renames of the transition [mod_sys::ctrl_ref2::stm0::t0] to [mod_sys_ctrl_ref2_stm0_t0]
// renames of the transition [mod_sys::ctrl_ref2::stm0::t1] to [mod_sys_ctrl_ref2_stm0_t1]
// renames of the variable [mod_sys::ctrl_ref2::stm0::exit_9] to [mod_sys_ctrl_ref2_stm0_exit_9]
// renames of the variable [mod_sys::ctrl_ref2::stm0::scpc_8] to [mod_sys_ctrl_ref2_stm0_scpc_8]
// renames of the variable [mod_sys::ctrl_ref2::stm0::lock_7] to [mod_sys_ctrl_ref2_stm0_lock_7]
// renames of the event [mod_sys::rp_ref0::ext_setPoint::OUT] to [RP__ext_setPoint_23]
// renames of the event [mod_sys::rp_ref0::ext_setPoint::IN] to [RP__ext_setPoint_24]
// renames of the event [mod_sys::rp_ref0::ext_pow24VStatus::OUT] to [RP__ext_pow24VStatus_28]
// renames of the event [mod_sys::rp_ref0::int_pwmSignal::IN] to [RP__int_pwmSignal_18]
// renames of the event [mod_sys::rp_ref0::int_ActualHV::IN] to [RP__ActualHV_2_21]
// renames of the event [mod_sys::rp_ref0::currentState::IN] to [RP__currentState_26]
// renames of the channel variable [mod_sys::rp_ref0::ext_setPoint] to [EVT__RP__ext_setPoint_23]
// renames of the channel variable [mod_sys::rp_ref0::ext_pow24VStatus] to [EVT__RP__ext_pow24VStatus_28]

