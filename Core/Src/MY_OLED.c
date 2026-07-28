//
// Created by wnywl on 2026/5/18.
//

#include "MY_OLED.h"

#include "main.h"
#include "OLED.h"
#include "OLED_Data.h"
#include "stm32g4xx_hal.h"

void three_phase_inverter_interface_head(void) {
    OLED_Clear();

    OLED_ShowString(0,0,"Va:",OLED_6X8);
    OLED_ShowString(64,0,"Ia:",OLED_6X8);
    OLED_ShowString(0,8,"Vb:",OLED_6X8);
    OLED_ShowString(64,8,"Ib:",OLED_6X8);
    OLED_ShowString(0,16,"Vc:",OLED_6X8);
    OLED_ShowString(64,16,"Ic:",OLED_6X8);

    OLED_Update();
}
void three_phase_inverter_interface_main(void) {
    OLED_ShowFloatNum(18,0,va_measurement,2,3,OLED_6X8);
    OLED_ShowFloatNum(82,0,ia_measurement,2,3,OLED_6X8);
    OLED_ShowFloatNum(18,8,vb_measurement,2,3,OLED_6X8);
    OLED_ShowFloatNum(82,8,ib_measurement,2,3,OLED_6X8);
    OLED_ShowFloatNum(18,16,vc_measurement,2,3,OLED_6X8);
    OLED_ShowFloatNum(82,16,ic_measurement,2,3,OLED_6X8);

    OLED_Update();
}
void DeBug_interface_head(void) {
    OLED_ShowString(0,24,"Vm:",OLED_6X8);
    OLED_ShowString(64,24,"hzc:",OLED_6X8);
    OLED_ShowString(0,32,"kp1:",OLED_6X8);
    OLED_ShowString(64,32,"kp2:",OLED_6X8);
    OLED_ShowString(0,40,"kr1:",OLED_6X8);
    OLED_ShowString(64,40,"kr2:",OLED_6X8);
    OLED_ShowString(0,48,"p1:",OLED_6X8);
    OLED_ShowString(64,48,"p2:",OLED_6X8);
    OLED_ShowString(0,56,"EC_DeBug:",OLED_6X8);

    OLED_Update();
}
void DeBug_interface_main(void) {
    OLED_ShowFloatNum(18,24,S.vm,3,3,OLED_6X8);
    OLED_ShowFloatNum(88,24,hzc,2,2,OLED_6X8);
    OLED_ShowFloatNum(24,32,QPR1.kp,2,2,OLED_6X8);
    OLED_ShowFloatNum(88,32,QPR2.kp,2,2,OLED_6X8);
    OLED_ShowFloatNum(24,40,QPR1.kr,2,2,OLED_6X8);
    OLED_ShowFloatNum(88,40,QPR2.kr,2,2,OLED_6X8);
    OLED_ShowFloatNum(18,48,p1,2,2,OLED_6X8);
    OLED_ShowFloatNum(82,48,p2,2,2,OLED_6X8);

    switch (now_EC_DeBug) {
        case vm_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"vm_",OLED_6X8);
            break;
        case kp1_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"kp1_",OLED_6X8);
            break;
        case kr1_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"kr1_",OLED_6X8);
            break;
        case kp2_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"kp2_",OLED_6X8);
            break;
        case kr2_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"kr2_",OLED_6X8);
            break;
        case p1_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"p1_",OLED_6X8);
            break;
        case p2_:
            OLED_ClearArea(54,56,72,8);
            OLED_ShowString(54,56,"p2_",OLED_6X8);
            break;
        default:
            break;
    }

    OLED_Update();
}
