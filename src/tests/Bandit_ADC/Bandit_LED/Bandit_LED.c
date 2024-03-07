#include "Bandit_LED.h"

void init_LED_pins(){
    // Cleanup for direct register access, minimal slice readdress
    //  since they are shared to a high degree
    gpio_init_mask(                         // Enable GPIO for LEDs
                PINSH(RGB_R_PAD) | 
                PINSH(RGB_G_PAD) | 
                PINSH(RGB_B_PAD) | 
                PINSH(USER_LED_PAD)
                );

    // Set GPIO Muxes to PWM sources
    gpio_set_function(RGB_R_PAD, GPIO_FUNC_PWM);
    gpio_set_function(RGB_G_PAD, GPIO_FUNC_PWM);
    gpio_set_function(RGB_B_PAD, GPIO_FUNC_PWM);
    gpio_set_function(USER_LED_PAD, GPIO_FUNC_PWM);

    // Zero out all PWM levels just in case
    set_RGB_levels(0,0,0);  
    set_ULED_level(0);  

    /*
        Set PWM top, set PWM slice clk div, 
        invert both channel outputs, enable pwm slice

        125MHz / 256 (divider) == 488281 Hz counter clock
            488281 / 256 max -> 1.907kHz output f, good enough!
    */

    // RED and GREEN RGB Channels
    pwm_hw->slice[PWMSLICE_RG].top =    0xFF - 1u;
    pwm_hw->slice[PWMSLICE_RG].div =    (0xFF << PWM_CH6_DIV_INT_LSB);
    pwm_hw->slice[PWMSLICE_RG].csr =    PWM_CH6_CSR_A_INV_BITS |
                                        PWM_CH6_CSR_B_INV_BITS |
                                        PWM_CH6_CSR_EN_BITS;

    // BLUE RGB Channel, USER LED
    pwm_hw->slice[PWMSLICE_BU].top =    0xFF;
    pwm_hw->slice[PWMSLICE_BU].div =    (0xFF << PWM_CH7_DIV_INT_LSB);
    pwm_hw->slice[PWMSLICE_BU].csr =    PWM_CH7_CSR_A_INV_BITS |
                                        PWM_CH7_CSR_B_INV_BITS |
                                        PWM_CH7_CSR_EN_BITS;

}
void set_RGB_levels(uint8_t R_, uint8_t G_, uint8_t B_){
    // Channel B is upper 16 bits, Channel A is lower 16 bits
    //  Red     -> Channel A, slice 6
    //  Green   -> Channel B, slice 6
    //  Blue    -> Channel A, slice 7
    pwm_hw->slice[PWMSLICE_RG].cc = (((uint16_t)G_ << 16) | (uint16_t)R_);
    pwm_hw->slice[PWMSLICE_BU].cc = (pwm_hw->slice[PWMSLICE_BU].cc & 0xFFFF0000 | (uint16_t)B_); 
}
void set_ULED_level(uint8_t _L){
    // Channel B is upper 16 bits, CHannel A is lower 16 bits
    //  UserLed -> Channel B, slice 7
    pwm_hw->slice[PWMSLICE_BU].cc = ((uint16_t)_L << 16) | (pwm_hw->slice[PWMSLICE_BU].cc & 0x0000FFFF); 
}