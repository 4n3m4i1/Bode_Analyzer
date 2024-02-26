#ifndef BANDIT_PINS_h
#define BANDIT_PINS_h

/*(
    02-24-2024 Joseph A. De Vico

    Just pins!
    Maps directly to GPIO register bit offsets (and thus GPIO number)
)*/

#define PINSH(a)    (1u << a)

enum BODE_BANDIT_PINS {
    NC_0_PAD,
    ADC_CSN_PAD,
    ADC_SCK_PAD,
    ADC_SDI_PAD,
    ADC_SDO_A_PAD,
    ADC_SDO_B_PAD,
    NC_1_PAD,
    NC_2_PAD,
    NC_3_PAD,
    PGA_CSN_PAD,
    PGA_SCK_PAD,
    PGA_SI_PAD,
    RGB_R_PAD,
    RGB_G_PAD,
    RGB_B_PAD,
    USER_LED_PAD,
    NC_4_PAD,
    NC_5_PAD,
    PDM_WGN_PAD,
    NC_6_PAD,
    NC_7_PAD,
    VDDA_EN_PAD,
    NC_8_PAD,
    NC_9_PAD,
    NC_10_PAD,
    NC_11_PAD,
    NC_12_PAD,
    NC_13_PAD,
    NC_14_PAD,
    NC_15_PAD
};

#define PWMSLICE_RG 6
#define PWMCHAN_R   0
#define PWMCHAN_G   1

#define PWMSLICE_BU 7
#define PWMCHAN_B   0
#define PWMCHAN_U   1

#define PWMSLICE_ISR    1

#endif