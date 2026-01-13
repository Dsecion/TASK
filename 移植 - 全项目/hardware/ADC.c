#include "ADC.h"

// 简单的低通滤波系数
#define VOLTAGE_FILTER_COEF 0.05f

static float voltage_filtered = VOLTAGE_NOMINAL;

/**
 * @brief  ADC初始化
 * @note   NUCLEO-F401RE 使用 A4 (PC1) 作为电池电压检测
 */
void ADC_User_Init(void) {
    GPIO_InitTypeDef GPIO_InitStructure;
    ADC_CommonInitTypeDef ADC_CommonInitStructure;
    ADC_InitTypeDef ADC_InitStructure;

    // 开启时钟
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);

    // 配置PC1为模拟输入 (NUCLEO A4 Pin)
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    // ADC通用配置
    ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4; // PCLK2=84MHz, ADC_CLK=21MHz
    ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled;
    ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_5Cycles;
    ADC_CommonInit(&ADC_CommonInitStructure);

    // ADC1配置
    ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE; // 关闭连续转换，防止OVR锁死数据
    ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfConversion = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    // 配置规则通道 PC1 (Channel 11)
    ADC_RegularChannelConfig(ADC1, ADC_Channel_11, 1, ADC_SampleTime_480Cycles);

    // 开启ADC
    ADC_Cmd(ADC1, ENABLE);
    
    // 启动首次转换
    ADC_SoftwareStartConv(ADC1);
}

/**
 * @brief  获取电池电压（带滤波）
 * @note   非阻塞读取：查询EOC标志，如果完成则更新并启动下一次
 * @retval 电压值(V)
 */
float Get_Battery_Voltage(void) {
    // 检查转换是否完成 (EOC=1)
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_EOC) != RESET) {
        // 读取数据（会自动清除EOC标志）
        uint16_t ad_value = ADC_GetConversionValue(ADC1);
        
        // 计算瞬时电压
        float voltage_instant = (float)ad_value * (ADC_REF_VOLTAGE / 4096.0f) * VOLTAGE_DIVIDER_RATIO;
        
        // 低通滤波
        if (voltage_filtered < 1.0f && voltage_instant > 1.0f) {
            voltage_filtered = voltage_instant;
        } else {
            voltage_filtered = voltage_filtered * (1.0f - VOLTAGE_FILTER_COEF) + voltage_instant * VOLTAGE_FILTER_COEF;
        }
        
        // 启动下一次转换，为下一次读取做准备
        ADC_SoftwareStartConv(ADC1);
    }
    // 注意：如果出现 Overrun (OVR)，EOC 可能不会置位或者数据被锁死
    // 检查并清除 OVR 标志以防万一
    if (ADC_GetFlagStatus(ADC1, ADC_FLAG_OVR) != RESET) {
        ADC_ClearFlag(ADC1, ADC_FLAG_OVR);
        ADC_SoftwareStartConv(ADC1); // 重启转换
    }
    
    return voltage_filtered;
}

/**
 * @brief  获取电压补偿比例
 * @retval 补偿系数 (V_nom / V_curr)
 */
float Get_Voltage_Scale(void) {
    float v_curr = Get_Battery_Voltage();
    
    // 防止除零和异常电压
    if (v_curr < 7.0f) { // 电池电压过低，可能是未接电池（USB供电），不进行补偿或设为1
        return 1.0f; 
    }
    
    // 如果当前电压高于标称电压（满电状态），不进行负补偿（不削弱动力）
    // 保持 Scale = 1.0，确保满电时拥有最强动力
    if (v_curr > VOLTAGE_NOMINAL) {
        return 1.0f;
    }

    float scale = VOLTAGE_NOMINAL / v_curr;
    
    // 限幅，防止补偿过大导致震荡
    if (scale > 1.4f) scale = 1.4f;
    
    return scale;
}
