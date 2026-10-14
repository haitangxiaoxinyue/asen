#include "ControlTask.h"
#include "Gm6020.h"
#include "Pid.h"
#include "main.h"
#include "can.h"
#include "tim.h"
#include "iwdg.h"
#include <cmath>


static Gm6020 motor(1);
//电机id 

static Pid velPid(305.0f,0.02f,0.4f,16384.0f,-16384.0f);//目标速给电机的电流（kp 先 500 起步，翻倍调）
static Pid posPid(20.0f,0.0f,0.0f,30.0f,-30.0f);//目标角度给期望速度（kp=5起步，输出限±30rad/s=电机极限内）
//速度位置pid
/*class Pid {
public:
    Pid(float kp, float ki, float kd, float out_max, float out_min);  // 构造函数（造对象时调用）
    float calc(float ref, float fdb);                                 // 计算函数（按按钮）
private:
    float kp_, ki_, kd_;       // 内部存三个参数
    float out_max_, out_min_;  // 内部存上下限
    float integ_, last_err_;   // 内部存运行状态
};*/


static const float kVelAmp = 20.0f;//速度环幅（20rad/s≈191rpm，低于电机最高320rpm，安全）
static const float kPosAmp = 3.14f;//位置环幅  波高那个系数
static const float kSinFreq = 0.5f;//频率 
static const float k2Pi     = 6.2831853f;


//运行累计时间

float t = 0.0f;
//flase 速度 ture 位置
static bool positionMode = true;

// 给 Ozone 调试看的目标值和实际值
// 注意：不加 static，做成真正的全局变量，Ozone 的 Data Sampling 才能按名字找到
float g_ref = 0.0f;   // 目标值（想要的速度/角度）
float g_fdb = 0.0f;   // 实际值（电机真实的速度/角度）

// 速度反馈一阶低通滤波（组长建议：电机反馈转速是整数 rpm，步进≈0.1047rad/s，有量化抖动）
static float velFdbFilt = 0.0f;          // 滤波后的实际速度 (rad/s)
static const float kVelFiltAlpha = 0.1f; // 滤波系数(0~1)：越小越平滑但越滞后




void ControlTaskInit(void)
{
    // 配 CAN 过滤器 
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0;
    filter.FilterIdLow = 0;
    filter.FilterMaskIdHigh = 0;
    filter.FilterMaskIdLow = 0;
    filter.FilterFIFOAssignment = CAN_RX_FIFO0;
    filter.FilterActivation = ENABLE;
    filter.SlaveStartFilterBank = 14;
    HAL_CAN_ConfigFilter(&hcan1, &filter);

    // 启动 CAN1
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);

    // 启动 TIM6 定时器（1kHz 中断）
    HAL_TIM_Base_Start_IT(&htim6);
}




/*float Pid::calc(float ref, float fdb) {
    float err   = ref - fdb;        // ① 误差 = 目标 - 实际
    float deriv = err - last_err_;  // ② 误差的变化量（这拍比上拍差得更多还是更少）

    integ_ += err;                  // ③ 把这次的误差"记账"累加起来

    float out = kp_ * err                          // ④ P 项
              + ki_ * integ_ * kSampleTime         // ⑤ I 项
              + kd_ * deriv / kSampleTime;         // ⑥ D 项

    // 限幅 + 抗积分饱和
    if (out > out_max_) { out = out_max_; integ_ -= err; }
    else if (out < out_min_) { out = out_min_; integ_ -= err; }

    last_err_ = err;                // ⑦ 记下这拍误差，给下一拍用
    return out;                     // ⑧ 返回控制量
}*/









void MainTask(void)
{
    // 1. 时间加tim6对应的一毫秒
    t += 0.001f;

    // sine表示当前的正弦值
    float sine = sinf(k2Pi * kSinFreq * t);

    // 速度反馈滤波：把 motor.vel() 的量化抖动平滑掉，再给速度环用
    float velRaw = motor.vel();
    velFdbFilt += kVelFiltAlpha * (velRaw - velFdbFilt);

    // 3. 根据模式算出该给的电压
    float voltage = 0.0f;
    if (positionMode)
    {
        // 位置模式位置环(外) → 速度环(内)，串级
        float posRef = kPosAmp * sine;                      // 目标角度
        float posFdb = motor.angle();                       // 实际角度
        // 速度前馈：目标角度 posRef = A*sin(ωt) 的导数 = 目标角速度 A*ω*cos(ωt)
        // 把"理论上该跑多快"直接给速度环，位置环 PID 只需补小误差，滞后大幅减小
        float cosine = cosf(k2Pi * kSinFreq * t);
        float velFF  = kPosAmp * k2Pi * kSinFreq * cosine;  // 前馈速度（最大 3.14*π≈9.87 rad/s）
        float velRef = posPid.calc(posRef, posFdb) + velFF; // 位置环输出 + 前馈
        g_ref = posRef;                                     // 抄给 Ozone 看
        g_fdb = posFdb;                                     // 抄给 Ozone 看
        voltage = velPid.calc(velRef, velFdbFilt);          // 速度环输出 = 电流（用滤波后速度）
        
    }
    else
    {
        // 速度模式：速度环闭环（先调这个环，再调位置环）
        float velRef = kVelAmp * sine;                      // 目标速度（正弦波）
        float velFdb = velFdbFilt;                          // 实际速度（滤波后）
        g_ref = velRef;                                     // 抄给 Ozone 看
        g_fdb = velFdb;                                     // 抄给 Ozone 看
        voltage = velPid.calc(velRef, velFdb);              // 速度环输出 = 电流
    }

    // 不要超出去（电流模式：±16384）
    if (voltage > 16384.0f)  voltage = 16384.0f;
    if (voltage < -16384.0f) voltage = -16384.0f;

    // 5. 把电压写给电机编码成 8 字节发出去
    motor.setVoltage((int16_t)voltage);
    uint8_t txBuf[8] = {0};
    motor.encode(txBuf);

    CAN_TxHeaderTypeDef txHeader;
    txHeader.StdId = motor.txId();
    txHeader.ExtId = 0;
    txHeader.IDE = CAN_ID_STD;
    txHeader.RTR = CAN_RTR_DATA;
    txHeader.DLC = 8;
    txHeader.TransmitGlobalTime = DISABLE;

    uint32_t txMailbox = 0;
    HAL_CAN_AddTxMessage(&hcan1, &txHeader, txBuf, &txMailbox);

    // 喂狗
    HAL_IWDG_Refresh(&hiwdg);
}





// 收到帧
void CanFeedbackCallback(uint32_t std_id, const uint8_t *data)
{
    if (std_id != motor.rxId()) {   // 
        return;
    }
    motor.decode(data);             // 读出来
}

// 收到信息回溯
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rxHeader;
    uint8_t data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxHeader, data) != HAL_OK) {
        return;
    }
    CanFeedbackCallback(rxHeader.StdId, data);
}

// 贤弟机会到了我们就该出了
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance != TIM6) {   //TIM6
        return;
    }
    MainTask();                     // 每 1ms 跑一次控制循环
}