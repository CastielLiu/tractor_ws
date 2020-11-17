
# 智能农机底层制动系统控制程序(arduino)
- 利用步进电机控制左侧离合踏板和右侧制动踏板，实现制动执行器控制;
- 利用线性位移传感器检测离合踏板和制动踏板的位置，对控制状态进行反馈.
- 订阅/brake_cmd并按指令进行控制
- 发布/brake_state反馈制动系统状态，反馈频率与控制指令频率一致

## 传感器位置标定
1. 向话题/request_sensor_val定时发布请求，系统将反馈出左右踏板传感器AD值
2. 订阅/brake_sensor_adc_left和/brake_sensor_adc_right查看传感器AD值
3. 读取左踏板释放极限位置和踩踏极限位置处传感器的AD值，分别设置到程序中g_adc_l_max，g_adc_l_min
4. 同上获取g_adc_r_min, g_adc_r_max.

## 驱动电机转动方向标定
修改下述参数值修改电机转动方向
int BRAKE_DIR_PRESS   = LOW;
int BRAKE_DIR_RELEASE = HIGH;

int CLUTCH_DIR_PRESS  = LOW;
int CLUTCH_DIR_RELEASE  = HIGH;

## 重要参数
- g_brake_tatol_grade 制动行程等级，应与上层控制器保持一致
- g_adc_l_min, g_adc_l_max 左侧踏板两极限位置时传感器AD值，标定
- g_adc_r_min, g_adc_r_max 右侧踏板两极限位置时传感器AD值，标定
- rotate_duration_us       每个控制指令电机转动的时长，根据上层指令控制周期调整


## 相关话题名称
/brake_cmd
/brake_state

/request_sensor_val
/brake_sensor_adc_left
/brake_sensor_adc_right
