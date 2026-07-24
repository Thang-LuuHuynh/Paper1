# Tại sao cần sửa code STM32 để cải thiện quỹ đạo hình vuông

## 1. Mục tiêu

Robot cần bám quỹ đạo hình vuông cạnh 2 m theo cơ chế của `Paper1-cu`:

- chạy liên tục, không dừng tại đỉnh;
- giảm tốc trước góc;
- heading tham chiếu đổi 90° tại waypoint;
- robot quẹo bằng chuyển động của cả hai bánh;
- không được dừng bánh trong rồi dùng riêng bánh ngoài để xoay robot;
- sau góc, sai số ngang phải giảm nhanh và cạnh thẳng phải ít lượn.

Các thử nghiệm trên `Paper1-main` cho thấy chỉ thay đổi bộ điều khiển BSMC ở
ROS không thể đồng thời bảo đảm góc gọn, hai bánh luôn quay và cạnh thẳng có sai
số nhỏ. Nguyên nhân chính còn lại nằm trong vòng điều khiển tốc độ bánh xe trên
STM32.

## 2. Chuỗi điều khiển thực tế

Lệnh đi từ controller đến động cơ theo chuỗi:

```text
bsmc_square.py
    |
    |  /cmd_vel: v, w
    v
robot_serial_bridge.py
    |
    |  CMD,v,-w
    v
ESP32
    |
    |  UART
    v
STM32
    |
    |  đổi v,w thành setpoint RPM trái/phải
    |  PI tốc độ + bù deadzone
    v
Driver động cơ và hai bánh xe
```

BSMC chỉ phát được vận tốc thẳng `v` và vận tốc góc `w`. Việc một bánh có thật
sự quay đúng setpoint hay không do STM32, driver và động cơ quyết định.

## 3. Bằng chứng từ thử nghiệm phần cứng

### 3.1. Controller không ra lệnh dừng bánh trong

Trong các thử nghiệm R20–R23, controller đã nâng vận tốc tiến ở góc để vận tốc
bánh trong theo mô hình vi sai luôn dương:

```text
v_inner = v - |w| L / 2
v_outer = v + |w| L / 2
```

với khoảng cách hai bánh:

```text
L = 0.17 m
```

Ví dụ ở R22:

```text
v = 0.097 m/s
w = 0.550 rad/s

v_inner = 0.097 - 0.550*0.17/2
        = 0.05025 m/s
```

Với đường kính bánh `0.063 m`, setpoint lý thuyết của bánh trong xấp xỉ:

```text
RPM_inner = 0.05025 * 60 / (pi*0.063)
          ≈ 15.2 RPM
```

Như vậy, ở thời điểm bánh phải đo được `0 RPM`, controller vẫn đang yêu cầu nó
chạy khoảng `15 RPM`. Đây không phải lệnh pivot của BSMC.

Ở R23, ngay cả khi đã giảm tải quay:

```text
v = 0.097 m/s
w = 0.352 rad/s
```

thì setpoint bánh trong vẫn khoảng:

```text
v_inner ≈ 0.067 m/s
RPM_inner ≈ 20.3 RPM
```

Nhưng dữ liệu thực tế vẫn có thời điểm:

```text
RPM trái ≈ 39–41 RPM
RPM phải = 0 RPM
gyro_z   ≈ 0.77–0.80 rad/s
```

Gyro tăng mạnh khi bánh phải bằng 0 xác nhận robot thực sự đang xoay gần như
bằng một bánh. Đây không phải lỗi hiển thị RPM hoặc mất gói encoder.

### 3.2. Tăng `v` trong ROS không xử lý được nguyên nhân gốc

Các cấu hình đã thử:

| Run | Cơ chế góc | Waviness cạnh thẳng | Heading RMS | Mẫu bánh trong gần 0 |
|---|---|---:|---:|---:|
| R20 | tỷ lệ nền 0.15, recovery | 2.61 cm | 4.89° | 12.7% |
| R21 | nâng tỷ lệ nền lên 0.20 | 2.89 cm | 5.33° | 12.4% |
| R22 | preempt tỷ lệ bánh trước vùng tải | 2.76 cm | 5.34° | 9.3% |
| R23 | tăng `v` và giảm `w` khi anti-stall | 3.51 cm | 7.15° | 4.5% |

R23 giảm được số mẫu pivot nhưng không loại bỏ hoàn toàn, đồng thời góc rộng và
đoạn thẳng xấu rõ rệt. Nếu tiếp tục bù ở ROS bằng cách tăng `v` hoặc giảm `w`,
robot sẽ:

- quẹo ngày càng rộng;
- lệch khỏi waypoint nhiều hơn;
- mất phần lớn cạnh thẳng để sửa sai số sau góc;
- tăng lateral RMS và heading RMS;
- vẫn không bảo đảm bánh phải khởi động đúng lúc.

Do đó, bù bằng BSMC chỉ che lỗi của vòng điều khiển motor và làm xấu quỹ đạo.

## 4. Nguyên nhân trong code STM32

Firmware được tìm thấy tại:

```text
/home/thang/NCKH/BSMC--WMR-main/Core/Src/main.c
```

Firmware dùng:

```c
#define WHEEL_DIAMETER  0.063f
#define WHEEL_BASE      0.17f
#define MOTOR_PWM_MIN   120.0f

PID_TypeDef pidL = {2.48f, 24.10f, 0.00f, ...};
PID_TypeDef pidR = {2.44f, 24.10f, 0.00f, ...};
```

Công thức đổi `v,w` sang vận tốc hai bánh là đúng:

```c
float v_L = v - w * (WHEEL_BASE / 2.0f);
float v_R = v + w * (WHEEL_BASE / 2.0f);
```

Vấn đề nằm ở vòng PI:

```c
float err = pidR.setpoint - rpmR;
pidR.integral += err * TS;

pidR.output = pidR.Kp * err
            + pidR.Ki * pidR.integral
            + pidR.Kd * (err - pidR.last_error) / TS;

if (pidR.output > 999.0f) pidR.output = 999.0f;
if (pidR.output <   0.0f) pidR.output =   0.0f;

if (pidR.setpoint > 1.0f
    && pidR.output > 0.0f
    && pidR.output < MOTOR_PWM_MIN)
{
    pidR.output = MOTOR_PWM_MIN;
}
```

### 4.1. Integral wind-down khi vào góc

Trước góc, hai bánh đang chạy khoảng cùng tốc độ. Khi heading tham chiếu đổi
90°, setpoint bánh trong giảm nhanh.

Trong một khoảng ngắn:

```text
RPM thực bánh trong > setpoint mới
```

nên:

```text
err = setpoint - RPM thực < 0
```

Code vẫn cộng sai số âm vào integral dù output đã chạm giới hạn dưới. Integral
vì vậy bị kéo xuống, còn output bị clamp về 0.

### 4.2. Bù deadzone không hoạt động khi output bằng 0

Điều kiện deadzone có:

```c
pidR.output > 0.0f
```

Khi PI đã tạo output âm và bị clamp thành 0, nhánh bù `MOTOR_PWM_MIN` không
chạy. Bánh phải mất PWM, giảm tốc rồi đứng hẳn.

Sau khi RPM về dưới setpoint, sai số mới chuyển sang dương. Integral cần một
khoảng thời gian để hồi phục đủ lớn trước khi output dương trở lại. Khoảng trễ
này phù hợp với đoạn bánh phải đứng khoảng 0.4–0.5 s trong log.

### 4.3. Một `MOTOR_PWM_MIN` chung chưa chắc phù hợp hai bánh

Hai động cơ, hộp số và tải cơ khí không hoàn toàn giống nhau. Dữ liệu cho thấy
bánh phải dễ mất tốc hơn bánh trái. Dùng cùng `MOTOR_PWM_MIN = 120` có thể không
đủ thắng ma sát tĩnh của bánh phải khi robot đang chịu tải ngang ở góc.

## 5. Những thay đổi cần thực hiện trên STM32

Không nên thay đổi tất cả cùng lúc. Thứ tự đề xuất:

### 5.1. Thêm anti-windup đúng cho hai PI tốc độ

Không tích phân thêm theo chiều làm output bão hòa sâu hơn.

Ví dụ nguyên tắc:

```c
float integral_candidate = pidR.integral + err * TS;
float output_candidate =
      pidR.Kp * err
    + pidR.Ki * integral_candidate
    + pidR.Kd * derivative;

bool saturating_high = output_candidate >= PWM_MAX && err > 0.0f;
bool saturating_low  = output_candidate <= 0.0f   && err < 0.0f;

if (!saturating_high && !saturating_low) {
    pidR.integral = integral_candidate;
}
```

Có thể dùng conditional integration như trên hoặc back-calculation. Không nên
chỉ giới hạn integral ở biên rất lớn `±999/Ki`, vì giới hạn đó không ngăn
wind-down tại bão hòa 0 PWM.

### 5.2. Bổ sung feed-forward/deadzone hợp lý

PI hiện tại phải tự tạo toàn bộ PWM từ sai số. Nên thêm thành phần feed-forward
theo setpoint RPM hoặc PWM ma sát tĩnh:

```text
PWM = PWM_feedforward(setpoint_RPM) + PWM_PI
```

Tối thiểu cần có hai giá trị hiệu chuẩn riêng:

```c
MOTOR_PWM_MIN_L
MOTOR_PWM_MIN_R
```

Không nên cưỡng bức PWM tối thiểu khi bánh đang overspeed. Chỉ dùng start boost
khi:

- setpoint vẫn dương;
- RPM thấp hơn rõ rệt so với setpoint hoặc gần 0;
- boost có thời gian giới hạn;
- có hysteresis để tránh bật/tắt liên tục.

### 5.3. Xử lý bumpless transfer khi setpoint thay đổi nhanh

Khi setpoint bánh trong giảm mạnh tại waypoint, trạng thái integral cũ của đoạn
thẳng không còn phù hợp. Có thể:

- giới hạn tốc độ thay đổi setpoint RPM ở tầng motor;
- tracking integral theo output hiện tại;
- reset hoặc hiệu chỉnh integral khi đổi dấu/chuyển trạng thái;
- không reset integral ở mọi gói `CMD`, vì ROS gửi lệnh liên tục khoảng 25 Hz.

### 5.4. Mở rộng telemetry để kiểm chứng

STM32 nên gửi thêm:

```text
setpoint_rpm_L
setpoint_rpm_R
measured_rpm_L
measured_rpm_R
pwm_L
pwm_R
integral_L
integral_R
```

Khi đó có thể xác nhận trực tiếp:

- setpoint bánh phải vẫn dương;
- output/PWM có bị giữ ở 0 hay không;
- integral có bị kéo âm khi vào góc hay không;
- `MOTOR_PWM_MIN_R` đã đủ thắng ma sát hay chưa.

## 6. Quy trình kiểm chứng sau khi sửa firmware

### Bước 1 — Test từng bánh khi robot kê lên

- Ra setpoint 5, 10, 15, 20 và 30 RPM.
- Tăng rồi giảm setpoint nhanh.
- Kiểm tra bánh không đứng lâu khi setpoint vẫn dương.
- Ghi RPM, PWM và integral.

### Bước 2 — Test trên sàn với lệnh cố định

Thử các lệnh đã gây lỗi:

```text
v=0.063, w=0.550
v=0.087, w=0.451
v=0.097, w=0.352
```

Yêu cầu:

- cả hai RPM luôn lớn hơn 0 khi cả hai setpoint dương;
- không xuất hiện gyro tăng đột biến do pivot;
- motor không rung do start boost.

### Bước 3 — Chạy lại hình vuông Paper1-cu

Sau khi vòng motor hoạt động đúng, trả controller về góc gọn:

- `sharp_corner_w = 0.55 rad/s`;
- tỷ lệ bánh trong nền nhỏ, chỉ để bảo đảm setpoint dương;
- không cần anti-stall quá mạnh ở ROS;
- giữ `k3 = 7` vì giảm `k3` đã làm cạnh thẳng lượn hơn;
- đánh giá lại RMS, lateral bias, heading RMS và RPM từng bánh.

### Tiêu chí đạt

```text
RPM_inner > 0 trong toàn bộ pha góc
không có đoạn gyro tăng tương ứng với pivot một bánh
sai số ngang giảm về gần 0 trước cuối mỗi cạnh
waviness và heading RMS không xấu hơn baseline tốt nhất
```

## 7. Kết luận

Controller ROS đã phát lệnh để bánh trong chạy tiến, nhưng bánh phải vẫn về
0 RPM trong khi setpoint lý thuyết còn khoảng 15–20 RPM. Gyro đồng thời tăng
mạnh, xác nhận robot thực sự pivot bằng một bánh.

Nguyên nhân phù hợp với code STM32 hiện tại là integral wind-down kết hợp với
điều kiện deadzone không hoạt động khi PI output bị clamp bằng 0. Tăng `v`, giảm
`w` hoặc tăng gain BSMC chỉ làm góc rộng và cạnh thẳng xấu hơn mà không loại
được nguyên nhân gốc.

Vì vậy, muốn robot vừa quẹo liên tục bằng hai bánh vừa bám tốt cạnh thẳng, cần
sửa vòng PI/deadzone trên STM32, hiệu chuẩn riêng bánh phải và bổ sung telemetry.
Sau khi tầng motor bám đúng setpoint, mới nên tiếp tục tinh chỉnh BSMC và
boundary layer.

> Lưu ý: tài liệu này mới phân tích và đề xuất thay đổi. Chưa sửa hoặc flash
> firmware STM32. Việc flash cần project STM32CubeIDE/Makefile đầy đủ, thiết bị
> nạp và quy trình kiểm tra an toàn.
