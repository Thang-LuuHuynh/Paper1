# Hướng dẫn nâng `Paper1-cu` lên controller mượt của `Paper1-main`

Tài liệu này dùng cho người tiếp quản repository cũ trên GitHub. Mục tiêu là
đưa **source code** của `Paper1-cu` lên trạng thái đã thử nghiệm phần cứng trong
`Paper1-main` ngày 2026-07-22, đặc biệt là BSMC hình số 8, nhưng vẫn giữ lịch sử
Git và không chép các file build/cache.

## 1. Phạm vi và nguyên tắc an toàn

Chỉ sửa các file ROS 2/Python trong package `amr_control`. Không sửa firmware
ESP32/STM32, chiều motor, GPIO, PWM, encoder polarity, E-stop hoặc giới hạn vận
tốc phần cứng trong lần migration này.

Không chép các thư mục sau từ `Paper1-main`:

- `build/`
- `install/`
- `log/`
- `__pycache__/`
- `.pytest_cache/`
- dữ liệu CSV/PDF/PNG cũ

Quy trình bắt buộc:

1. Tạo branch hoặc backup của `Paper1-cu`.
2. Đối chiếu source.
3. Sửa từng nhóm chức năng.
4. Xem diff.
5. Chạy unit test và build.
6. Chỉ thử robot sau khi xác nhận đúng parameter và không có controller khác
   publish `/cmd_vel`.

Ví dụ tạo branch trước khi sửa:

```bash
cd /home/thang/Paper1/Paper1-cu
git status
git switch -c migrate-smooth-controller-20260722
```

## 2. Các file cần cập nhật

| File | Mục đích |
|---|---|
| `src/amr_control/amr_control/bsmc_eight.py` | Feedforward đầy đủ, gain theo vùng tâm, mặt trượt thích nghi, log thành phần điều khiển |
| `src/amr_control/amr_control/eight_profiles.py` | Profile hình số 8 mới |
| `src/amr_control/test/test_eight_profiles.py` | Khóa các giá trị profile bằng test |
| `src/amr_control/amr_control/camera.py` | Bù offset từ tâm AprilTag về tâm động học `base_link` |
| `src/amr_control/amr_control/embedded_paper_capture.py` | Log disturbance, sửa thời điểm kết thúc run, xuất metric/plot |
| `src/amr_control/amr_control/bsmc_square.py` | Làm mượt heading reference tại góc vuông |
| `src/amr_control/amr_control/bsmc_circle.py` | Gain trượt mới và phím đánh dấu disturbance |
| `src/amr_control/amr_control/bsmc_experiment.py` | CLI cho các parameter mới |

Nếu cả hai folder cùng tồn tại trên một máy, có thể dùng `diff -u` để xem từng
file. Không ghi đè ngay khi chưa đọc diff:

```bash
diff -u \
  /home/thang/Paper1/Paper1-cu/src/amr_control/amr_control/bsmc_eight.py \
  /home/thang/Paper1/Paper1-main/src/amr_control/amr_control/bsmc_eight.py
```

## 3. Cấu hình hình số 8 cần khóa

### 3.1. Gain cơ sở đã chạy trên robot

| Parameter | Giá trị |
|---|---:|
| `k1` | `0.2205844943` |
| `k2` | `6.5` |
| `k3` | `7.0` |
| `ks1` | `0.03683289` |
| `ks2` | `0.1159106176` |
| `phi1` | `1.0` |
| `phi2` | `1.5` |
| `amplitude` | `1.0 m` |
| `angular_speed` | `0.07 rad/s` |
| `trajectory_ramp_time` | `12.0 s` |
| `path_rotation_deg` | `-45.0 deg` |
| `max_v` | `0.18 m/s` |
| `max_w` | `0.85 rad/s` |

### 3.2. Feedforward và gain gần tâm

| Parameter | Giá trị mục tiêu r08 |
|---|---:|
| `w_feedforward_scale` | `1.0` |
| `w_feedforward_scale_negative` | `1.0` |
| `w_feedforward_scale_positive` | `1.0` |
| `negative_yaw_rate_feedback_gain` | `0.50` |
| `positive_yaw_rate_feedback_gain` | `0.30` |
| `feedback_speed_floor` | `0.05` |
| `center_k1` | `1.5` |
| `center_k1_radius` | `0.50 m` |
| `center_k2` | `18.0` |
| `center_k2_radius` | `0.80 m` |
| `center_k3` | `0.0` |
| `center_k3_radius` | `0.65 m` |
| `ki_y` | `0.0` |
| `i_y_max` | `0.2` |

`ki_y` đã được cài trong thuật toán nhưng đang **tắt** (`0.0`). Thử nghiệm phần
cứng cho thấy tích phân ngang tích lũy qua hai lobe và có thể chống lại hiệu
chỉnh tại tâm. Không tự bật tích phân khi chưa có protocol thử riêng.

### 3.3. Cảnh báo lệch profile hiện có

Run ba vòng r08 thực tế ghi trong summary:

```text
center_k3 = 0.0
```

Tuy nhiên, `Paper1-main/src/amr_control/amr_control/eight_profiles.py` tại thời
điểm viết tài liệu vẫn chứa `center_k3 = 1.0`. Nguyên nhân là hai đường chạy
khác nhau:

- `ros2 run amr_control bsmc_eight` lấy default trực tiếp từ
  `bsmc_eight.py`;
- runner tuning/experiment có thể lấy giá trị từ `eight_profiles.py`.

Để tái tạo đúng r08, phải đặt `center_k3 = 0.0` đồng nhất trong:

1. `bsmc_eight.py`;
2. `eight_profiles.py`;
3. `test_eight_profiles.py`;
4. command line hoặc YAML nếu có override.

Sau mỗi run, mở file `_summary.json` và kiểm tra `parameters.center_k3`; không
chỉ tin giá trị trong source.

## 4. Sửa `bsmc_eight.py`

### 4.1. Parameter và state mới

Trong `__init__`, khai báo:

```python
self.declare_parameter('w_feedforward_scale', 1.0)
self.declare_parameter('w_feedforward_scale_negative', 1.0)
self.declare_parameter('w_feedforward_scale_positive', 1.0)
self.declare_parameter('center_k2_radius', 0.80)
self.declare_parameter('center_k3', 0.0)
self.declare_parameter('center_k3_radius', 0.65)
self.declare_parameter('ki_y', 0.0)
self.declare_parameter('i_y_max', 0.2)
```

Sau khi đọc parameter, tạo state:

```python
self.i_y = 0.0
self.KI_Y = float(self.get_parameter('ki_y').value)
self.I_Y_MAX = float(self.get_parameter('i_y_max').value)
```

### 4.2. Không tính `s2` trước khi có gain theo tâm

Trong code cũ, `s2 = e_theta + c*e_y` được tính quá sớm nên dù `k3_control`
được giảm ở tâm, thành phần SMC vẫn giữ toàn bộ ảnh hưởng của `e_theta`.

Sau khi đã nội suy `k2_control` và `k3_control`, tính lại:

```python
if abs(self.k3) > 1e-9:
    s2_heading_scale = max(0.0, min(1.0, k3_control / self.k3))
else:
    s2_heading_scale = 1.0

s2 = s2_heading_scale * e_theta + self.c * e_y
sat_s2 = self.sat(s2 / self.phi2)
```

Ý nghĩa: khi tiến gần tâm và `k3_control` giảm về 0, cả feedback heading thông
thường lẫn phần heading nằm trong mặt trượt đều giảm tương ứng. `e_y` không còn
bị hai nhánh heading cùng chống lại.

### 4.3. Tách các thành phần `w_cmd`

Thay biểu thức cộng dài bằng các biến có thể log:

```python
v_feedback = max(v_d, self.FEEDBACK_SPEED_FLOOR)
w_ey = v_feedback * k2_control * e_y
w_eth = v_feedback * k3_control * math.sin(e_theta)
w_smc = self.Ks2 * sat_s2

self.i_y += e_y * self.timer_period
self.i_y = max(-self.I_Y_MAX, min(self.I_Y_MAX, self.i_y))
w_iy = self.KI_Y * self.i_y

w_ff = entry_scale * w_ff_scale * w_d
w_cmd_before_yaw_rate = (
    w_ff + w_ey + w_eth + w_smc + w_iy + self.yaw_feedforward
)
w_yaw_rate = yaw_rate_gain * (
    w_cmd_before_yaw_rate - self.current_w
)
w_cmd = w_cmd_before_yaw_rate + w_yaw_rate
```

Giữ nguyên clamp cuối cùng `MAX_V/MAX_W`. Không tăng giới hạn actuator trong
migration.

### 4.4. Log chẩn đoán ở tâm

Khi `distance_from_center < 0.2 m`, log ít nhất:

```text
w_ff, w_ey, w_eth, w_smc, w_iy, w_yaw_rate,
s2_heading_scale, w_cmd_before_yaw_rate, w_cmd
```

Log này dùng để phát hiện lại hiện tượng `w_ey` và `w_eth` triệt tiêu nhau.

## 5. Sửa `eight_profiles.py` và test

Trong profile `1m`, thay:

```python
"w_feedforward_scale": 1.0,
"w_feedforward_scale_negative": 1.0,
"w_feedforward_scale_positive": 1.0,
"center_k2_radius": 0.80,
"center_k3": 0.0,
"center_k3_radius": 0.65,
"ki_y": 0.0,
"i_y_max": 0.2,
```

Trong `test_eight_profiles.py`, assert đúng các giá trị trên. Test phải thất bại
nếu controller/profile vô tình quay về feedforward 0.55/0.80 hoặc bán kính tâm
0.65 cũ.

## 6. Bù offset AprilTag trong `camera.py`

Import `Parameter` sau dòng shebang:

```python
from rclpy.parameter import Parameter
```

Sau khi pose tag đã được tính/lọc và yaw đã có, đổi từ tâm tag về tâm động học:

```python
tag_offset_x = self.get_parameter_or(
    'tag_offset_x',
    Parameter('tag_offset_x', Parameter.Type.DOUBLE, 0.0557),
).value
tag_offset_y = self.get_parameter_or(
    'tag_offset_y',
    Parameter('tag_offset_y', Parameter.Type.DOUBLE, 0.0),
).value

yaw_rad = math.radians(self.pose['yaw'])
self.pose['x'] -= (
    math.cos(yaw_rad) * tag_offset_x
    - math.sin(yaw_rad) * tag_offset_y
)
self.pose['y'] -= (
    math.sin(yaw_rad) * tag_offset_x
    + math.cos(yaw_rad) * tag_offset_y
)
```

`0.0557 m` chỉ là giá trị đo trên robot hiện tại. Nếu phần cứng khác, đo vector
từ tâm trục bánh tới tâm AprilTag trong frame robot. Không trừ một hằng số trực
tiếp vào trục X/Y thế giới vì offset phải quay theo yaw.

Sau migration phải kiểm tra quay robot tại chỗ: nếu `/odom_camera` còn vẽ vòng
tròn lớn khi tâm động học đứng yên, offset hoặc yaw vẫn chưa đúng.

## 7. Làm mượt góc hình vuông trong `bsmc_square.py`

Thêm parameter:

```python
self.declare_parameter('heading_reference_rate', 0.85)
self.HEADING_REFERENCE_RATE = max(
    0.01, float(self.get_parameter('heading_reference_rate').value)
)
self.reference_theta = None
self.reference_segment_index = None
```

Trong `generate_path_following_trajectory`, không nhảy `theta_d` tức thời 90
độ. Rate-limit heading target và tính `w_d` nhất quán:

```python
remaining = self.normalize_angle(target_theta - self.reference_theta)
max_step = self.HEADING_REFERENCE_RATE * self.timer_period
theta_step = max(-max_step, min(max_step, remaining))
self.reference_theta = self.normalize_angle(
    self.reference_theta + theta_step
)
theta_d = self.reference_theta
w_d = theta_step / self.timer_period
```

Reset `reference_theta` và `reference_segment_index` khi anchor/reset một run
mới. Việc này vừa làm lệnh góc mượt hơn, vừa tránh spike sai số góc do reference
nhảy rời rạc.

## 8. Disturbance capture

### `bsmc_circle.py`

- Import `Bool`.
- Tạo publisher `/disturbance_event`.
- Trong keyboard loop, phím `d` publish `Bool(data=True)` tại thời điểm người
  thử nghiệm **thả robot** sau khi gây nhiễu.

Giá trị mặc định hiện tại trong `Paper1-main` là:

```python
ks1 = 0.1
ks2 = 0.2
```

Đây là thay đổi controller, không chỉ logging. Nếu dùng dữ liệu paper cũ có
profile Circle khác, phải khóa gain theo protocol và ghi đúng giá trị từ
summary; không trộn các run khác gain vào cùng thống kê.

### `embedded_paper_capture.py`

Thêm cột CSV và subscription cho `disturbance_event`. Khi export, tính:

- `disturbance_event_time_s`;
- `disturbance_peak_position_error_m`;
- `disturbance_peak_heading_error_rad`;
- `disturbance_iae_10_m_s`;
- `disturbance_recovery_time_s`.

Recovery được định nghĩa là sai số vị trí dưới `0.03 m` liên tục ít nhất `2 s`.
Xuất thêm plot `_disturbance_recovery.png/.pdf`.

Quan trọng: sửa stop timer để thời lượng Circle/Eight bắt đầu từ clock thực của
controller (`node.start_time`) và trừ thời gian pause. Nếu dùng thời điểm tạo
capture, startup camera/EKF có thể làm run kết thúc thiếu phần cuối quỹ đạo.

## 9. CLI trong `bsmc_experiment.py`

Thêm argument và validation:

```text
--start-phase-deg
--ki-y
--i-y-max
```

Quy tắc:

```text
ki_y >= 0
i_y_max > 0
```

Đảm bảo `resolve_eight_profile()` và lệnh ROS con truyền các giá trị này xuống
node. Gain trong manifest/summary cũng phải lưu `ki_y` và `i_y_max`.

## 10. Build và test offline

Chạy trong `Paper1-cu` sau khi sửa:

```bash
cd /home/thang/Paper1/Paper1-cu
source /opt/ros/humble/setup.bash
python3 -m pytest src/amr_control/test/test_eight_profiles.py -q
colcon build --packages-select amr_control --symlink-install
source install/setup.bash
```

Sau build, kiểm tra parameter mà không cho robot chạy nếu có thể bằng unit test
hoặc một node test riêng. Trước thử phần cứng:

```bash
ros2 node list
ros2 topic list
ros2 topic hz /odom_camera
ros2 topic hz /odometry/filtered
ros2 topic hz /espnow_link
```

Chỉ chạy đúng camera profile hình số 8:

```bash
ros2 run amr_control camera_eight
```

## 11. Lệnh validation hình số 8

Để tránh profile/default lệch nhau, lần validation đầu nên truyền rõ parameter
quan trọng trên CLI:

```bash
ros2 run amr_control bsmc_eight --ros-args \
  -p paper_laps:=3.0 \
  -p paper_run_id:=eight_bsmc_migrated_r01 \
  -p center_k3:=0.0 \
  -p center_k2_radius:=0.80 \
  -p ki_y:=0.0
```

Trước khi Enter:

- robot đứng đúng tâm và yaw gần 0 độ;
- camera, EKF, serial, ESP-NOW đang cập nhật;
- không có controller khác publish `/cmd_vel`;
- vùng chạy trống, người vận hành giữ E-stop/ngắt nguồn;
- không đổi gain giữa run.

Sau run, mở `_summary.json` và xác nhận:

```text
k1=0.2205844943, k2=6.5, k3=7.0
ks1=0.03683289, ks2=0.1159106176
center_k2=18.0, center_k2_radius=0.8
center_k3=0.0, ki_y=0.0
requested_laps=3.0
```

## 12. Mốc nghiệm thu tham khảo từ r08

Run phần cứng r08 ba vòng đạt:

| Metric | r08 |
|---|---:|
| Position RMS | `2.748 cm` |
| Path RMS | `1.66 cm` |
| Heading RMS | `3.555 deg` |
| Crossing path RMS | `0.96 cm` |
| Lateral RMS trái sang phải | `0.263 cm` |
| Lateral RMS phải sang trái | `1.389 cm` |
| Peak `abs(w_cmd)` | `0.314 rad/s` |
| Bão hòa `w_cmd` | `0%` |
| Camera age max | `62.7 ms` |

Ba vòng có position RMS lần lượt `2.763`, `2.752`, `2.730 cm`, không có drift
tăng theo vòng. Đây là mốc tham khảo để phát hiện regression, không phải điều
kiện để loại một run paper chỉ vì RMSE cao.

Nhánh phải sang trái vẫn khó hơn: lateral RMS khoảng `1.39 cm`, heading RMS
khoảng `7.59 deg`. Không tuyên bố lỗi giao điểm đã được loại bỏ hoàn toàn.

## 13. Checklist hoàn tất migration

- [ ] Chỉ source/test/config được chuyển; không chép build/cache/data.
- [ ] `bsmc_eight.py`, `eight_profiles.py`, test và CLI thống nhất
      `center_k3=0.0` cho target r08.
- [ ] Feedforward ba hướng đều bằng `1.0`.
- [ ] `center_k2_radius=0.80`.
- [ ] Mặt trượt `s2` dùng `s2_heading_scale`.
- [ ] Integral có anti-windup nhưng `ki_y=0.0` mặc định.
- [ ] Camera bù offset theo yaw và offset đã được đo trên robot.
- [ ] Square heading reference được rate-limit và reset đúng lúc.
- [ ] Disturbance event xuất hiện trong CSV/summary/plot.
- [ ] Unit test pass.
- [ ] `colcon build --packages-select amr_control` pass.
- [ ] Summary của run xác nhận đúng gain thực tế.
- [ ] Robot tự dừng và publish zero velocity sau run.

## 14. Ghi chú cho dữ liệu paper

Một run ba vòng chỉ là một experimental unit (`n=1`), không phải `n=3` độc
lập. Để làm bảng/kết luận chính, giữ nguyên code/gain đã khóa và chạy tối thiểu
3, khuyến nghị 5 run khởi động độc lập cho BSMC và cùng số run Backstepping.
Backstepping ablation chỉ được khác bằng `Ks1=Ks2=0`; các shaping, camera, EKF,
giới hạn actuator và trajectory phải giống BSMC.

Gọi các metric trên là **camera-derived tracking errors**. AprilTag không phải
independent ground truth tuyệt đối vì camera cũng tham gia EKF feedback.
