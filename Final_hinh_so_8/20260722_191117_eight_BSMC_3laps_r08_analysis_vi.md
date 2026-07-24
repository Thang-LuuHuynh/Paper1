# Phân tích BSMC hình số 8 — r08, 3 vòng

## Tính hợp lệ của run

- Run ID: `eight_bsmc_ff1_center_s2_k2r080_3laps_r08`
- Hoàn thành theo pha: 2.9996 vòng (sai khác do rời rạc lấy mẫu ở 25 Hz)
- Số mẫu dùng tính metric: 6756
- Camera age: P95 = 44.1 ms, P99 = 45.3 ms, max = 62.7 ms; đều dưới ngưỡng audit 300 ms
- ESP-NOW inter-arrival: P95 = 58.4 ms, P99 = 69.9 ms, max = 106.4 ms
- Không có sequence gap trong log nên không suy ra packet-loss chính xác
- Đỉnh lệnh góc: 0.314 rad/s trên giới hạn 0.85 rad/s; không có mẫu bão hòa
- Controller tự dừng và odometry sau run xác nhận robot đứng yên

## Kết quả toàn run

| Metric | Giá trị |
|---|---:|
| RMSE e_x | 2.183 cm |
| RMSE e_y | 1.670 cm |
| RMSE vị trí | 2.748 cm |
| RMSE góc | 3.555 deg |
| Bias e_x | -1.783 cm |
| Bias e_y | +0.884 cm |
| Bias góc | -1.161 deg |
| Max abs(e_x) | 5.027 cm |
| Max abs(e_y) | 3.319 cm |
| Max sai số vị trí | 5.202 cm |
| Max abs(e_theta) | 12.386 deg |
| Path RMSE theo báo cáo tuning | 1.66 cm |
| Crossing path RMSE theo báo cáo tuning | 0.96 cm |

## Độ lặp theo vòng

| Vòng | Position RMSE (cm) | Heading RMSE (deg) | Max position (cm) | Max abs heading (deg) |
|---:|---:|---:|---:|---:|
| 1 | 2.763 | 3.781 | 5.202 | 12.386 |
| 2 | 2.752 | 3.554 | 5.044 | 12.102 |
| 3 | 2.730 | 3.313 | 5.080 | 11.708 |
| Mean +/- sample SD | 2.748 +/- 0.017 | 3.549 +/- 0.234 | 5.109 +/- 0.083 | 12.065 +/- 0.341 |

Ba vòng chồng khít và không có xu hướng drift tăng theo thời gian. Tuy nhiên,
ba vòng trong cùng một lần khởi động là repeated measures, không phải ba run
độc lập; không dùng `n=3` cho kiểm định thống kê.

## Hai hướng qua giao điểm (bán kính 0.15 m)

| Hướng | Vòng | RMSE e_y (cm) | Max abs(e_y) (cm) | RMSE góc (deg) | Max abs(góc) (deg) |
|---|---:|---:|---:|---:|---:|
| Trái sang phải | 1 | 0.247 | 0.372 | 1.948 | 2.776 |
| Trái sang phải | 2 | 0.261 | 0.389 | 2.387 | 3.484 |
| Trái sang phải | 3 | 0.276 | 0.409 | 2.107 | 2.822 |
| Phải sang trái | 1 | 1.461 | 2.686 | 8.447 | 10.118 |
| Phải sang trái | 2 | 1.419 | 2.352 | 7.317 | 9.234 |
| Phải sang trái | 3 | 1.279 | 2.225 | 6.932 | 8.604 |

Nhánh trái sang phải rất tốt và lặp lại. Nhánh phải sang trái còn bias e_y
khoảng +1.00 cm toàn run và heading RMS 7.59 deg, nhưng không tăng theo vòng;
thực tế cải thiện dần từ vòng 1 tới vòng 3.

## So với r07 một vòng

- Position RMS: r08 = 2.748 cm, r07 = 2.760 cm (tương đương).
- Path RMS: r08 = 1.66 cm, r07 = 1.606 cm (r08 xấu hơn nhẹ).
- Heading RMS: r08 = 3.555 deg, r07 = 3.346 deg (r08 xấu hơn nhẹ).
- Crossing L->R lateral RMS: r08 = 0.263 cm, r07 = 0.317 cm (r08 tốt hơn).
- Crossing R->L lateral RMS: r08 = 1.389 cm, r07 = 1.206 cm (r08 xấu hơn).

Provenance cần khóa lại trước validation: r08 dùng `center_k3=0.0`, trong khi
summary r07 ghi `center_k3=1.0`. `eight_profiles.py` đang ghi 1.0 nhưng
`bsmc_eight.py` hiện khai báo default 0.0; executable lấy giá trị sau. Do đó
r08 không phải repeat cùng tham số với r07.

## Kết luận cho paper

Run này đủ chất lượng làm figure minh họa hoặc pilot result: quỹ đạo đẹp,
position RMS dưới 3 cm, path RMS dưới 2 cm, ba vòng không drift và không bão
hòa actuator. Chưa đủ làm bằng chứng thực nghiệm cuối cùng hoặc tuyên bố BSMC
tốt hơn Backstepping.

Trước khi đưa vào bảng/kết luận chính cần:

1. Khóa một giá trị `center_k3` duy nhất và đồng bộ code, summary, Methods.
2. Chạy tối thiểu 3, khuyến nghị 5 run khởi động độc lập cho BSMC, mỗi run 3 vòng.
3. Chạy cùng số run Backstepping với mọi tham số giữ nguyên, chỉ đặt Ks1=Ks2=0.
4. Dùng run làm đơn vị thống kê; báo mean +/- sample SD theo run, không coi 15 vòng là n=15.
5. Gọi metric là camera-derived tracking error, không gọi AprilTag là independent ground truth vì camera tham gia EKF feedback.
6. Chỉ báo ESP-NOW inter-arrival cho log này; không báo one-way latency hoặc packet loss khi chưa có sequence.
