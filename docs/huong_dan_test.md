# Hướng dẫn test: vòng 3 radar IWR6843 và tránh dây điện

Các bước để tự chạy lại mọi bài test của cấu hình 3 radar (`models/x500_radar3`) và node tránh vật cản
(`scripts/avoid_node.py`). Mọi lệnh chạy trong `examples/px4-mapping/`.

## 0. Cần có

- PX4 SITL đã build. `run_sitl.sh` dùng `build/px4_sitl_mapping` nếu có (`scripts/build_px4.sh`), không thì
  `build/px4_sitl_default`. Cấu hình 3 radar không cần driver riêng nào, build nào cũng được.
- Gazebo Harmonic, ROS 2 Humble (`/opt/ros/humble`), Python: numpy, PyYAML, pymavlink, pytest.
- Không có sim khác đang chạy: `run_sitl.sh` chờ tới khi máy không có `gz sim` hay ArduPilot SITL nào trong
  60 s liền (`QUIET_TIME=5` để chờ ngắn hơn).

## 1. Một lần: sinh model radar

```bash
scripts/make_isk_x3.sh
```

Sinh `models/radar_isk_0`, `radar_isk_120`, `radar_isk_240` (và `radar_mmwave`) từ `profiles/iwr6843isk.yaml`.
Chạy lại mỗi khi sửa profile.

## 2. Unit test (không cần Gazebo, vài giây)

```bash
python3 -m pytest tests -q
```

Phải ra `44 passed`. Gồm mô hình radar, điểm của dây (`tests/test_wires_avoid.py`: dây 20 mm thấy ở 10 m,
dây 1,2 mm chỉ ở gần, dây ngoài vùng nhìn không thấy) và ba luật tránh vật cản (phanh, giữ phanh, tiếp tuyến,
hình cầu an toàn, giới hạn tốc độ).

## 3. Xem bằng mắt: Gazebo + RViz, một chuyến bay

Terminal 1:

```bash
RADARS=3 ./run_sitl.sh                        # world power_line; PX4_GZ_WORLD=power_line_thin cho dây 1,2 mm
```

Mở Gazebo (camera bám UAV, khung FOV ba radar) và RViz (`rviz/radar3.rviz`: đám điểm màu theo SNR, khung FOV,
dây thật màu đỏ). Chờ dòng `pxh>` (hoặc `PX4 SITL ready` khi có `PX4_DAEMON=1`), rồi thêm khoảng 15 s.

Terminal 2:

```bash
source env.sh
scripts/avoid_node.py --goal 45 0 10 --speed 5                          # ba dây 20 mm
scripts/avoid_node.py --world power_line_thin --goal 45 0 10 --speed 2  # dây 1,2 mm (sim chạy world đó)
```

Node cất cánh, leo lên 10 m, bay tới đích, tự tránh. Kết thúc khi tới đích, khi phanh giữ UAV đứng yên 5 s,
hoặc khi va chạm; sau đó hạ cánh và in:

```
held by the emergency brake
final    (22.8, -0.6, 9.7), 0.08 m/s
speed    5.1 m/s at most
closest  2.70 m to trunk (wire radius subtracted)
result:  PASS
```

`closest` là khoảng cách thật (Gazebo) tới dây hoặc vật cản gần nhất trong suốt chuyến bay. PASS khi
≥ `--min-distance` (mặc định 1 m, như bài báo). Mã thoát: 0 PASS, 1 FAIL, 2 lỗi thiết lập.

UAV hạ cánh tại chỗ dừng: muốn bay lại thì tắt sim (Ctrl+C ở terminal 1, hoặc `pxh> shutdown`) và chạy lại bước này.

## 4. Đo tỉ lệ thành công (headless, nhiều lượt)

Không cần terminal nào khác: script tự khởi động sim không GUI trước mỗi lượt và tắt sau lượt cuối.
Tắt sim đang chạy ở bước 3 trước.

```bash
scripts/avoid_trials.sh 5 power_line 5          # 5 lượt, ba dây 20 mm + thân cây, 5 m/s
scripts/avoid_trials.sh 3 power_line_thin 2     # 3 lượt, dây 1,2 mm, 2 m/s
scripts/avoid_trials.sh 3 power_line_thin 5 4   # tiếp tục đánh số từ lượt 4 (tham số thứ 4)
```

Mỗi lượt mất 2–3 phút. Kết quả:

- `out/trials/summary.txt`: một dòng mỗi lượt (lý do kết thúc, khoảng cách gần nhất);
- `out/trials/<world>_<tốc độ>_<n>.txt`: toàn bộ output của lượt đó;
- `out/trials/<world>_<tốc độ>_<n>.csv`: log 20 Hz: `t, x, y, z` (vị trí, khung Gazebo ENU), `vx, vy, vz`,
  `cmd_x, cmd_y, cmd_z` (vận tốc node ra lệnh), `brake` (1 khi phanh khẩn cấp), `points` (số điểm radar đang
  dùng), `d_cloud` (điểm radar gần nhất), `d_wire`, `d_obstacle` (khoảng cách thật tới dây và vật cản).

Đọc nhanh một lượt:

```bash
python3 -c "
import csv; r = list(csv.DictReader(open('out/trials/power_line_5_1.csv')))
print('min wire %.2f, min obstacle %.2f, end x %.1f' % (min(float(a['d_wire']) for a in r),
      min(float(a['d_obstacle']) for a in r), float(r[-1]['x'])))"
```

Dòng `no flight (see ...)` trong summary là lượt lỗi thiết lập (sim chưa lên, không có heartbeat): không tính.

## 5. Đổi tham số

`scripts/avoid_node.py --help` liệt kê hết. Hay dùng:

| Cờ | Mặc định | Ý nghĩa |
|---|---|---|
| `--speed` | 5 | tốc độ bay tới đích (m/s) |
| `--avoid-radius` | 6 | r_a: hình cầu tránh (luật tiếp tuyến), m |
| `--safety-radius` | 1,5 | r_s: hình cầu an toàn (đẩy ra), m |
| `--a-max`, `--margin` | 3, 3 | chiều dài hình nón phanh l_H = v²/2a + margin |
| `--brake-speed` | 0,3 | v_eb: phanh giữ tới khi chậm hơn mức này |
| `--keep` | 0,3 | gộp đám điểm trong bao nhiêu giây |
| `--goal X Y Z` | 45 0 10 | đích, toạ độ Gazebo (UAV xuất phát ở gốc) |

Vật cản nằm trong world SDF: `worlds/power_line.sdf` (dây là model `wire_*`, thân cây `trunk`). Sửa world xong
không cần build gì; node và bộ chấm điểm đọc thẳng file.

## 6. Khi có lỗi

| Triệu chứng | Nguyên nhân thường gặp |
|---|---|
| `run_sitl.sh` chưa mở gì | đang chờ máy rảnh 60 s (`scripts/sim_guard.sh`, in dòng `no other simulation; ...` hoặc tên sim đang chạy); `QUIET_TIME=5` để chờ ít hơn |
| `A Gazebo world is already running` | còn sim cũ của ví dụ này: `scripts/stop_sim.sh` |
| Một lần chạy bị treo, cần dừng hết | `scripts/stop_sim.sh` (chỉ dừng tiến trình của px4-mapping) |
| `no MAVLink heartbeat` | PX4 chưa lên, hoặc cổng 14540 đang bị tiến trình khác giữ |
| `not in Offboard mode (nav_state 14)` | PX4 từ chối Offboard; xem `run/px4.log` (preflight, GPS) |
| `Preflight Fail: Strong magnetic interference` | world thiếu `<spherical_coordinates>` (copy từ `worlds/radar_hover.sdf`) |
| Không có điểm radar trong RViz | xem `run/radar_array.log`; cần đã chạy `scripts/make_isk_x3.sh` |
