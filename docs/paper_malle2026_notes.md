# Ghi chú: bài báo Malle & Ebeid 2026, ánh xạ với hệ US-D1

*Ghi 2026-10-27, để trao đổi lại sau. Phần làm hiện tại tập trung vào mô phỏng Ainstein US-D1.*

**Bài báo:** N. H. Malle, E. Ebeid. *Omnidirectional Solid-State mmWave Radar Perception for UAV Power
Line Collision Avoidance*, arXiv:2602.03229 (ICRA 2026); nguồn [1] trong `radar_survey.md`.
Mã nguồn: github.com/nhma20/spherical-radar-drone (driver: github.com/nhma20/xwr6843_ros2).

## Họ làm gì

- **Phần cứng:**
  - Quad QAV250, Pixhawk 4 Mini với PX4 1.13 (RTPS), Raspberry Pi 4 với ROS 2 Foxy.
  - 6 radar TI IWR6843 (60–64 GHz), firmware có sẵn, chế độ 3D, 10 Hz.
- **Các radar:**
  - **Trước:** bo ISK tầm xa, đo được 122° ngang × 41° dọc.
  - **Năm hướng còn lại** (sau, trên, dưới, trái, phải): bo AOP, đo được khoảng 75° ngang × 104–135° dọc.
  - Hai radar trái và phải xoay 15° về phía trước.
  - Sai số tầm đo với corner reflector ở 1 m: trung bình 6,07 cm, σ 2,79 cm.
- **Xử lý:**
  - Dùng **đám điểm** trực tiếp trong khung UAV. Không lọc đất, không gom cụm, không bản đồ.
  - Quan sát chính: khi radar nhìn gần vuông góc với dây (trong khoảng ±30°), điểm phát hiện nằm gần điểm gần nhất trên dây.
- **Tránh vật cản (thuần phản ứng):**
  1. Hình cầu tránh r_a: với mỗi điểm nằm phía trước hướng bay, cộng vector tiếp tuyến t = (p̂ × ĝ) × p̂ vào vận tốc người lái.
  2. Phanh khẩn cấp:
     - Điều kiện: có điểm trong hình nón quanh hướng bay, dài l_H = v²/(2a_max) + s_margin, nửa góc 0,245 rad.
     - Phanh kéo tới khi tốc độ < v_eb.
  3. Hình cầu an toàn r_s: v = k_s(‖p‖ − r_s)·p̂, đẩy UAV ra xa điểm.
  - Vận tốc đã sửa gửi xuống PX4. Bài báo không cho giá trị các tham số.
- **Kết quả:**
  - Đường dây ba pha dài khoảng 35 m: ba dây 20 mm, một dây 10 mm, cách nhau khoảng 3 m, cao khoảng 9 m.
  - Radar trước thấy dây từ hơn 10 m, các radar khác từ khoảng 7 m.
  - Né ở 10 m/s phía trước, 5 m/s các hướng khác. Thấy cả dây thép 1,2 mm.
  - Không lần nào UAV đến gần dây quá 1 m.
  - **Không báo:** độ trễ, tỉ lệ thành công, tỉ lệ phát hiện theo khoảng cách.

## Khác biệt căn bản với hệ của mình

**US-D1 không cho đám điểm.** Mỗi gói chỉ có một khoảng cách và một SNR (manual D00.02.05, mục 5.1),
không có góc, không có nhiều mục tiêu. Mô phỏng của mình làm đúng như vậy
(`profiles/us_d1.yaml`, `output: altimeter`). Hệ quả:
- Không tính được vector tiếp tuyến chính xác. Hướng của vật chỉ biết là "trong búp 43° × 30°", tức sai tới ±21,5°.
- Phanh hình nón và hình cầu an toàn vẫn dùng được, vì chỉ cần khoảng cách và hướng trục búp.

## Ánh xạ

| Khía cạnh | Bài báo | Hệ US-D1 hiện tại |
|---|---|---|
| Cảm biến | 6 × IWR6843, đám điểm 3D | 3 × US-D1, một khoảng cách mỗi cảm biến |
| Vùng phủ | Gần như toàn cầu, cả trên và dưới | Khoảng 130° phía trước, chỉ ngang |
| Nơi xử lý | Node ROS 2 trên Pi, vận tốc offboard | `CollisionPrevention` của PX4 (72 ô 5°, chỉ Position mode) |
| Thuật toán | Tiếp tuyến + phanh hình nón + hình cầu an toàn, có trục đứng | Giảm tốc và dừng trước `CP_DIST`, chỉ mặt phẳng ngang |
| Quãng hãm | l_H = v²/(2a) + margin | Cùng công thức (mục 2 của `radar_survey.md`). Đã đo trong sim: 7,9 m/s thì vào sát 4,5 m với `CP_DIST` = 5 m |
| Vật mảnh | Dây 20/10/1,2 mm thật | Mô hình chưa làm được dây: tia Gazebo cách nhau 1° (17 cm ở 10 m) |

## Hướng phát triển đã đề xuất (chưa làm)

1. **Mô hình dây điện trong `radar_model`:** đoạn thẳng có đường kính, RCS của trụ mảnh, điểm gần nhất khi nhìn trong ±30° vuông góc. Kèm world đường dây ba pha giống thí nghiệm và dây 1,2 mm. Dùng được cho cả US-D1 lẫn IWR6843.
2. **Cấu hình 6 × IWR6843 theo FOV và sai số đo trong bài, đặt cạnh 3 × US-D1:** cùng world, cùng kịch bản.
3. **Node ROS 2 theo phương trình 1–11 của bài báo:**
   - Vận tốc offboard qua `~/px4_ws/src/px4_msgs`.
   - Với US-D1 thì chỉ dùng phanh hình nón và hình cầu an toàn; với IWR6843 thì thêm tiếp tuyến.
   - So với CP có sẵn của PX4.
4. **Đo những gì bài báo thiếu:** tỉ lệ phát hiện theo tầm cho từng cỡ dây, tỉ lệ né thành công qua nhiều lần chạy, độ trễ từ scan tới lệnh.
5. **Hai điểm còn hở trong bài báo:** chưa lọc đất và nhiễu, và giả định "điểm gần nhất" sai dần ngoài ±30°.

## Cập nhật 2026-10-27

Đã dọn mọi profile và world của radar cũ (EchoFlight, omni 50 m, µSharp Patch, world 1 km;
bản lưu: `px4-mapping_old_radars_2026-10-27.tar.gz` trong thư mục scratchpad của phiên làm việc).
Thêm `profiles/iwr6843isk.yaml` (radar trước của bài) và viết lại `iwr6843aop.yaml` (radar bên) theo
số đo trong bài. Radar trước đã chạy trong Gazebo và RViz (`./run_sitl.sh`, `hover_test.py`).
Chưa có: bốn radar AOP còn lại gắn trên vehicle, node tránh vật cản theo phương trình 1–11, mô hình dây.

## Cập nhật 2026-10-27 (chiều): vòng 3 radar và node tránh vật cản

Đã làm hướng 1–3 ở trên, với 3 ISK thay vì 6 radar (đủ 360° ngang): `models/x500_radar3`,
`radar_model.wire_detections` (RCS trụ mảnh), `scripts/radar_array.py`, `scripts/avoid_node.py`
(phương trình của bài, tham số là của ví dụ này), world `power_line` và `power_line_thin`.
Tỉ lệ thành công: 5/5 với ba dây 20 mm ở 5 m/s, 3/3 với dây 1,2 mm ở 2 m/s
([radar3_avoidance_results.md](radar3_avoidance_results.md)).
Hai điều bài báo không nói nhưng phải xử lý: bù độ trễ của đám điểm, và giữ phanh khi một khung radar bị rơi.

## So với mã nguồn của tác giả (đọc 2026-10-27)

github.com/nhma20/spherical-radar-drone (`src/offboard_control.cpp`, `launch/spherical_radar.launch.py`) và
github.com/nhma20/xwr6843_ros2 (driver, `cfg_files/`). Bài báo không in tham số, nhưng mã nguồn có.

**Thuật toán** (`OffboardControl::speed_limiter`, 10 Hz, đầu vào là cần điều khiển của người lái):

| | Mã của tác giả | `avoid_node.py` |
|---|---|---|
| Hình cầu tránh ("caution") | 2,5 m; cắt bớt thành phần vận tốc hướng vào điểm, hệ số 0,1 | r_a 6 m; không có |
| Tiếp tuyến | `(p × z) × p`, dấu chọn theo vận tốc mong muốn, hệ số 0,25; cộng thêm một số hạng theo vận tốc thật | `(p × g) × p`, hệ số 1 |
| Hình cầu an toàn | 0,75 m; nếu có điểm trong đó thì **chỉ** trả về vector đẩy ra | r_s 1,5 m; cộng vào lệnh |
| Phanh khẩn cấp | nón nửa góc atan(1/4) = 0,245 rad; kích khi thời gian tới điểm < 1,5 × thời gian dừng (a_max 10 m/s²), chỉ khi tốc độ > 2 m/s; lệnh lùi 2 m/s theo hướng bay, kèm tiếp tuyến nhỏ; nhả khi tốc độ < 2 m/s | cùng nón; chiều dài v²/2a + margin (a 3 m/s², margin 3 m); lệnh 0; giữ tới < 0,3 m/s và khi nón còn điểm |
| Giới hạn tốc độ theo tầm nhìn | có: tối đa √(2·a·16/1,5) phía trước (tầm 16 m), √(2·a·8/1,5) hai bên | không |
| Làm mượt lệnh | lọc 0,85 / 0,15 | không |
| Gộp đám điểm | node `radar_pointcloud_combiner`, khung `drone_yaw_only`, chỉ x, y, z | `radar_array.py` + 0,3 s trong node, khung thế giới |

**Radar** (cấu hình `xwr6843ISK_profile_10Hz_v2.cfg` mà launch dùng cho radar trước):

| | Cấu hình thật | `profiles/iwr6843isk.yaml` |
|---|---|---|
| Độ phân giải tầm | 0,122 m | 0,1 m (ước lượng) |
| Vùng tầm | 0,2–30 m (`cfarFovCfg`) | 0,2–40 m (ước lượng) |
| Độ phân giải phương vị | 15° | ô góc 5° (ước lượng) |
| Ngưỡng CFAR | 15 dB | 12 dB (ước lượng) |
| Vùng góc | az ±70°, el ±30° (`aoaFovCfg`); launch lọc 140° × 45° | 122° × 41° (số đo trong bài) |
| Tốc độ khung | 10 Hz | 10 Hz |

Driver phát `PointCloud2` trên `xwr6843_pcl` (đổi tên thành `/front_pcl`...), trường x, y, z (+ vel, snr, noise
tuỳ chọn); khung radar của TI nhìn theo +y, không theo +x như ở đây.

**Cách gắn 6 radar** (`spherical_radar.launch.py`, `static_transform_publisher` x y z yaw pitch roll; chú thích
"yaw, roll, pitch" trong file ghi sai, chỉ thứ tự yaw-pitch-roll cho radar front nhìn về trước; khung radar TI
nhìn theo +y, quạt phương vị theo x):

| Radar | Nhìn về | Quạt rộng nằm theo | FOV driver lọc |
|---|---|---|---|
| front (ISK) | trước | **đứng** (lên–xuống) | 140° × 45° |
| rear (AOP) | sau | ngang | 85° × 120°* |
| top, bot (AOP) | lên, xuống | trái–phải | 85° × 120°* |
| left, right (AOP) | hai bên, xoay 15° về trước | trước–sau | 85° × 120°* |

\* Launch đặt `radar_azimuth_fov` hai lần (lần sau bằng `ELEVATION_AOP` = 85) và không đặt elevation, nên driver
lọc 85° × 120° (mặc định), không phải 110° × 85° như ý định.

Độ phủ mọi hướng (tính bằng `scripts/radar_layouts.py`): bố trí của tác giả 82% (FOV đo trong bài), 90% (FOV
driver lọc); `x500_radar6` (6 ISK cùng vị trí, trước/sau quạt đứng) 71%; `x500_radar3` 35%.

Hệ quả cho mô phỏng: ô góc 15° và ngưỡng 15 dB làm số điểm mỗi khung ít đi. Mô hình dây đang tính SNR theo diện
tích ô góc, nên nếu đổi sang 15° thì cần hiệu chỉnh lại `snr_ref` (để dây 20 mm vẫn thấy từ hơn 10 m, như bài báo),
rồi đo lại.

## Né theo tiếp tuyến của bài báo (2026-10-29)

Bản đầu của `avoid_node.py` lấy tiếp tuyến (p̂ × ĝ) × p̂ với ĝ là **hướng tới đích**. Với dây chắn ngang ngay
trước mặt, p̂ ∥ ĝ nên tiếp tuyến bằng 0 và chỉ còn luật phanh. Trong bài (Sec. IV, Scenario 1), ĝ_n là **trục
trọng lực**: t = (p̂ × ĝ_n) × p̂ = ẑ − (p̂·ẑ) p̂, nằm trong mặt phẳng đứng chứa p̂. Mã của tác giả cũng làm vậy
(`unit_z` trong `speed_limiter`). Bản `--mode tangent` (`tangent_avoid`, mặc định) theo đúng bài:

- Dấu của t chọn sao cho t·v > 0; thành phần là (p̂·v̂) t̂. Lặp lại với v_u (vận tốc tới waypoint) và v_d (vận tốc
  thật), cộng dồn qua mọi điểm, rồi v_out = min(1, |v_u| / |t_s + v_u|)(t_s + v_u).
- Trọng số theo độ sâu trong r_a và k_t·|v| lấy theo mã tác giả, (1 − (|p| − r_s)/(r_a − r_s)).
- Phanh khẩn cấp chỉ khi |v| > v_eb (2 m/s, như mã tác giả), nhả khi xuống dưới v_eb rồi quay lại luật tiếp tuyến.
- Trong r_s: chỉ gửi vector đẩy ra (Scenario 3; mã tác giả cũng `return` ngay).
- Dây thấp hơn thì leo qua, cao hơn thì chui dưới. Dây ngang tầm có t·v ≈ 0 nên sẽ đổi dấu theo nhiễu góc ngẩng;
  dưới ngưỡng `--tie` 0,1 thì tính là leo qua.

Phần thêm ngoài bài báo, `--corridor` 2 m: chỉ những điểm cách đường bay (tia theo v) dưới 2 m mới sinh tiếp tuyến.
Lý do: mặt phẳng tiếp tuyến của bài là mặt cắt ngang qua một dây nằm ngang. Với cột đứng bên cạnh đường bay, các
điểm thấp hơn drone cho t nghiêng **vào** cột. Ở lượt SITL #101 (chưa có corridor), drone leo lên trên ngọn thân
cây (15 m) và bị kéo ngang 2,7 m, chỉ còn cách cây 1,18 m. Cột đứng ngay trên đường bay thì vẫn chỉ leo qua.
Luật tiếp tuyến đứng không vòng ngang được, và bài báo cũng không có cách nào khác.

Thử động học (vận tốc giới hạn gia tốc 4 m/s², đủ 4 hướng nhìn, `wire_detections` cho mọi dây và cột): mọi lượt
đều qua mà không phanh. Dây 20 mm ngang tầm ở 5 m/s cách gần nhất 2,35 m. Dưới dây 2 m: chui dưới, 2,23 m. Dây
1,2 mm ở 5 m/s: 2,27 m; ở 2 m/s: 1,72 m.

## Bố trí 6 radar tối ưu, có AOP (2026-10-29)

So sánh đầy đủ: README, mục "Bố trí radar". Tóm tắt: sáu ISK phủ tối đa 71,2% (bố trí lập phương). Một ISK + năm
AOP phủ 90,6% (98% dải ±30°): ISK trước quạt đứng; AOP sau quạt rộng nằm ngang; AOP trái/phải xoay 15° về trước,
quạt rộng theo trục trước–sau; AOP trên/dưới quạt rộng theo trục trái–phải (`radar_layouts.LAYOUTS["aop"]`, vehicle `x500_radar_aop`).
Đây là ý định của launch file tác giả (chú thích trong file). Đọc khung radar TI như driver thì cách họ lắp thật chỉ
phủ 83%. Thả tự do hướng năm AOP chỉ lên 91,0% và giảm dải ±30°. Sáu AOP phủ 96% nhưng không hướng nào thấy dây xa
hơn khoảng 7 m.

## Né bốn hướng: độ dịch nhỏ nhất và tiếp tuyến tỉ lệ (2026-11-03)

Tiếp tuyến đứng của bài chỉ cho lên hoặc xuống. Ở chuỗi vật cản, drone đâm `pole_right`: một cột đứng chắn đường
không vượt lên được. Bản mặc định giờ là `--way-round shortest`, phần thêm ngoài bài báo:

1. **Chọn hướng (`way_round`).** Thử 16 hướng × 0,1–5 m dịch đường bay trong mặt phẳng vuông góc với hướng bay. Chọn
   độ dịch có chi phí nhỏ nhất, chi phí gồm:
   - độ dài của độ dịch;
   - 2 × phần còn thiếu so với `--clearance` 2 m;
   - 10 × phần còn thiếu so với r_s.

   Dưới r_s/2 thì cấm hẳn. Khi gần hoà, thứ tự ưu tiên là lên, trái, phải, xuống. Đổi phía phải rẻ hơn 1 m
   (`--switch`). Chỉ xét điểm phía trước, cách đường bay < 5 m, trước waypoint đang bay tới (+ r_s). Kết quả: dây
   ngang thì vượt trên hoặc chui dưới như bài; cột bên cạnh thì vòng ngang; khối thì vòng phía trống; cổng hẹp hơn
   2 × clearance thì bay giữa.
2. **Vận tốc (`way_round_velocity`).** Đi theo tiếp tuyến chạm vòng an toàn của điểm cản gần nhất:
   - thành phần vuông góc = tốc độ tiến × độ dịch / (khoảng cách tới điểm đó − r_s), ít nhất min(độ dịch, 1) m/s;
   - không quá 2,5 m/s; dốc hơn thế thì giảm tốc độ tiến, giữ độ dốc;
   - lên ≤ 3 m/s, xuống ≤ 1,5 m/s (MPC_Z_VEL_MAX_UP/_DN).

   Cùng một luật cho cả bốn hướng.

Thứ tự luật không đổi: hình cầu an toàn, rồi phanh khẩn cấp (chỉ > v_eb), rồi caution sphere 2,5 m, rồi né.
Tiếp tuyến đứng từng điểm của bài vẫn có ở `--way-round paper`, và dùng khi không có độ dịch nào hợp lệ.

Các lỗi đã gặp (lượt SITL, [radar3_avoidance_results.md](radar3_avoidance_results.md)):
- #601: một cột *sau* waypoint làm đường bay kéo dài đổi phía liên tục. Drone lắc 120 s. Sửa: horizon.
- #603: hướng né nhân với tổng trọng số mọi điểm. Một cột hàng trăm điểm cho 4,9 m/s ngang dù chỉ cần dịch 0,1 m,
  drone văng 5,6 m. Sửa: luật tiếp tuyến tỉ lệ ở trên.
- Độ dịch 0 bị tính nhầm chi phí 0,15 m (như hướng xuống), nên có vật phía trước là luôn dịch lên 0,1 m. Đã sửa.
- #702: vài điểm sát giữa cổng làm phương án bay giữa thành bị cấm. Drone đổi sang vòng ngoài cột với lệnh ngang tối
  đa, cách cột 0,78 m. Sửa: r_s thành chi phí mềm, tốc độ ngang ≤ 2,5 m/s.

Bản cuối: chuỗi vật cản 3/3 (≥ 1,83 m), 10 m/s 2/2, close proximity 2/2. Mô phỏng động học 5 seed: 1,91–2,05 m.
