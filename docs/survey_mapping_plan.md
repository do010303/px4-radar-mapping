# Kế hoạch: lập map bằng radar + RTK (ý 1, chỉ radar)

Mục tiêu: từ dữ liệu radar và tư thế drone (GPS RTK), xác định **vị trí và point cloud của từng vật**, theo
hai chế độ:

- **Thời gian thực:** bản đồ dựng trên drone, gửi về trạm mặt đất trong giới hạn **300 kbit/s**.
- **Xử lý sau chuyến bay:** từ log đầy đủ ghi trên drone (luôn ghi, kể cả khi đang chạy thời gian thực).

Làm trên SITL hiện có, chỉ trong `examples/px4-mapping` (không sửa `src/`, `boards/`, `px4_sitl_default`).

## Giả định

| Hạng mục | Giá trị |
|---|---|
| Radar | IWR6843 ISK + 5 AOP (`x500_radar_aop`), 10 Hz |
| Định vị | RTK fix, σ ≈ 2 cm (`SIM_GZ_GPS_PNOIS 0.02`; các param `SIM_GZ_GPS_*` đã có trong build `px4_sitl_mapping`) |
| Đường truyền | 300 kbit/s, tính cả header |
| Xử lý trên drone | máy tính đồng hành (companion), trong sim là một tiến trình riêng |

## Điểm khác so với phần né vật cản

Hiện `radar_array.py` đặt điểm radar bằng **pose ground truth của Gazebo**, nên bản đồ tốt hơn thực tế.
Bài mới phải dùng **pose ước lượng của PX4** (EKF2 + RTK), giống ngoài thực tế; ground truth chỉ dùng để chấm điểm.

Công thức đưa một detection về hệ thế giới, với pose nội suy đúng thời điểm quét:

$$p_w = R(t)\,\big(R_m\,p_r + l\big) + t(t)$$

với $p_r$ là điểm trong hệ radar, $R_m, l$ là góc lắp và vị trí lắp radar trên khung, $R(t), t(t)$ là tư thế
và vị trí drone.

## Chỉ số đánh giá

- **Vị trí vật:** cột: lệch ngang của trục; dây: khoảng cách đến đường dây thật; khối/tường: lệch tâm và kích thước.
- **Point cloud:** khoảng cách điểm đến bề mặt thật (median, 95 %); độ phủ (% bề mặt có điểm trong 0,3 m).
- **Vật giả / bỏ sót.**
- **Thời gian thực:** kbit/s (trung bình, đỉnh), độ trễ, chất lượng so với bản offline của cùng chuyến bay.

## Các giai đoạn

### GĐ0. Bãi thử và mission (0,5 ngày)

- World `worlds/survey_site.sdf`: 2 cột, 2–3 dây, một khối nhà nhỏ, một bức tường; file đáp án
  `survey_site.truth.yaml` (giống `radar_hover.targets.yaml`).
- `scripts/survey_mission.py`: bay các đường song song qua vật ở 2–3 độ cao và một vòng quanh vật, khoảng cách
  10–30 m. Dùng lại phần offboard MAVLink của `avoid_node.py`.
- RTK trong sim: `SIM_GZ_GPS_PNOIS`, `SIM_GZ_GPS_EPH/EPV` nhỏ, chọn được qua biến môi trường của `run_sitl.sh`.
- **Xong khi:** bay hết mission; ulog có `vehicle_attitude`, `vehicle_local_position`, `sensor_gps`.

### GĐ1. Ghi dữ liệu như phần cứng thật (1 ngày)

- `radar_array.py --log DIR`: mỗi khung của mỗi radar ghi timestamp cảm biến, số hiệu radar và các detection
  **trong hệ radar** (r, az, el, v, snr), chưa biến đổi. Kèm pose ground truth, chỉ để chấm.
- Pose ước lượng lấy từ **ulog PX4**; hardlink ulog sang `run/log_archive` trước khi bay.
- Lỗi đưa vào để thử độ nhạy: lệch đồng hồ radar–PX4 (`--clock-offset`, `--clock-drift`), sai góc lắp
  (`--mount-error`), RTK float hoặc mất fix.
- **Xong khi:** một chuyến bay cho một thư mục log đọc lại được; biết dung lượng log mỗi phút.

### GĐ2. Đưa điểm về hệ thế giới, offline (1–1,5 ngày)

- `scripts/survey_process.py`: đọc log → nội suy pose tại thời điểm quét (vị trí tuyến tính, quaternion SLERP)
  → công thức trên → lọc Doppler, giữ điểm tĩnh → gộp → xuất PLY và hình.
- Ước lượng lệch thời gian từ chính dữ liệu: chọn offset làm đám điểm gọn nhất (ngoài thực tế bắt buộc phải có).
- Thí nghiệm tách nguồn sai số: pose truth / EKF + RTK / EKF + GPS thường; thêm lệch 20 và 50 ms; thêm lệch lắp 1°.
  Kết quả là **bảng ngân sách sai số đo được**, thay cho ước lượng.
- **Xong khi:** point cloud các vật nằm đúng chỗ, có bảng sai số.

### GĐ3. Tách vật (1,5 ngày)

- Lọc bóng ma: chỉ giữ voxel có điểm từ ≥ N lần quét và từ ≥ 2 vị trí nhìn khác nhau.
- Phân cụm DBSCAN, rồi khớp hình dạng đơn giản:
  - dây (dài, mảnh, gần ngang): RANSAC đường thẳng, sau đó catenary;
  - cột (đứng): hình trụ;
  - khối: hộp bao và tâm.
- Đầu ra: `objects.yaml` (loại, vị trí, kích thước, độ bất định, số điểm) và một file PLY cho mỗi vật.
- **Xong khi:** chấm tự động với file đáp án của world.

### GĐ4. Tinh chỉnh vị trí bằng khoảng cách, multilateration (2 ngày)

- **Ý tưởng:** góc radar sai vài độ (1–2 m ở 30 m) nhưng khoảng cách chỉ sai vài cm. RTK cho biết drone ở đâu
  với sai số cm, nên vị trí vật giải được từ các khoảng cách đo ở nhiều vị trí bay.
- Với điểm (cột, góc nhà): $\min_x \sum_k w_k\big(\lVert x - s_k\rVert - r_k\big)^2$, trong đó $s_k$ là vị trí
  radar và $r_k$ là khoảng cách đo ở lần quét $k$. Với dây: dùng khoảng cách đến đường thẳng.
- Khởi tạo từ GĐ3, gán detection theo cổng, dùng hàm mất mát Huber, ước lượng hiệp phương sai để có độ bất định.
- **Phải kiểm tra:** sim trả khoảng cách đến điểm nào trên vật. Nếu điểm phản xạ không giống thực tế, kết quả sẽ
  lạc quan; điều này cần ghi rõ trong báo cáo.
- **Xong khi:** so được sai số GĐ3 với GĐ4 trên cùng log. Mục tiêu giả thuyết: dưới 0,2 m ở 30 m cho cột và dây.

### GĐ5. Thời gian thực trong 300 kbit/s (2 ngày)

- `scripts/onboard_mapper.py` (vai trò companion):
  - pose PX4 thời gian thực qua MAVLink `ODOMETRY` hoặc `ATTITUDE_QUATERNION`, khoảng 50 Hz;
  - đưa điểm về hệ thế giới như GĐ2;
  - voxel map 0,3 m, đếm hit / log-odds.
- **Kênh 300 kbit/s mô phỏng:** token bucket trên UDP localhost giữa drone và trạm mặt đất, đo kbit/s thật.
- **Các mức gửi**, chọn theo băng thông còn lại; log đầy đủ luôn ghi song song:

  | Mức | Nội dung | Ước lượng |
  |---|---|---|
  | L1 | point cloud nén, 8 B/điểm, đã giảm mẫu | ~60–170 kbit/s |
  | L2 | chỉ các voxel thay đổi | vài chục kbit/s |
  | L3 | danh sách vật | vài kbit/s |

- `scripts/ground_viewer.py`: hiển thị trên RViz bản đồ nhận được.
- Lợi ích phụ: voxel map này dùng lại được cho `avoid_node.py` (việc "local memory map" đang treo).

### GĐ6. Đánh giá và tài liệu (1–1,5 ngày)

- Ma trận thử: 2 world × 3 mission × {truth, RTK, GPS thường} × {lệch đồng hồ, lệch lắp} × 3 lần.
- `docs/survey_mapping_results.md`, thêm mục trong README, pytest cho phần đưa điểm về hệ thế giới,
  multilateration và đo băng thông.

**Tổng: khoảng 10 ngày làm việc.** Nếu thiếu thời gian, làm GĐ0 → 1 → 2 → 3 trước: chừng đó đã cho vị trí và
point cloud của vật (offline). Sau đó làm GĐ5, cuối cùng GĐ4.

## Rủi ro

- **Sim radar không mô phỏng pha hay điểm phản xạ thật**, nên multilateration có thể đẹp hơn thực tế. Cần dữ liệu
  radar thật để xác nhận.
- **Độ phân giải góc kém:** point cloud sẽ không ra hình dạng chi tiết. Phần đó thuộc ý 2 (camera) hoặc SAR (cần
  raw ADC, ngoài khả năng của sim).
- **Tải máy và sim ArduPilot được ưu tiên:** chạy đêm, kiểm tra hợp lệ từng lượt (`cloud_check`, độ trễ radar).

## Ngoài phạm vi

Camera, SAR/raw ADC, bay thật.

## Kết quả GĐ0 (06/11/2026)

Đã có:

- `worlds/survey_site.sdf` và file đáp án `worlds/survey_site.truth.yaml` (tên `.targets.yaml` đã được
  `hover_test` dùng cho việc khác).
- `scripts/survey_mission.py`: 34 waypoint, 455 m. Mission bay qua `avoid_node.py`, tức là **luôn bật né vật
  cản**. Trước khi cất cánh, script kiểm tra đường bay cách mọi vật ≥ 3 m. Sau chuyến bay, CSV, output và
  hardlink tới ulog được lưu vào `out/survey/<tag>.*`.
- `run_sitl.sh GPS=rtk|std`. Khi không đặt `GPS`, cấu hình giữ nguyên như cũ, nên các bài né vật cản không đổi.
- `scripts/pose_check.py`: so pose ước lượng với ground truth, cả hai đều có sẵn trong ulog SITL.

Bay `PX4_GZ_WORLD=survey_site RADARS=aop GPS=rtk`: hết mission, PASS, gần nhất 4,3 m (tường), radar không trễ.

Sai số pose (ước lượng so với thực tế, phần đang bay):

| | `SENS_GPS0_DELAY` 110 ms (mặc định) | `SENS_GPS0_DELAY` 0 (`GPS=rtk`) |
|---|---|---|
| Vị trí ngang, median / 95 % | 0,32 / 0,58 m | 0,09 / 0,21 m |
| Vị trí đứng, median / 95 % | 0,03 / 0,14 m | 0,05 / 0,12 m |
| Nghiêng, median / 95 % | 0,8 / 1,8° | 0,4 / 0,8° |
| Heading, median / 95 % | 4,1 / 7,7° | 0,5 / 1,4° |
| Sai lệch do tư thế ở 20 m, 95 % | 2,7 m | 0,5 m |

Nhận xét:

- **Trễ GPS khai báo sai là nguồn sai số lớn nhất.** Ước lượng đi trước thực tế 0,11 s, kéo theo heading lệch
  vài độ. Ngoài thực tế cũng phải đặt `SENS_GPS0_DELAY` đúng bằng độ trễ của bộ thu RTK.
- **Sau khi sửa, heading vẫn là phần lớn của sai số do tư thế** (0,5 m ở 20 m, 95 %). Heading lệch quá 3°
  chỉ xảy ra lúc cất cánh (1,2 % thời gian). GPS hai anten sẽ giảm thêm phần này, nhưng sim chưa có.
- Còn lệch gốc toạ độ khoảng 9 cm. Mức này nhỏ so với sai số góc của radar (1–2 m ở 30 m).

Chuyến bay với trễ sai được giữ lại để so sánh: `out/survey/rtk_delay011*.ulg`.

## Kết quả GĐ1 (06/11/2026)

Đã có:

- `scripts/radar_log.py`: ghi mỗi khung của mỗi radar như phần cứng xuất ra.
  - Nội dung: timestamp và các điểm (x, y, z, v, SNR) trong hệ của chính radar đó, ghi trước bộ lọc Doppler.
  - Kèm pose thật của radar, chỉ dùng để chấm điểm.
  - Mỗi file `chunk_NNNN.npz` chứa 5 s; khi bị dừng (SIGTERM) chỉ mất tối đa phần chưa ghi. Có test đọc/ghi lại.
- `radar_array.py --log DIR`, `run_sitl.sh RADAR_LOG=1`. Khi không bật thì không có gì thay đổi, nên các bài
  né vật cản không đổi (85 test pass).
- `survey_mission.py` hardlink bản ghi vào `out/survey/<tag>.radar/`.
- `scripts/survey_check.py <tag>`: báo tốc độ khung, số điểm, khoảng hở, dung lượng, băng thông, và đo độ lệch
  đồng hồ giữa radar và ulog.

Chuyến `rtk_2` (`GPS=rtk RADAR_LOG=1`, PASS, gần nhất 4,2 m):

| | Kết quả |
|---|---|
| Khung | 6 radar × 10 Hz, khoảng hở dài nhất 0,10 s (không mất khung) |
| Điểm mỗi khung khi bay | ISK phía trước 11; AOP: dưới 4,7, trái 4,1, phải 3,1, sau 1,7, trên 0 |
| Dung lượng | 0,8 MB/phút |
| Băng thông định dạng TI khi bay | median 50, 95 % 136, max 247 kbit/s (vừa 300 kbit/s) |
| Băng thông khi còn trên mặt đất | median 229, max 350 kbit/s (radar thấy mặt đất ở cự ly gần) |
| Băng thông dạng nén 8 B/điểm | trung bình 30 kbit/s |
| Đồng hồ radar so với ulog PX4 | lệch −6 ms (sai số vị trí còn lại 2,3 cm), tức là khớp |

Lưu ý:

- **Radar thật nhiều khả năng có nhiều điểm hơn** (clutter, đa đường, mặt đất khi bay thấp), nên con số băng
  thông ở trên là mức dưới.
- **Lỗi lệch đồng hồ và lệch góc lắp** sẽ được thêm ở bước xử lý (GĐ2, `--clock-offset`, `--mount-error`), thay
  vì lúc ghi. Trong sim hai cách cho kết quả như nhau, và một chuyến bay dùng được cho nhiều mức lỗi.
- **Mất RTK fix** dùng `GPS=std`.

## Kết quả GĐ2 (06/11/2026)

`scripts/survey_process.py <tag>` đặt các điểm radar vào hệ world (ENU) bằng **pose ước lượng trong ulog**:

- Điểm gốc của EKF (`ref_lat/lon/alt`) được so với toạ độ địa lý của world, nên bản đồ ở toạ độ tuyệt đối như
  khi dùng RTK.
- Vị trí nội suy tuyến tính, quaternion nội suy nlerp (250 Hz).
- Tính cả vận tốc do drone xoay tại vị trí lắp radar.
- Sau đó lọc Doppler bằng vận tốc ước lượng.

Đầu ra là `<tag>.map.ply` và `<tag>.map.png`, kèm điểm chấm so với `survey_site.truth.yaml`.
`--table` in bảng ngân sách sai số; `--find-clock` tìm độ lệch đồng hồ radar–PX4 từ chính dữ liệu.

**Phát hiện 1: heading sai lúc đầu chuyến bay.** Trước lần tăng tốc ngang đầu tiên, heading chỉ dựa vào từ kế
và lệch tới −23°, trong khi EKF tự báo σ 4–7°. Radar ISK đặt điểm lệch tới 7 m. Heading chỉ đúng lại (0,5°)
khi drone bắt đầu bay ngang (54 s). Cách xử lý: bỏ các khung có σ heading của EKF > 1,5° (`estimator_states`
covariance thứ 3). Ngoài thực tế dùng được ngay vì không cần ground truth. Ở `rtk_2` bước này bỏ 696 khung (~7 %).
Khi bay thật nên có một đoạn tăng tốc ngang trước khi tới khu khảo sát.

**Bảng ngân sách sai số** (`rtk_2`, cùng các khung, lọc Doppler 3σ; "trên vật" là điểm cách vật gần nhất ≤ 2 m):

| Phương án | Điểm trên vật | Median (m) | 95 % (m) | Trong 0,5 m | > 2 m | Độ phủ: dây, cột, nhà, tường |
|---|---|---|---|---|---|---|
| Pose thật | 20396 | 0,45 | 1,58 | 60 % | 4,6 % | 28, 100, 10, 8 % |
| Pose ước lượng (RTK) | 20354 | 0,46 | 1,59 | 60 % | 4,7 % | 26, 100, 10, 8 % |
| + lệch đồng hồ 20 ms | 20295 | 0,46 | 1,58 | 60 % | 4,7 % | 26, 100, 9, 8 % |
| + lệch đồng hồ 50 ms | 20195 | 0,48 | 1,59 | 59 % | 4,7 % | 27, 100, 10, 7 % |
| + lệch đồng hồ 100 ms | 19555 | 0,53 | 1,60 | 56 % | 4,9 % | 28, 100, 10, 5 % |
| + lệch lắp 1° | 20264 | 0,46 | 1,59 | 60 % | 4,7 % | 28, 100, 9, 9 % |
| + lệch lắp 2° | 20139 | 0,47 | 1,61 | 59 % | 4,8 % | 26, 100, 9, 9 % |
| + lệch lắp 5° | 19260 | 0,54 | 1,66 | 54 % | 6,4 % | 22, 100, 9, 8 % |

GPS thường (`std_1`, `GPS=std`: pose lệch ngang median 1,1 m, đứng 2,7 m, phần lớn là độ lệch cố định):

| Phương án | Trong 0,5 m | Median trên vật (m) | 95 % (m) | > 2 m |
|---|---|---|---|---|
| Pose thật | 79 % | 0,40 | 1,56 | 2,2 % |
| Pose ước lượng (GPS thường) | 39 % | 0,57 | 1,60 | 2,5 % |

(Tỷ lệ "trong 0,5 m" của `std_1` và `rtk_2` không so trực tiếp với nhau được: `std_1` bay thấp hơn khoảng 2,7 m nên
thấy nhiều mặt đất hơn. Trong cùng một chuyến thì so được.)

**Kết luận:**

- **Với RTK và trễ GPS đặt đúng, pose không còn là vấn đề.** Pose ước lượng gần như tốt bằng pose thật
  (0,46 so với 0,45 m). Sai số còn lại là của radar: sai số góc làm các cột bị tán rộng 1–2 m.
- **Mức chịu được:** lệch đồng hồ ≤ 50 ms và lệch lắp ≤ 2° gần như không ảnh hưởng. Từ 100 ms hoặc 5° thì bắt
  đầu thấy rõ.
- **`--find-clock`** tìm lại được độ lệch đồng hồ trong khoảng ±20 ms, dựa trên độ gọn của đám điểm: đặt sai
  100 ms thì nó trả về −0,02 s, giá trị thật ≈ 0.
- **GPS thường làm hỏng bản đồ:** tỷ lệ điểm nằm đúng trên vật giảm một nửa, chủ yếu do lệch độ cao.

**Giới hạn của sim, cần lưu ý cho GĐ3:**

- Nhà và tường có rất ít điểm: khoảng 1,4 điểm mỗi khung khi bay ngang, độ phủ 8–10 %. Cột dùng mô hình phản xạ
  hình trụ riêng nên rất mạnh, còn bề mặt rộng thì yếu (giới hạn 2 ô mỗi range bin với AOP, cạnh tranh với
  mặt đất).
- Dây chỉ phủ 22–44 %.

Cần xem lại mô hình bề mặt rộng trước khi rút kết luận về hình dạng nhà hoặc tường.

## Kết quả GĐ3 (06/11/2026)

`scripts/survey_objects.py <tag>`, đầu vào là các điểm do `survey_process.place` đặt:

1. **Mặt đất:** RANSAC tìm mặt phẳng nằm ngang, lấy mẫu trong 30 % số điểm thấp nhất. Khi lấy mẫu trong toàn bộ
   điểm, có lần nó chọn mặt phẳng ngang dây ở 10,7 m.
2. **Bóng ma:** giữ voxel 0,5 m nào có điểm từ ≥ 3 khung và được nhìn từ các vị trí cách nhau ≥ 2 m.
3. **Cột:** tìm các ô lưới 0,5 m có điểm xếp chồng ≥ 3 m theo chiều cao, rồi phân cụm. Trục cột lấy ở chỗ dày
   điểm nhất của cụm. (Nếu phân cụm toàn bộ điểm, hai cột bị nối với nhau qua dây.)
4. **Dây:** RANSAC tìm đường gần nằm ngang, dài ≥ 5 m, trong phần điểm còn lại sau khi bỏ bán kính 3 m quanh
   mỗi cột.
5. **Phần còn lại:** phân cụm thành hộp.

Đầu ra: `<tag>.objects.yaml`, `<tag>.objects/<tên>.ply`, `<tag>.objects.png`, kèm điểm chấm so với đáp án.
Thêm 2 test trên dữ liệu tổng hợp (`tests/test_survey_objects.py`, tổng 87 test).

**Thay đổi mission:** chuyến `rtk_2` gần như không có điểm ở đoạn giữa nhịp dây (0–15 điểm mỗi dây). Lý do:
- AOP chỉ thấy dây 20 mm trong khoảng ~7 m;
- drone giữ hướng mũi +x, nên ISK nhìn dọc theo dây chứ không vuông góc.

Vì vậy `survey_mission.py` thêm hai lượt dọc tuyến: trên đỉnh tuyến (y = 0, 16,5 m) và bên cạnh (y = 8, 10 m).
Mission mới có 38 waypoint, 574 m. Chuyến `rtk_3` PASS, gần nhất 3,1 m (đỉnh cột); băng thông bay vẫn như cũ.

**Kết quả `rtk_3`:**

| Vật | Pose ước lượng (RTK) | Pose thật |
|---|---|---|
| `pylon_a` / `pylon_b` (trục) | 0,03 / 0,14 m | 0,01 / 0,01 m |
| `lamp_post` / `trunk` (trục) | 0,06 / 0,03 m | 0,02 / 0,02 m |
| Chiều cao cột | −0,19 đến +0,07 m | −0,20 đến +0,07 m |
| `wire_c` | 0,02 m, 93 % chiều dài | 0,02 m, 96 % |
| `wire_b` | 0,04 m, 77 % | 0,04 m, 76 % |
| `wire_earth` | 0,05 m, 76 % | 0,02 m, 77 % |
| `wire_a` (xa lượt bay bên cạnh nhất) | 0,20 m, 22 % | 0,36 m, 23 % |
| Tay xà | gộp vào cột | gộp vào cột |
| Tường | chỉ thấy mép trên, tâm lệch 3 m | không thấy |
| Nhà | không thấy | không thấy |
| Vật giả | 1 (20 điểm) | 2 đoạn dây ngắn (34, 51 điểm) |

**Nhận xét:**

- **Vị trí cột và dây chính xác hơn nhiều so với từng điểm radar.** Từng điểm lệch 1–2 m, nhưng trục cột ra
  0,01–0,14 m và dây ra 0,02–0,05 m. Lý do: sai số góc của radar ngẫu nhiên, nên khi gộp hàng nghìn điểm nhìn
  từ nhiều vị trí, sai số triệt tiêu nhau.
- **Pose RTK ước lượng không làm kết quả kém đi** so với pose thật.
- **Hệ quả cho GĐ4:** mục tiêu đặt ra (< 0,2 m ở 30 m) đã đạt cho cột và dây mà chưa cần multilateration.
  Trong sim, GĐ4 sẽ khó cho thêm cải thiện, nên đề xuất để GĐ4 sau, khi có dữ liệu radar thật.
- **Vẫn còn yếu:**
  - Bề mặt rộng (nhà, tường): mô hình radar trong sim cho rất ít điểm, xem GĐ2.
  - `wire_a`: chỉ thấy được 22 % chiều dài. Cần một lượt bay ở phía y âm, nhưng phía đó vướng cột đèn và thân cây.

## Kiểm tra độ nhạy trong sim (bắt đầu 06/11, xong 10/11/2026; kết quả đầy đủ ở mục sau)

**Mục đích:** chưa có radar thật, nên kiểm tra xem kết luận GĐ2–3 có còn đúng không nếu radar thật khác sim.

Đã có:

- Trong `radar_array.py`, các tuỳ chọn độ nhạy áp đồng đều cho cả 6 radar: `--angle-scale`, `--surface-db`,
  `--thin-db`, `--points-scale` (cùng với `--angle-outlier` đã có sẵn). Ở giá trị mặc định thì không có gì thay đổi.
- `scripts/survey_sensitivity.sh [biến thể...]`: bay mỗi biến thể một chuyến.
- `scripts/survey_compare.py`: in mỗi chuyến một dòng.

| Biến thể | Trạng thái |
|---|---|
| `angle2`, `thin-5` | đã bay xong |
| `ghost5`, `surface+10`, `points3`, `bright`, `worst` | chưa bay |

**Chạy tiếp lần sau** (khoảng 35 phút; tự in bảng đầy đủ vào `out/survey/sensitivity.md`):

    scripts/survey_sensitivity.sh ghost5 surface+10 points3 bright worst

Kết quả tạm (`out/survey/sensitivity_partial.md`):

| Chuyến | Biến thể | Trong 0,5 m | Median trên vật | > 2 m | Cột | Dây | Vật giả | kbit/s khi bay (median / 95 % / max) |
|---|---|---|---|---|---|---|---|---|
| `rtk_3` | cơ sở | 60 % | 0,41 m | 4,6 % | 4/4 (≤ 0,14 m) | 4/4 (≤ 0,20 m) | 1 | 53 / 135 / 244 |
| `sens_angle2` | sai số góc ×2 | 38 % | 0,70 m | 14,7 % | 4/4 (≤ 0,12 m) | 3/4 (≤ 0,08 m) | 4 | 55 / 140 / 266 |
| `sens_thin-5` | dây và cột −5 dB | 70 % | 0,37 m | 3,4 % | 4/4 (≤ 0,12 m) | **0/4** | 0 | 38 / 99 / 248 |

Nhận xét tạm:

- **Vị trí cột vững** với cả hai biến thể.
- **Tìm dây rất nhạy với tầm thấy dây.** Chỉ cần −5 dB (tầm thấy giảm khoảng 30 %) là không tìm được dây nào,
  vì lượt bay dọc tuyến cách dây 4–7 m, đúng ở rìa tầm thấy của AOP. Đây là thông số cần đo đầu tiên khi có radar
  thật, và có thể phải bay gần dây hơn.
- **Sai số góc ×2** làm bản đồ thô hơn (median 0,41 → 0,70 m) và nhiều vật giả hơn (1 → 4), nhưng vẫn tìm được
  vật.

## Kết quả kiểm tra độ nhạy (10/11/2026)

Đủ 7 biến thể cùng chuyến cơ sở `rtk_3`, tất cả `GPS=rtk`. Bảng gốc ở `out/survey/sensitivity.md`.

| Biến thể | Mission (gần nhất) | kbit/s khi bay, TI (median / 95 % / max) | Nén 8 B (95 % / max) | Trong 0,5 m | Median trên vật | Cột | Dây | Nhà / tường | Vật giả |
|---|---|---|---|---|---|---|---|---|---|
| cơ sở | PASS (3,13 m) | 53 / 135 / 244 | — | 60 % | 0,41 m | 4/4 | 4/4 | không / mép | 1 |
| sai số góc ×2 | PASS (3,07 m) | 55 / 140 / 266 | — | 38 % | 0,70 m | 4/4 | 3/4 | không / không | 4 |
| dây và cột −5 dB | PASS (3,13 m) | 38 / 99 / 248 | — | 70 % | 0,37 m | 4/4 | **0/4** | không / mép | 0 |
| 5 % bóng ma | PASS (3,17 m) | 54 / 136 / 244 | — | 59 % | 0,41 m | 4/4 | 3/4 | không / mép | 2 |
| bề mặt +10 dB | PASS (3,16 m) | 77 / **286 / 489** | 107 / 189 | 75 % | 0,37 m | 4/4 | 4/4 | 2,5 / 2,1 m | **12** |
| điểm ×3 | PASS (3,13 m) | 58 / 151 / **351** | 54 / 134 | 63 % | 0,41 m | 4/4 | 4/4 | 2,2 / 2,9 m | 0 |
| bề mặt +10 dB và điểm ×3 | PASS (3,16 m) | 77 / **387 / 826** | 148 / **323** | 81 % | 0,36 m | 4/4 | 3/4 | 2,5 / 2,6 m | **13** |
| tệ nhất (góc ×2, −5 dB, 5 % bóng ma) | PASS (3,24 m) | 62 / 168 / 256 | 60 / 95 | 48 % | 0,59 m | 3/4 (sót cột đèn) | **0/4** | không / mép | 4 |

Sai số của cột và dây tìm được giữ ở ≤ 0,14 m trong mọi biến thể; biến thể chỉ ảnh hưởng tới việc **có tìm thấy
hay không**.

**Kết luận nào đứng vững:**

- **Né vật cản an toàn ở mọi biến thể,** kể cả tệ nhất: 8/8 PASS, gần nhất 3,07–3,24 m (đường bay được thiết kế
  cách vật 3,3 m).
- **Cột:** tìm được và đúng vị trí (≤ 0,14 m) ở mọi biến thể. Chỉ ở biến thể tệ nhất là sót cột đèn mảnh (r = 0,1 m).
- **Bóng ma 5 % gần như không ảnh hưởng.** Lọc Doppler và lọc voxel nhìn nhiều lần đã loại gần hết: tỷ lệ điểm
  > 2 m chỉ tăng 4,6 → 5,5 %.
- **Pose RTK không phải giới hạn.** Độ thô của bản đồ do sai số góc radar quyết định (×2 thì median 0,41 → 0,70 m).

**Kết luận phụ thuộc radar thật (cần đo trước tiên):**

1. **Tầm thấy dây.** Chỉ cần −5 dB là mất cả 4 dây, vì lượt bay dọc tuyến cách dây 4–7 m, đúng ở rìa tầm thấy của
   AOP. Nếu radar thật kém hơn sim, phải bay gần hơn hoặc dùng ISK nhìn vuông góc với dây (quay mũi drone, hoặc
   lắp ISK hướng ngang).
2. **Độ phản xạ của bề mặt rộng.** Khi tăng +10 dB thì nhà và tường hiện ra, nhưng sinh ra 12–13 vật giả:
   - mép mái nhà bị RANSAC nhận nhầm thành "dây" ở z ≈ 4 m;
   - bức tường 6 m bị nhận thành "cột".

   Bước tách vật cần thêm một lớp **bề mặt/khối** (phẳng, rộng), tìm trước dây và cột, và dây phải đủ mảnh và
   đứng riêng. Việc này chỉ đáng làm khi biết radar thật thấy bề mặt tới mức nào.
3. **Băng thông.**
   - Ở cơ sở, dữ liệu định dạng TI vừa 300 kbit/s.
   - Khi bề mặt sáng hơn hoặc có nhiều điểm hơn, đỉnh lên 351–826 kbit/s (95 % tới 387). Vậy **không thể đẩy nguyên
     stream TI qua đường truyền 300 kbit/s.**
   - Dạng nén 8 B/điểm vẫn vượt 300 kbit/s trong vài giây ở biến thể sáng nhất (max 323).
   - Do đó GĐ5 phải có giảm mẫu thích ứng hoặc chỉ gửi voxel thay đổi, không chỉ nén.

**Thứ tự đo khi có radar thật:** tầm thấy dây 10–20 mm (E1) → độ phản xạ của tường/mái (E1, E2) → số điểm mỗi
khung khi bay (E3).

## Kết quả GĐ5, phần 1–4 (10/11/2026, đánh giá bằng phát lại các chuyến đã ghi)

Chuỗi xử lý: `onboard_mapper.py` (trên drone) → `radar_link.py` (đường truyền) → `ground_map.py` (mặt đất,
RViz: `rviz/ground.rviz`). Đánh giá từ đầu đến cuối bằng `realtime_eval.py`, bảng gốc ở `out/survey/realtime.md`
và `realtime_delay.md`. Có thêm 6 test (tổng 93).

**Bộ dựng bản đồ trên drone:**
- Pose chỉ dùng mẫu quá khứ (trễ radar 50 ms, trễ MAVLink 20 ms); voxel 0,5 m; lọc bóng ma chạy dần khi dữ liệu tới.
- Tốn 1,7 ms mỗi khung, tức 10 % thời gian thực bằng Python.
- Bản đồ khớp với bản xử lý sau chuyến bay: **IoU 0,99** (`rtk_3`).

**Đường truyền:** token bucket ở 90 % tốc độ, ưu tiên nghiêm ngặt pose → voxel → điểm (điểm SNR cao trước).
Mỗi datagram tính thêm 28 B header UDP/IP.

| Chuyến | Đường truyền | kbit/s thực (median / 95 % / max) | Điểm gửi được | Cột: mặt đất / offline | Dây: mặt đất / offline | Vật giả: mặt đất / offline |
|---|---|---|---|---|---|---|
| `rtk_3` | 300 | 17 / 57 / 74 | 100 % | 4/4 / 4/4 | 4/4 / 4/4 | 1 / 1 |
| `rtk_3` | 50 | 17 / 44 / 46 | 87 % | 4/4 / 4/4 | 3/4 / 4/4 | 1 / 1 |
| `sens_points3` | 300 | 19 / 59 / 84 | 100 % | 4/4 / 4/4 | 4/4 / 4/4 | 0 / 0 |
| `sens_points3` | 50 | 19 / 44 / 45 | 86 % | 4/4 / 4/4 | 3/4 / 4/4 | 0 / 0 |
| `sens_bright` | 300 | 27 / 171 / 203 | 100 % | 4/4 / 4/4 | 3/4 / 3/4 | 12 / 13 |
| `sens_bright` | 50 | 27 / 45 / 45 | 33 % | 4/4 (≤ 0,35 m) / 4/4 | 3/4 / 3/4 | 12 / 13 |
| `sens_worst` | 300 | 6 / 29 / 45 | 100 % | 3/4 / 3/4 | 0/4 / 0/4 | 4 / 4 |

Sai số của cột và dây tìm được trên bản đồ mặt đất bằng bản offline (≤ 0,23 m). Ngoại lệ duy nhất là `sens_bright`
ở 50 kbit/s: cột lệch tới 0,35 m, vì chỉ 33 % số điểm qua được.

**Thời gian từ lúc radar thấy vật (3 khung có điểm trong vòng 1 m) tới lúc vật có trên bản đồ mặt đất** (`rtk_3`):

- Cột, tường, thân cây, tay xà: 0,5–1,8 s. Dây b và dây chống sét: khoảng 5 s.
- Dây a, dây c, nhà: 17–79 s. Lý do không nằm ở đường truyền: các vật này bị radar "chạm" lẻ tẻ từ xa (hoặc bởi
  điểm tán của cột ở gần) từ sớm, nhưng chỉ đủ điều kiện xác nhận ở lượt bay đi gần chúng.
- Riêng phần đường truyền chỉ thêm khoảng 0,1 s (radar 50 ms + đường truyền 50 ms).
- Phần lớn độ trễ nằm ở **điều kiện xác nhận voxel**: phải được nhìn trong 3 khung, từ các vị trí cách nhau ≥ 2 m.
  Khi bay 4 m/s, điều đó tốn ít nhất 0,5 s.

**Kết luận:**

- **300 kbit/s đủ cho bản đồ thời gian thực ở mọi biến thể đã thử.** Đỉnh 203 kbit/s, 100 % điểm gửi được.
- **Ở 50 kbit/s, bản đồ voxel vẫn đầy đủ;** chỉ có điểm thô giảm (33–87 %), làm dây a bị sót và cột kém chính xác hơn.
- **Không cần chuyển hẳn sang chế độ "chỉ thu thập" khi băng thông thiếu.** Bản đồ thời gian thực giảm chất lượng
  dần, còn log đầy đủ vẫn ghi trên drone. Chỉ dưới khoảng 20 kbit/s voxel mới trễ nhiều giây (pose đã chiếm 31 %).

## Kết quả GĐ5, phần 5: chạy live trên SITL (10/11/2026)

`scripts/survey_live.sh [tag] [kbps]` chạy cả chuỗi: sim → `radar_array --frames-udp` (khung radar thô, như radar
gửi tới máy tính trên drone) → `onboard_live.py` (pose PX4 qua MAVLink 14550) → đường truyền 300 kbit/s →
`ground_map.py --udp` → RViz `rviz/ground.rviz`.
- `HEADLESS=1`: không mở Gazebo GUI.
- `VIEW=0`: không mở RViz.

Khác với chạy phát lại:
- MAVLink không có σ heading, nên heading được coi là đã căn sau khi drone bay ≥ 2 m/s liên tục 2 s.
- Đồng hồ của drone (sim time) không trùng đồng hồ mặt đất, nên không đo được độ trễ khi chạy live.

| Chuyến | Hiển thị | Mission | Đường truyền (median / 95 % / max) | Cột | Dây | Vật giả |
|---|---|---|---|---|---|---|
| `live_1` | Gazebo GUI và RViz | **treo tại chỗ** (radar trễ, `avoid_node` giữ) | — | — | — | — |
| `live_h1` | không | PASS (3,16 m) | 19 / 54 / 77 kbit/s, 100 % điểm | 4/4 (≤ 0,14 m) | 4/4 (≤ 0,14 m; dây a 61 %) | 0 |
| `live_2` | Gazebo GUI và RViz | **treo** ("radar cloud 3.69 s old: holding") | — | — | — | — |
| `live_3` | chỉ RViz | PASS (3,15 m) | 17 / 59 / 78, 100 % | 4/4 (≤ 0,13 m) | 3/4 (≤ 0,06 m, sót dây a) | 2 |
| `live_4` | Gazebo GUI và RViz, protobuf C++ | PASS (3,13 m), không bị giữ lần nào, radar trễ median 0,18 s | 18 / 58 / 77, 100 % | 4/4 (≤ 0,15 m) | 4/4 (≤ 0,12 m; dây a 27 %) | 0 |

**Nguyên nhân treo khi có GUI và cách sửa:**

- Code Python sinh sẵn của gz-msgs được viết cho protobuf 3. Với protobuf 6.33 trong user site, nó chỉ chạy được
  bằng backend Python thuần (`radar_model.py` đặt `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python`).
- Mỗi khung LaserScan tốn 4 ms để giải mã, mỗi giây có 60 khung. Kết quả là `radar_array` chạy 100 % một nhân
  ngay cả khi headless, và khi có thêm GUI thì tụt lại phía sau.
- Lấy mẫu stack (`scripts/sampler.py`, bật bằng `PY_SAMPLE`) cho thấy khoảng 2/3 thời gian làm việc nằm ở
  `decoder.py`.
- **Cách sửa:** `run_sitl.sh` chạy `radar_array` bằng protobuf 3.12 của hệ thống với backend C++
  (`python3 -s`, `PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=cpp`): 0,18 ms mỗi khung, nhanh hơn 22 lần.
  `radar_array` giảm từ 100 % xuống 44 % CPU, output không đổi. `RADAR_FAST_PB=0` quay về cách cũ.
- **Ảnh hưởng tới né vật cản:** cùng code, cùng output, chỉ nhanh hơn. Độ trễ radar giảm (median 0,22 → 0,18 s),
  nên cơ chế giữ vì radar trễ ít khi phải can thiệp hơn.

**Kết luận phần 5:** chạy live cho kết quả bằng chạy phát lại (cột 4/4, dây 3–4/4, ≤ 77 kbit/s). Dây a lúc được
lúc không, vì nằm ở rìa vùng radar thấy.

## Kết quả GĐ5, phần A–D: sửa bãi thử, mô hình bề mặt, lớp hiển thị, C++ (10/11/2026)

Sau `live_4`, xem trên RViz thấy ba vấn đề:
- radar không quét tới 2 khối (nhà, tường) và dây thứ hai, vì đường bay chỉ đi một phía đường dây;
- cột điện trên RViz dày, vì điểm của nhiều lượt quét chồng lên nhau;
- một số phần tính toán nên chuyển sang C++.

**A. Bãi thử và đường bay** (đường dây vẫn dọc trục x, như phần né vật cản):
- `worlds/survey_site.sdf`:
  - tường đổi thành 5 × 0,4 × 6 m, đặt tại (40, 7);
  - cột đèn dời tới (35, −10,75), thân cây tới (20, −10,75).
- `survey_site.truth.yaml` sửa theo. Bản cũ: `scratchpad/*.v1`.
- `survey_mission.py`: 41 waypoint, 684 m; vật gần nhất 3,1 m.
  1. Zigzag ở y = ±14 (z = 6 / 12).
  2. Bay trên đường dây ở z = 16,5.
  3. Bay dọc hai bên ở y = 8 và y = −7,5, z = 10.
  4. Bay vòng tròn r = 20 quanh (30, 0).
- Lượt bay vẫn qua `avoid_node`, nên né vật cản luôn bật.
- Các chuyến bay trước ở world cũ không so sánh được với world này: cột đèn đã đổi chỗ, nên bị tính là cột giả.

**B. Mô hình bề mặt của radar** (chỉ áp cho bay khảo sát):
- Đo trên bàn thử: tường 6 × 4 m đặt chính diện biến mất với AOP từ ngoài 8 m, trong khi dây 20 mm vẫn thấy tới
  10 m. Nguyên nhân là mô hình bề mặt giảm theo R⁻⁴, như vật điểm.
- Mặt phẳng lớn thực tế giảm theo R⁻² (gương phẳng). Đổi thành `--surface-law 20 --surface-db 8`: ISK thấy
  tường tới 30 m, AOP tới khoảng 15 m.
- `survey_live.sh` truyền mặc định (biến `SURFACE`).
- Né vật cản giữ nguyên R⁻⁴ và 0 dB: không đặt tuỳ chọn thì không có gì thay đổi. Muốn đổi cho né vật cản thì
  phải kiểm lại phần né.

**C. Lớp hiển thị trên RViz** (`ground_map.py`):
- `/ground/voxels_sharp`: chỉ hiện voxel có trọng số ≥ 0,35 lần trọng số lớn nhất trong vòng 1,5 m (và ≥ 0,5).
  Trọng số mỗi điểm là (5 / max(r, 5))², tức điểm đo gần được tin hơn.
  - Trong bản tin VOXEL, trọng số chiếm 1 byte, nên mỗi bản ghi tăng từ 8 lên 9 B.
  - Bán kính cột (median) giảm từ 1,27 m xuống 0,35 m.
- `/ground/objects`: các vật tìm được (trụ, đoạn dây, khối), cập nhật 3 s một lần trong luồng riêng.
- `rviz/ground.rviz` bật sẵn lớp sharp, tắt lớp voxel đầy đủ.

**D. C++** (`cpp/mapper_core.cpp`, pybind11; build bằng `cpp/build.sh`):
- `VoxelMap`, `place_frame` (đưa khung radar về hệ thế giới, kèm cổng Doppler) và `PoseTrack` (pose nhân quả,
  nội suy và ngoại suy).
- Bộ lập map trên drone: 1,69 → 0,03 ms mỗi khung, nhanh hơn 55 lần, cho cùng một map (IoU 0,99).
- `PoseTrack`: 1,6 µs mỗi lần tra, so với 1,6 ms.
- Nếu không có module C++ thì tự quay về Python; `ONBOARD_PY=1` ép dùng Python.
- Tìm vật (`survey_objects.find_objects`): 8,2 → 0,5 s, nhờ gom cột theo ô lưới thay vì theo điểm.
  - Ground map dùng lại hàm này, nên chạy được mỗi 3 s.
- Các test so sánh C++ với Python nằm ở `tests/test_mapper_core.py`.

**Sửa bộ tìm vật sau `live_5`:**
- Cột đèn và thân cây chỉ được bay qua theo trục x, nên sai số góc của radar kéo dãn đám điểm theo x
  (sd 0,9 × 0,3 m, tỉ lệ 3). Vì vượt ngưỡng 2,5, chúng bị coi là tường.
  - Nay chỉ coi là tường khi vừa dãn vừa có sd dài > 1,2 m (`POLE_MAX_SPREAD`); tường 5 m có sd 1,4 m.
- Các đám điểm nhỏ (15–56 điểm) cạnh dây đã tìm thấy là phần tản của chính dây đó. Trong vòng 1 m quanh dây
  (`WIRE_ABSORB`), chúng được gộp vào dây.
- Ở `live_6`, vài ô cao trên mép tường và mép mái (73–93 điểm, 2–4 ô) vượt ngưỡng 60 điểm nên bị coi là cột.
  Cột giả này lại lấy hết điểm trong vòng 3 m, nên tường biến mất.
  - Cột thật có hàng nghìn điểm (cột đèn 3700). Ngưỡng nâng lên 300 (`POLE_MIN_POINTS`).
- Ba test mới; tổng 99 test, tất cả đều qua. Cả ba test đều hỏng nếu dùng quy tắc cũ.

| Chuyến | Hiển thị | Mission | Đường truyền (median / 95 % / max) | Cột | Dây | Khối | Vật giả |
|---|---|---|---|---|---|---|---|
| `live_5` | headless | PASS (2,92 m) | 39 / 115 / 148 kbit/s, 100 % điểm | 4/4 (≤ 0,12 m) | 4/4 (≤ 0,06 m; dây a 94 %) | nhà, tường | 0 |
| `live_6` | Gazebo GUI và RViz | PASS (3,07 m), không bị giữ lần nào | 38 / 110 / 150, 100 % | 4/4 (≤ 0,14 m) | 4/4 (≤ 0,06 m; dây a 94 %) | nhà, tường | 0 |

Hai chuyến được chấm lại sau khi sửa bộ tìm vật (`survey_objects.py out/survey/<tag>`).

Trong lúc bay `live_6`, ground map vẫn dùng bộ tìm vật cũ. Nó tìm thêm 1 cột giả và sót tường.

Kết quả `live_5`:
- **Cột:** ban đầu tìm được 2/4, sau khi sửa 4/4.
- **Vật giả:** ban đầu 9, sau khi sửa 0.
- **Đường truyền:** tăng lên 148 kbit/s do mặt phẳng sáng hơn, vẫn dưới 300.
- **Nhà và tường:** tìm được, nhưng tâm cao hơn thực khoảng 1,7 m.
  - Drone bay phía trên ở z = 10, nên chỉ thấy mái và đỉnh tường; độ tản theo góc ngẩng của radar đẩy điểm
    lên cao.
  - Muốn thấy phần thấp thì phải có lượt bay thấp ngang qua.
- **Xà ngang:** thanh 15 cm nằm trong đám điểm của cột, nên không tách riêng được.

## Kết quả GĐ6 (10/11/2026)

Kết quả tổng hợp nằm ở [survey_mapping_results.md](survey_mapping_results.md). README có thêm mục
"Lập map bằng radar + RTK".

**Ma trận thử đã thu gọn so với kế hoạch:**
- 1 world, 1 đường bay. World cũ đã bỏ; các đường bay cũ được gộp thành đường 41 waypoint.
- 3 chuyến bay: `live_5`, `live_6` (RTK) và `std_5` (GPS thường).
- Trên mỗi chuyến, 6 biến thể dựng lại sau chuyến bay (`scripts/survey_matrix.py`):
  - pose thật hoặc ước lượng;
  - lệch đồng hồ 50 / 100 ms;
  - lệch lắp 2 / 5°.
- Đường truyền: phát lại ở 300 / 100 / 50 kbit/s.

**Test:**
- `tests/test_survey_process.py`: drone nghiêng và xoay, điểm vẫn rơi đúng vào vật. Bỏ qua roll và pitch thì điểm
  lệch hơn 2 m.
- Đường truyền đã có test trong `tests/test_radar_link.py`.
- Multilateration chưa có test, vì GĐ4 hoãn.
- Tổng 101 test, tất cả đều qua.

**Kết quả:**
- Với RTK: cột ≤ 0,14 m, dây ≤ 0,06 m, 0 vật giả.
- Với GPS thường: dây lệch 0,65 m, 14 vật giả.
- Dữ liệu radar thô (theo định dạng TI) có đỉnh 369 kbit/s, vượt 300. Map gửi về dùng tối đa 150 kbit/s.

**Sửa thêm:**
- `survey_live.sh` nhận biến `GPS=`.
- `survey_objects.py` xoá các file `.ply` cũ trong `<tag>.objects/` trước khi ghi.

## Lượt bay thấp bên nhà và tường (12/11/2026)

- **Đường bay:** `survey_mission.py` có thêm `--low-pass-alt` (mặc định 3 m). Đoạn phía bắc từ đông về tây bay ở
  y = 12,5, cách nhà 3,5 m và tường 5,3 m. Đường bay có 43 waypoint, 691 m; `--low-pass-alt 0` cho lại đường cũ.
- **Chuyến `low_1`** (headless): mission PASS, vật gần nhất 3,03 m, không bị giữ lần nào; đường truyền max 161 kbit/s.
- **Kết quả:**
  - Nhà: tâm lệch 1,0 m (trước 1,3–1,8 m), cao 2,9 m (trước 1,2–2,3 m).
  - Tường: tâm lệch 0,6 m (trước 1,5–1,7 m), cao 4,5 m (trước 2,7–3,0 m).
  - Cột và dây không đổi; 0 vật giả.
- **Sửa bộ tìm khối:** tường dài và mỏng (tỉ lệ 8–10) chạm gần mặt đất (≤ 3 m) được tính là khối bất kể tỉ lệ
  (`BLOCK_GROUND_REACH`). Trước đó, ở `low_1` tường bị cắt thành 3 dây giả; các chuyến cũ không đổi. Thêm 1 test,
  tổng 107 test.
- **Sửa dừng tiến trình:**
  - `survey_live.sh` chỉ dừng sim của chính nó, và chờ nếu đang có một chuyến khác.
  - `run_sitl.sh` dọn theo danh sách PID riêng.
  - `stop_sim.sh` bắt được cả `radar_array` chạy bằng `python3 -s`.
  - `low_1` là chuyến đầu chạy với bản sửa này: không sót tiến trình nào.
