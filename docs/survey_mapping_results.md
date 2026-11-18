# Lập map bằng radar + RTK: kết quả (ý 1, chỉ radar, SITL)

Kết quả tổng hợp sau GĐ0–3, GĐ5 và GĐ6 (10/11/2026). Kế hoạch và chi tiết từng giai đoạn nằm ở
[survey_mapping_plan.md](survey_mapping_plan.md); cách chạy ở [README](../README.md#lập-map-bằng-radar--rtk).
Mọi số liệu trong tài liệu này lấy từ sim (Gazebo + mô hình radar `scripts/radar_model.py`), chưa có radar thật.

## Bài toán và trả lời ngắn

| Câu hỏi | Trả lời trong sim |
|---|---|
| Lập map thời gian thực trong 300 kbit/s được không? | **Được.** Lý do: không gửi dữ liệu radar thô về mặt đất mà gửi các voxel của map trên drone, cộng với điểm radar khi còn chỗ.<br>• Ở 300 kbit/s: median 37, max 150 kbit/s; mọi điểm đều gửi được.<br>• Xuống 50 kbit/s vẫn đủ map; chỉ bớt điểm (55 % điểm). |
| Dữ liệu thô có vừa 300 kbit/s không? | **Không, ở lúc đỉnh.**<br>• Theo định dạng TI, 6 radar gửi trung bình 124 kbit/s, đỉnh 369 kbit/s trong 1 s.<br>• Khi nén lại: 43 kbit/s.<br>• Vì vậy dữ liệu thô đầy đủ được ghi trên drone để xử lý sau chuyến bay (`RADAR_LOG=1`). |
| Drone nghiêng thì toạ độ vật có đúng không? | **Đúng.** Mỗi điểm được đặt bằng tư thế đầy đủ của drone (roll, pitch, yaw), cộng vị trí và hướng gắn của từng radar, nội suy tại thời điểm quét.<br>• Có test kiểm điều này (`tests/test_survey_process.py`).<br>• Nếu bỏ qua roll và pitch, điểm lệch hơn 2 m. |
| Vị trí tương đối và tuyệt đối của vật | Có cả hai. Map nằm ở toạ độ tuyệt đối: gốc EKF lấy từ RTK, so với toạ độ địa lý của world.<br>• Mỗi vật có vị trí, kích thước và point cloud riêng (`<tag>.objects.yaml`, `<tag>.objects/*.ply`).<br>• Vị trí tương đối giữa các vật suy ra trực tiếp từ đó. |<br>• Kết quả ghi theo hệ local NED của PX4 (gốc EKF, tức chỗ cất cánh; x bắc, y đông, z xuống), kèm gốc dạng lat/lon/alt; `--frame world` cho hệ world ENU.
| RTK có cần không? | **Cần.**<br>• Với GPS thường: dây lệch 0,65 m và có 11–23 vật giả.<br>• Với RTK: dây lệch ≤ 0,06 m và 0–5 vật giả. |

## Bãi thử, đường bay, radar

- **World `worlds/survey_site.sdf`:**
  - đường dây dọc trục x, dài 30 m, gồm 3 dây 20 mm ở z = 10 (y = −3, 0, 3) và 1 dây đất 10 mm ở z = 12;
  - 2 cột điện cao 13 m;
  - cột đèn cao 8 m, thân cây cao 10 m;
  - nhà 6 × 4 × 4 m, tường 5 × 0,4 × 6 m.
  - Đáp án nằm ở `survey_site.truth.yaml`.
- **Đường bay** (`scripts/survey_mission.py`): 43 waypoint, 691 m, khoảng 4 phút, luôn đi qua phần né vật cản
  (`avoid_node`). Gồm các lượt:
  1. zigzag hai bên; đoạn phía bắc từ đông về tây bay thấp (y = 12,5, cao 3 m) ngang tầm nhà và tường (từ 12/11;
     các chuyến `live_5`, `live_6`, `std_5` bay đường cũ, không có lượt này);
  2. bay trên đường dây;
  3. bay dọc hai bên đường dây;
  4. bay vòng tròn r = 20 m quanh khu đường dây.
- **Radar:** 1 IWR6843ISK nhìn trước + 5 IWR6843AOP, mỗi radar 10 Hz.
- **Pose:** EKF2 với `GPS=rtk` (nhiễu vị trí 2 cm) hoặc `GPS=std` (1,5 m).
- **Mô hình bề mặt cho bay khảo sát:** mặt phẳng lớn giảm theo R⁻² và sáng hơn 8 dB.
  - Lý do: với mặc định R⁻⁴, một bức tường đặt chính diện biến mất khỏi AOP từ ngoài 8 m.
  - Né vật cản vẫn dùng mặc định.

## Cách xử lý

**Trên drone** (`scripts/onboard_live.py` + `cpp/mapper_core.cpp`, 0,03 ms mỗi khung radar):
1. Lấy pose của PX4 qua MAVLink. Chỉ dùng mẫu đã tới, ngoại suy theo vận tốc.
2. Đặt các điểm vào hệ thế giới.
3. Lọc Doppler: bỏ điểm không khớp với vận tốc của chính drone.
4. Lập map voxel 0,5 m. Voxel chỉ được xác nhận khi thấy trong ≥ 3 khung, từ các vị trí cách nhau ≥ 2 m (lọc
   bóng ma).
5. Gửi về mặt đất (`scripts/radar_link.py`, token bucket ở 90 % tốc độ đường truyền), theo thứ tự ưu tiên:
   - pose, 10 Hz;
   - voxel đã xác nhận, 9 B mỗi voxel, không bao giờ bỏ;
   - điểm radar, mạnh nhất trước, khi còn chỗ.

**Dưới mặt đất** (`scripts/ground_map.py`, hiển thị trên RViz):
- dựng map từ các voxel nhận được;
- hiện lớp "sharp": các voxel có điểm đo ở gần;
- tìm vật mỗi 3 s.

**Sau chuyến bay** (`scripts/survey_process.py`, `scripts/survey_objects.py`):
- Đặt lại các điểm từ bản ghi radar và ulog. Bỏ các khung có σ heading của EKF > 1,5°.
- Bỏ mặt đất (RANSAC) và bóng ma.
- Tìm vật theo thứ tự:
  1. cột: ô lưới cao ≥ 3 m, ≥ 300 điểm;
  2. khối: nhà, tường;
  3. dây: RANSAC đường thẳng, kèm kiểm tra không có bề mặt ở cạnh;
  4. phần còn lại.

## Kết quả

### Ma trận thử (`scripts/survey_matrix.py`, vật tìm được sau chuyến bay, sai số lớn nhất theo m)

Ghi chú:
- Bỏ xà ngang: thanh 15 cm, nằm trong đám điểm của cột.
- Sai số của khối là khoảng cách từ tâm tìm được tới tâm thật.

| Chuyến | Pose / lỗi thêm vào | Cột | Dây | Nhà, tường | Vật giả |
|---|---|---|---|---|---|
| `live_5` (RTK, headless) | pose thật | 4/4 (≤ 0,03) | 4/4 (≤ 0,03) | 2/2 (≤ 1,75) | 2 |
| | **pose ước lượng** | **4/4 (≤ 0,12)** | **4/4 (≤ 0,06)** | 2/2 (≤ 1,78) | **0** |
| | + lệch đồng hồ 50 ms | 4/4 (≤ 0,13) | 4/4 (≤ 0,05) | 2/2 (≤ 1,85) | 1 |
| | + lệch đồng hồ 100 ms | 4/4 (≤ 0,36) | 4/4 (≤ 0,06) | 2/2 (≤ 1,88) | 1 |
| | + lệch lắp 2° | 4/4 (≤ 0,11) | 4/4 (≤ 0,12) | 2/2 (≤ 1,76) | 0 |
| | + lệch lắp 5° | 4/4 (≤ 0,10) | 4/4 (≤ 0,24) | 2/2 (≤ 1,92) | 3 |
| `live_6` (RTK, Gazebo GUI) | pose thật | 4/4 (≤ 0,04) | 4/4 (≤ 0,05) | 2/2 (≤ 1,57) | 0 |
| | **pose ước lượng** | **4/4 (≤ 0,14)** | **4/4 (≤ 0,06)** | 2/2 (≤ 1,45) | **0** |
| | + lệch đồng hồ 50 ms | 4/4 (≤ 0,13) | 4/4 (≤ 0,06) | 2/2 (≤ 1,64) | 0 |
| | + lệch đồng hồ 100 ms | 4/4 (≤ 0,33) | 4/4 (≤ 0,06) | 2/2 (≤ 1,72) | 0 |
| | + lệch lắp 2° | 4/4 (≤ 0,11) | 4/4 (≤ 0,08) | 2/2 (≤ 1,41) | 2 |
| | + lệch lắp 5° | 4/4 (≤ 0,12) | 4/4 (≤ 0,20) | 2/2 (≤ 1,22) | 5 |
| `low_1` (RTK, headless, có lượt bay thấp) | pose thật | 4/4 (≤ 0,05) | 4/4 (≤ 0,04) | 2/2 (≤ 0,92) | 0 |
| | **pose ước lượng** | **4/4 (≤ 0,15)** | **4/4 (≤ 0,05)** | **2/2 (≤ 1,00)** | **0** |
| | + lệch đồng hồ 50 ms | 4/4 (≤ 0,14) | 4/4 (≤ 0,05) | 2/2 (≤ 1,04) | 0 |
| | + lệch đồng hồ 100 ms | 4/4 (≤ 0,36) | 4/4 (≤ 0,05) | 2/2 (≤ 1,16) | 0 |
| | + lệch lắp 2° | 4/4 (≤ 0,15) | 4/4 (≤ 0,05) | 2/2 (≤ 1,69) | 3 |
| | + lệch lắp 5° | 4/4 (≤ 0,10) | 4/4 (≤ 0,19) | 2/2 (≤ 0,87) | 5 |
| `std_5` (GPS thường, headless) | pose thật | 4/4 (≤ 0,03) | 4/4 (≤ 0,04) | 2/2 (≤ 1,63) | 1 |
| | **pose ước lượng** | **4/4 (≤ 0,16)** | **4/4 (≤ 0,65)** | 2/2 (≤ 2,09) | **14** |
| | + lệch đồng hồ 50 ms | 4/4 (≤ 0,17) | 4/4 (≤ 0,69) | 2/2 (≤ 2,07) | 23 |
| | + lệch đồng hồ 100 ms | 4/4 (≤ 0,36) | 4/4 (≤ 0,73) | 2/2 (≤ 2,26) | 17 |
| | + lệch lắp 2° | 4/4 (≤ 0,14) | 4/4 (≤ 0,73) | 2/2 (≤ 2,05) | 11 |
| | + lệch lắp 5° | 4/4 (≤ 0,16) | 4/4 (≤ 0,66) | 2/2 (≤ 2,26) | 14 |

**Độ chính xác pose** (`scripts/pose_check.py`, EKF so với thật, median / 95 %):

| GPS | Ngang (m) | Đứng (m) | Heading (°) |
|---|---|---|---|
| RTK | 0,09–0,12 / 0,20–0,24 | 0,03 / 0,08 | 0,4–0,6 / 2,2–3,2 |
| thường | 1,12 / 1,59 | 0,33 / 0,96 | 1,1 / 5,0 |

Đọc bảng:
- **Với RTK, sai số còn lại là của radar, không phải của pose.**
  - Cột và dây đặt bằng pose ước lượng chỉ kém pose thật vài cm.
  - Sai số góc của radar làm đám điểm tản 1–2 m, nhưng tâm vật vẫn tìm đúng.
- **Mức chịu được:**
  - Lệch đồng hồ ≤ 50 ms và lệch lắp ≤ 2° gần như không đổi kết quả.
  - Lệch đồng hồ 100 ms làm cột lệch 0,35 m.
  - Lệch lắp 5° làm dây lệch 0,2 m và sinh thêm 3–5 vật giả.
  - Lệch đồng hồ đo lại được từ chính dữ liệu, trong khoảng ±20 ms (`survey_process.py --find-clock`).
- **GPS thường:**
  - Cột vẫn tìm được, vì nhiều điểm và cao.
  - Dây lệch 0,65 m, chủ yếu theo chiều đứng.
  - Map bị nhoè, nên có 11–23 vật giả.

### Thời gian thực (`scripts/realtime_eval.py`, phát lại `live_6` qua đường truyền)

| Đường truyền (kbit/s) | Dùng mỗi giây (median / 95 % / max) | Điểm radar gửi được | Cột | Dây | Vật giả |
|---|---|---|---|---|---|
| 300 | 37 / 110 / 144 | 100 % | 4/4 (≤ 0,15) | 4/4 (≤ 0,04) | 0 |
| 100 | 37 / 89 / 90 | 92 % | 4/4 (≤ 0,14) | 4/4 (≤ 0,05) | 0 |
| 50 | 36 / 45 / 45 | 55 % | 4/4 (≤ 0,14) | 4/4 (≤ 0,09) | 0 |

**Độ trễ: thời gian từ lúc radar thấy vật (trong 3 khung) tới lúc vật có trên map dưới mặt đất:**
- **Cột, nhà, tường:** 0,6–1,5 s. Ở 50 kbit/s vẫn vậy, nên không phải do đường truyền.
- **Dây:** 1,8–5 s với 2 dây; 2 dây còn lại 29 s và 60 s. Dây trả về thưa và lần đầu thấy thường ở rìa vùng nhìn, nên phải chờ
  một lượt bay thứ hai thấy lại.

**Chạy live trên SITL** (`scripts/survey_live.sh`):
- `live_5` (headless) và `live_6` (Gazebo GUI + RViz): mission PASS, vật gần nhất 2,9–3,1 m, không bị giữ vì radar
  trễ.
- Đường truyền dùng max 150 kbit/s.
- Kết quả giống phát lại.

**Tốc độ xử lý:**

| Bước | Trước | Sau | Cách làm |
|---|---|---|---|
| Lập map trên drone, mỗi khung | 1,69 ms | 0,03 ms | C++ |
| Giải mã khung radar trong sim | — | nhanh hơn 22 lần | protobuf C++ |
| Tìm vật | 8,2 s | 0,5 s | gom cột theo ô lưới |

## Giới hạn và việc còn lại

- **Chưa có radar thật.**
  - Mô hình bề mặt (R⁻², +8 dB), sai số góc và bóng ma đa đường đều là ước lượng.
  - Kết quả kiểm tra độ nhạy (trên bãi cũ, [survey_mapping_plan.md](survey_mapping_plan.md#kết-quả-kiểm-tra-độ-nhạy-10112026)):
    - sai số góc ×2: còn 4/4 cột, nhưng chỉ 3/4 dây;
    - trường hợp tệ nhất (góc ×2, yếu hơn 5 dB, 5 % bóng ma): mất cả 4 dây.
  - Dây là thứ cần kiểm với radar thật trước tiên.
- **Nhà và tường:** với đường bay cũ, drone chỉ bay phía trên ở z = 10, nên chỉ thấy mái và đỉnh tường; tâm tìm ra
  cao hơn thực 1,1–1,9 m. Lượt bay thấp (`low_1`, 12/11) cải thiện:

  | | Đường bay cũ (`live_5`, `live_6`) | Có lượt bay thấp (`low_1`) | Thật |
  |---|---|---|---|
  | Nhà: tâm lệch | 1,3–1,8 m (chủ yếu theo chiều cao) | 1,0 m (cao hơn 0,7 m) | — |
  | Nhà: kích thước tìm ra | 5,4–5,6 × 3,2–3,5 × 1,2–2,3 m | 5,9 × 3,5 × 2,9 m | 6 × 4 × 4 m |
  | Tường: tâm lệch | 1,5–1,7 m | 0,6 m | — |
  | Tường: kích thước tìm ra | 4,3 × 0,4 × 2,7–3,0 m | 4,8 × 0,6 × 4,5 m | 5 × 0,4 × 6 m |

  - Phần sát đất và mặt phía nam vẫn thiếu, vì lượt bay thấp chỉ đi phía bắc.
  - Tường dài và mỏng (tỉ lệ 8–10) từng bị loại khỏi "khối" theo luật tỉ lệ dài/rộng, vốn để loại hai dây chồng
    nhau. Nay tường chạm tới gần mặt đất (≤ 3 m) được tính là khối bất kể tỉ lệ. Hai dây ở 10–12 m không bao giờ
    thấp như vậy.
- **Point cloud từng vật:** sai số góc làm đám điểm tản rộng. Point cloud gọn (`<vật>.sharp.ply`,
  `survey_objects.sharpen`) bỏ các điểm đo từ xa:

  | Vật | Mọi điểm (median cách trục) | Gọn | Bán kính thật |
  |---|---|---|---|
  | cột điện | 0,63–0,66 m | 0,32–0,34 m | 0,2 m |
  | cột đèn, thân cây | 0,50–0,54 m | 0,19–0,20 m | 0,10–0,15 m |
  | dây | 0,19–0,23 m | 0,19–0,23 m (gần như không đổi) | 0,01 m |

  - Số liệu từ `live_5` và `live_6`.
  - Không áp cho khối: điểm cho chiều cao của nhà nằm ở xa, nên lọc thì chiều cao nhà giảm từ 2,3 xuống 1,3 m.
- **Xà ngang 15 cm:** không tách được khỏi cột.
- **Dây:** tìm được 76–95 % chiều dài.
- **GĐ4 (multilateration) hoãn tới khi có dữ liệu radar thật:** sim không mô phỏng điểm phản xạ thật, nên kết quả
  sẽ đẹp hơn thực tế.
- **Phạm vi thử:** 1 world và 1 đường bay; mỗi cấu hình 1–2 chuyến (RTK 2 chuyến, GPS thường 1 chuyến). Các chuyến
  trước khi đổi bãi (trước 10/11) chấm theo bãi cũ, không so được với bảng này.

**Khi có IWR6843ISK / AOPEVM:**
1. Ghi một chuyến bay qua cột và dây thật bằng cấu hình `profiles/*.yaml`.
2. Xử lý bằng `survey_process.py` và `survey_objects.py`. Bản ghi `radar_log` cùng định dạng với dữ liệu radar
   gửi ra.
3. So sánh SNR, sai số góc và độ phủ bề mặt với sim, rồi chỉnh mô hình.
4. Sau đó làm GĐ4.
