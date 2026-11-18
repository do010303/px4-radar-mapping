# Vòng 3 radar ISK và node tránh vật cản: kết quả mô phỏng

*2026-10-27, cập nhật 2026-11-03. Chạy lại: `scripts/make_isk_x3.sh` (một lần), rồi `scripts/avoid_trials.sh 5 power_line 5` và
`scripts/avoid_trials.sh 3 power_line_thin 2` (headless, khởi động lại sim trước mỗi lượt; log và CSV từng lượt trong
`out/trials/`).*

Cấu hình: 3 × IWR6843ISK (yaw 0/120/-120°), bay ở 10 m, tham số mặc định của `avoid_node.py`
(r_a 6 m, r_s 1,5 m, a_max 3 m/s², margin 3 m, v_eb 0,3 m/s; bài báo không in giá trị nào, đây là giá trị của ví dụ; mã nguồn của tác giả dùng giá trị khác,
xem [paper_malle2026_notes.md](paper_malle2026_notes.md)).
Một lượt thành công khi UAV không bao giờ đến gần dây hay vật cản dưới 1 m (khoảng cách thật từ Gazebo).

## Tỉ lệ thành công

### Né theo tiếp tuyến của bài báo (`--mode tangent`, 2026-10-29, 3×3 radar `RADARS=6`)

PASS nghĩa là cả hai điều kiện: bay hết mission, và không lúc nào cách dây hay vật cản dưới 1 m.

| Lượt | Đường bay | Tốc độ | Kết quả | Gần nhất | Phanh |
|---|---|---|---|---|---|
| #101 (chưa có `--corridor`) | (0,0,10) → (45,0,10), dây ngang tầm | 5 m/s | tới đích | thân cây **1,18 m**: bị kéo về phía cây | 0 |
| #102–103 | như trên | 5 m/s | **2/2** | thân cây 2,38–2,45 m; dây 2,74–2,81 m (leo qua, cao nhất 16,6 m) | 0 |
| #111–112 | waypoints 10,0,8 → 45,0,8 → 45,10,8 | 5 m/s | **2/2** | thân cây 2,48–2,55 m; dây 3,16–3,23 m (chui dưới, thấp nhất 6,2 m) | 0 |

Lệnh: `RADARS=6 scripts/avoid_trials.sh 2 power_line 5` (thêm
`AVOID_ARGS="--waypoints 10,0,8 45,0,8 45,10,8"` cho mission). Chưa thử dây 1,2 mm trong SITL. Thử động
học: [paper_malle2026_notes.md](paper_malle2026_notes.md).

### 1 ISK + 5 AOP (`RADARS=aop`, `--mode tangent`, 2026-10-29)

Bố trí `x500_radar_aop` (README, "Bố trí radar"): 91% mọi hướng, ISK trước quạt đứng, AOP nhìn thẳng xuống và lên.
AOP hiệu chỉnh `snr_ref` 14: dây 20 mm thấy chắc tới 8 m. Headless, `scripts/avoid_trials.sh` với `RADARS=aop`.

| Bài | Lượt | Kết quả | Gần nhất |
|---|---|---|---|
| tangent: (0,0,10) → (45,0,10), 5 m/s, dây ngang tầm | #301, #302 | **2/2**, tới đích | dây đất 2,08 m; thân cây 2,07 m |
| tangent: mission 10,0,8 → 45,0,8 → 45,10,8, 5 m/s | #303 | **1/1**, hết mission | wire_a 1,70 m (chui dưới) |
| high speed: 10 m/s | #311, #312 | **2/2**, tới đích | thân cây 2,12 / 2,09 m |
| close proximity: đích trên wire_a, 1,5 m/s, đủ luật, `--proximity-test` | #321, #322 | **2/2** | wire_a 1,26 / 1,38 m |
| close proximity: như trên, chỉ hình cầu an toàn (`--tangent-gain 0`) | #323 | FAIL sát nút | wire_a **0,96 m** |

Hai bài từng FAIL trên `x500_radar6` giờ qua:
- 10 m/s: trước đây là #122, đâm dây vì chúi mũi làm dây ra khỏi FOV. Giờ ISK trước có quạt đứng, cộng `--cmd-accel 3`.
- Close proximity với đủ luật: trước đây 0,73 m vì dây ngay dưới bụng ở mép quạt. Giờ AOP dưới nhìn thẳng xuống.

#323 không phải lỗi nhận biết: đám điểm gần nhất luôn khớp khoảng cách thật (0,98 m so với 0,96 m). Đó là giới hạn của
riêng luật Scenario 3.
- Diễn biến: drone tới thẳng ở 1,5 m/s; lệnh đẩy ra bắt đầu ở 1,66 m và bão hoà −2 m/s ở 1,36 m. Drone vẫn trôi
  thêm 0,4 m (trễ của vòng vận tốc PX4 cộng quãng hãm), dưới 1 m trong 0,3 s, rồi bật ra.
- Luật này là phản xạ cuối, không có gì làm drone chậm lại trước khi chạm vỏ cầu. Ở bài đủ luật, tiếp tuyến làm việc đó.
- Mã tác giả thêm "caution sphere" 2,5 m để hãm dần thành phần vận tốc hướng vào vật. Một CBF làm được việc đó một cách
  trơn tru. Cách khác: tăng r_s.

Lượt đầu (#201) hỏng vì AOP thấy cánh quạt của chính drone. Đã sửa bằng cách hạ ba AOP xuống, xem README.

### Chuỗi vật cản, né bốn hướng (`RADARS=aop`, `--mode tangent`, 2026-11-03)

World `power_line_course`: đoạn dây điện, rồi tường thấp, ống ngang trên cao, cổng hai cột lệch 2,2 m, khối bên phải,
dây thấp. Mission 7 waypoint ở 10 m, 5 m/s (`--waypoints 20,0,10 39,0,10 50,0,10 60,0,10 71,0,10 82,0,10 98,0,10`).
Luật né: `--way-round shortest` (độ dịch nhỏ nhất của đường bay, bốn hướng), xem
[paper_malle2026_notes.md](paper_malle2026_notes.md).

| Lượt | Bản | Kết quả | Gần nhất |
|---|---|---|---|
| #601 | độ dịch × tổng điểm | **FAIL**: timeout, lắc ngang 120 s trước waypoint (60, 0, 10) | 1,04 m `wire_beam` |
| #602 | như trên | PASS | 1,20 m `pole_left` |
| #603, #604 | + horizon (không tính điểm sau waypoint) | 2/2 PASS | 1,35 / 1,09 m `pole_left`: văng ngang tới y 5,6 m |
| #701, #703 | + tiếp tuyến tỉ lệ, clearance 2 m | 2/2 PASS | 1,59 m `pole_left`, 1,47 m `pole_right` |
| #702 | như trên | **FAIL** | 0,78 m `pole_left`: vài điểm sát giữa cổng, đổi sang vòng ngoài cột |
| #704–706 | + r_s thành chi phí mềm, tốc độ ngang ≤ 2,5 m/s (bản cuối) | **3/3 PASS**, 38–39 s, không phanh | 2,01 / 1,92 / 1,83 m `pole_right`; qua giữa cổng |

Bản cuối, các bài khác:

| Bài | Lượt | Kết quả | Gần nhất |
|---|---|---|---|
| tốc độ cao: (0,0,10) → (45,0,10), 10 m/s | #721, #722 | **2/2** | 2,26 m `wire_b`, 2,21 m `wire_c` |
| close proximity, đủ luật | #731, #732 | **2/2** | 1,98 / 2,02 m `wire_a` |

Bản trước bản cuối (độ dịch × tổng điểm, có horizon từ #611): 5 m/s 2/2 (2,35 / 2,31 m), 10 m/s 2/2 (1,98 / 2,14 m),
close proximity đủ luật 2,00 m, chỉ hình cầu an toàn (`--tangent-gain 0`) 1,55 m.

Ở bản cuối mọi vật cản của chuỗi đều cách ≥ 1,8 m. Drone đi dưới dây điện và `wire_beam`, trên `wall_low` và
`wire_low`, qua giữa cổng, vòng trái `block_right`. Từ #705, CSV có thêm `shift_x/y/z` (độ dịch đang chọn, m) và
`line_gap` (đường bay hiện tại cách điểm gần nhất bao nhiêu). Nguyên nhân của các điểm sát giữa cổng ở #702 chưa rõ.
Nhiễu góc σ 3° không giải thích được: có thể là tư thế bị trễ khi drone nghiêng mạnh.

### Đường ống radar: lọc Doppler và đặt điểm đúng thời điểm (`RADARS=aop`, 2026-11-03)

Hai thay đổi trong `radar_model.py`, `radar_stream.py` và `radar_array.py` (README, "Mô hình radar"):

1. **Lọc Doppler.** Vận tốc hướng tâm tính theo hướng thật, cộng nhiễu σ 0,05 m/s. Điểm nào có Doppler không khớp
   với một điểm tĩnh tại chỗ nó được đặt thì bị bỏ (`--doppler-gate 3`).
2. **Đúng thời điểm.**
   - Mỗi khung quét được đặt vào world bằng tư thế nội suy đúng lúc quét (`PoseFeed.at`).
   - Khi gộp, điểm của từng radar giữ vị trí world của khung riêng nó.

Thước đo là `scripts/cloud_check.py`: điểm radar gần nhất trừ vật cản thật gần nhất, xét các lúc vật cản thật
trong vòng 6 m. Headless, máy rảnh, chạy xen kẽ:

| Nhóm | Lượt | Kết quả | Đám điểm − thật: trung vị / 5 % | Điểm bị lọc Doppler |
|---|---|---|---|---|
| A. bản cũ, chuỗi vật cản | #931, #933, #935 | 3/3 PASS, 1,93–1,97 m | −0,06 … −0,08 / −0,37 … −0,42 m | không có |
| A. bản mới | #932, #934, #936 | 3/3 PASS, 1,90–1,98 m | **−0,02 … −0,04 / −0,17 … −0,18 m** | 126–138 |
| B. bản mới, 5 % điểm sai góc, bật lọc | #941, #942 | 2/2 PASS | −0,03 / −0,20, −0,04 / −0,17 m | 529–639 |
| B. như trên, tắt lọc (`--doppler-gate 0`) | #943, #944 | 2/2 PASS | −0,04 / **−0,42**, −0,04 / −0,26 m | 0 |
| C. bản mới, 10 m/s | #951 | PASS, 2,21 m | −0,03 / −0,16 m | 108 |
| C. bản mới, close proximity | #961 | PASS, 2,06 m | −0,02 / −0,06 m | 149 |

**Đúng thời điểm (thay đổi 2):** đuôi "điểm gần hơn thật" giảm từ khoảng 0,4 m xuống khoảng 0,17 m, ổn định
qua ba cặp. Kết quả bay không đổi: biên né 2 m đã đủ rộng.

**Lọc Doppler (thay đổi 1):**
- Có 5 % điểm sai góc thì lọc đưa đám điểm về mức không nhiễu.
- Không có điểm sai góc thì lọc bỏ khoảng 130 điểm mỗi chuyến, trên hàng trăm nghìn: không làm mất dữ liệu.
- Lọc chỉ bắt được phần sai góc lớn (khoảng ≥ 12–15°) nằm trong mặt phẳng chứa hướng bay.
- Điểm sai góc ở đây là mô hình (`--angle-outlier`), chưa đối chiếu với radar thật.

**Điều kiện test.** Các lượt #801, #841, #901 và #911–926 không đo được gì:
- #901: hai `avoid_node` cùng lái một sim. Giờ `avoid_node.py` từ chối khởi động khi đã có một bản khác chạy.
- Các lượt còn lại chạy khi máy có load 50–60 (hai job của dự án khác chạy sót). Ống radar không theo kịp, đám
  điểm cũ vài phần mười giây. Bản cũ và bản mới hỏng như nhau: trung vị −1,1 … −1,5 m, bỏ sót 15–26 %, đâm dây.

Từ đó, mỗi lượt tự kiểm tra:
- `avoid_trials.sh` ghi độ khớp đám điểm và load vào `summary.txt`.
- `avoid_node.py` đo tuổi của đám điểm theo đồng hồ sim so với đồng hồ PX4 (`/radar/pose`). Khi máy rảnh tuổi
  này là 0,16–0,30 s (chu kỳ quét, xử lý, truyền ROS); mốc là trung vị của giây đầu tiên.
  - Đám điểm cũ hơn mốc quá `--max-lag` 0,3 s thì drone dừng giữ chờ.
  - 95 % độ trễ vượt mốc quá 0,2 s, hoặc mốc quá 0,5 s, thì lượt bị ghi "NOT VALID".
  - Kiểm chứng: #971 (máy rảnh) PASS, độ trễ 0,22 s trung vị. #972 (16 vòng lặp chiếm CPU) đám điểm cũ tới 18 s,
    drone giữ suốt lượt, không bay vào vật cản.

### Tốc độ cao và close proximity (`--mode tangent`, 3×3 radar)

| Lượt | Kịch bản | Kết quả | Ghi chú |
|---|---|---|---|
| #121 | (0,0,10) → (45,0,10), 10 m/s | **PASS**, dây 1,82 m | phanh khẩn cấp (thân cây, rồi dây), dưới 2 m/s thì leo qua; tốc độ đạt 8,4 m/s |
| #122 | như trên | **FAIL**, đâm wire_b (0,22 m) | nhả phanh rồi tăng tốc gấp lên 10 m/s: chúi mũi 30–38° (ulog) đúng lúc dây cách 5–9 m, dây ra khỏi FOV, chỉ thấy lại ở 3 m. Sửa: `--cmd-accel 3` (chưa bay lại) |
| người dùng | đích trên wire_a, 1,5 m/s | FAIL, dây 0,73 m | tiếp tuyến đưa lên trên dây, v_u kéo xuống ngay đỉnh dây: dây ngay dưới bụng ở mép quạt `isk_fdown`/`isk_rdown` (−12 dB), đám điểm gần nhất 2,3 m khi dây thật 0,9 m; tiếp tuyến bằng 0 với điểm thẳng dưới. Lỗi của bố trí, không của luật |
| người dùng | đích trên wire_a, 1,5 m/s, `--tangent-gain 0 --proximity-test` | **PASS** | chỉ hình cầu an toàn: dừng trước dây |

Thử động học (không có độ nghiêng thân): dây 20 mm ở 8–10 m/s qua được, 2,35 m; dây 1,2 mm (thấy trong khoảng
7 m) ở 10 m/s đâm vào dây, với `--detect-range 7` (giới hạn 6 m/s) qua được, 1,81 m; đích trên dây: dừng ở 1,36–1,48 m.

### Phanh và giữ (`--mode brake`, bản đầu)

Profile hiện tại (2026-10-29, theo cấu hình radar của tác giả: tầm 30 m, ô góc 15°, ngưỡng 15 dB, `snr_ref` 28;
cột và thân cây dùng mô hình trụ mảnh):

| World | Vật cản | Tốc độ | Thành công | Gần nhất |
|---|---|---|---|---|
| `power_line` | thân cây 30 cm lệch 2,5 m, rồi 3 dây 20 mm + dây đất 10 mm | 5 m/s | **5/5** | thân cây 2,40–2,70 m; dừng giữ cách dây 3,62–3,96 m |
| `power_line_thin` | dây thép 1,2 mm | 2 m/s | **3/3** | dừng giữ cách dây 2,49–2,59 m |

Profile trước (2026-10-27, ước lượng: tầm 40 m, ô góc 5°, ngưỡng 12 dB; CSV trong `out/trials/profile_v1/`):

| World | Tốc độ | Thành công | Gần nhất |
|---|---|---|---|
| `power_line` | 5 m/s | 5/5 | thân cây 2,33–2,68 m; dây 3,60–3,84 m |
| `power_line_thin` | 2 m/s | 3/3 | dây 2,52–2,56 m |

Ở mọi lượt, UAV vòng qua thân cây nhờ luật tiếp tuyến, rồi dừng giữ trước dây nhờ phanh. Node không tự bay vòng hay
vượt lên trên dây: đúng theo luật của bài báo.

## Các lỗi đã sửa trên đường đi (trước đó tỉ lệ rất tệ)

1. **Đám điểm trễ, đặt sai chỗ:** đám điểm theo khung thân, trễ ~0,5 s, bị dùng với vị trí hiện tại nên ở 5 m/s
   vật cản có vẻ xa hơn thực tế 2–2,5 m. Sửa: `/radar/points_map` (toạ độ thế giới) trừ vị trí hiện tại.
2. **Phanh bật/tắt liên tục:** khi máy tải nặng, khung của radar trước có lúc rơi khỏi đám gộp (33–66 điểm thay vì
   ~250), hình nón trống, phanh nhả, UAV tăng tốc lại và nhích dần vào dây: 2/3 lượt bay xuyên dây. Sửa: phanh giữ cho
   tới khi tốc độ < v_eb (như bài báo), và node gộp đám điểm của 0,3 s gần nhất (`--keep`).

## Giới hạn

- **Vật cản nằm ngay trên đường bay (trong ±14°):** phanh giữ UAV đứng yên trước nó, không vòng qua (thân cây lệch
  0,7 m: 2/2 lượt dừng cách 4 m). Phải có lệnh khác của người lái (hoặc một bộ lập đường) để đi tiếp.
- **Dây 1,2 mm:** mô hình chỉ thấy nó dưới ~7 m; ở 5 m/s (trước bản sửa phanh) đã va chạm. Chưa đo lại ở 5 m/s,
  cũng chưa tìm tốc độ tối đa an toàn.
- Tầm phát hiện dây, độ suy giảm theo góc nhìn và mọi tham số của node là ước lượng; chưa đối chiếu với dữ liệu thật.
- Với ô góc 15°, mặt phẳng rộng (tường 8 m) chỉ còn vừa sát ngưỡng ở 20 m: mô hình tính mọi bề mặt theo R⁻⁴.
- `hover_test.py` (một radar, world `radar_hover`) chưa dùng mô hình trụ mảnh: cột của nó có thể không đạt với profile mới.
- 8 lượt là ít; các lượt khác nhau chủ yếu do nhiễu ngẫu nhiên và độ trễ của máy.
