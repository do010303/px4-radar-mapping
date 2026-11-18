# Cảm biến tránh vật cản cho UAV: radar hay lidar, tầm 100 m, xử lý trên máy bay

*Cập nhật 2026-10-21. Mỗi thông số sản phẩm và kết quả đo có số tham chiếu [n] trỏ tới danh mục
nguồn ở cuối tài liệu. Mục nào là tính toán của tôi thì ghi "tính toán của tôi" kèm giả định.
"Xử lý tại biên" trong tài liệu này hiểu là edge computing: xử lý ngay trên UAV, không qua mặt
đất. Mục 5.2 khảo sát rộng các radar có thể dùng, không giới hạn theo băng tần hay tầm.*

## Tóm tắt và khuyến nghị

**Bài toán:** tránh vật cản quanh UAV cỡ x500, phát hiện trong khoảng 100 m, mọi tính toán chạy
trên máy bay.

1. **Tầm 100 m là dư; điều quyết định là tầm với vật nhỏ nhất.**
   - Với tham số mặc định của PX4 (giảm tốc 3 m/s², trễ 0,4 s, chừa 5 m), phát hiện ở 40 m đã
     cho phép bay 13 m/s, và 100 m cho phép 22,7 m/s (mục 2). PX4 đặt trần tốc độ mặc định
     12 m/s [5].
   - Số đo về vật mảnh: radar tích hợp của DJI Matrice 400 phát hiện dây 12 mm ở 36 m và dây
     21,6 mm ở 50 m [26]; radar TI IWR6843 chỉ phát hiện dây 1,2 mm ở khoảng 10 m [1]. Tốc độ
     an toàn tương ứng khoảng 12,5, 15,3 và 4,4 m/s (bảng 2.2). Yêu cầu nên viết là "phát hiện
     vật nhỏ nhất cần né ở ít nhất 40 m", không phải "phủ 100 m".
2. **Radar là cảm biến chính, lidar là cảm biến phụ; hiệu năng hai loại vẫn được so sánh đầy đủ ở
   mục 4.** Không bên nào thắng tuyệt đối, nên lidar giữ vai trò bù đúng những chỗ radar yếu
   (mục 4.3, 6).
   - Lidar thắng ở độ phân giải góc (sai số góc dưới 0,15°, 200 000 điểm/s [8]), ở tầm với vật
     nhỏ cùng cỡ khối lượng (40 m với vật phản xạ 10% ở 265 g [8]; 190 m ở 498 g [9]), và ở độ phủ
     360° (một khối 265 g [8], trong khi radar module cần 12 khối khoảng 1,26 kg, tính toán của
     tôi).
   - Radar thắng khi có sương, mưa: ở mưa 25 mm/h, suy hao radar 77 GHz khoảng 10 dB/km còn lidar
     905 nm khoảng 35 dB/km (tầm nhìn dưới 0,5 km) [21]. Lidar 905 nm giảm 50% hiệu năng phát hiện
     mục tiêu trong buồng sương [20], và không còn điểm nào trên người đi bộ khi tầm nhìn dưới
     40 m [19].
   - Vật mảnh: radar có số đo từ 10 m (dây 1,2 mm) tới 36–50 m (dây 12–21,6 mm) [1][26]; lidar
     chưa có số đo. DJI dùng cả lidar quay (100 m với vật 10%) lẫn radar trên Matrice 400 [26],
     đúng hướng kết hợp hai loại.
3. **Xử lý trên máy bay không phải điểm nghẽn** (mục 3).
   - Lidar: tách điểm mất tối đa 29 ms mỗi khung ở 10 Hz [22]. Radar: chip TI tự làm FFT, CFAR và
     gom cụm trên DSP [11][12]; một Raspberry Pi 4B chạy tách điểm, theo dõi và bộ điều khiển né
     trong 13,65 ms từ điểm radar tới lệnh [25].
   - Độ trễ lớn nằm ở phía máy bay: trễ cảm biến tới 0,2 s và trễ bám tốc độ 0,1–0,5 s [3]. Tăng
     tần số cảm biến từ 10 lên 20 Hz chỉ tiết kiệm tối đa khoảng 50 ms, tức 0,6 m ở 12 m/s, trong
     khi quãng hãm là 24 m.
4. **Giao diện với PX4 có hai hạn chế cần biết trước** (mục 3.2): chỉ nhận mảng 72 khoảng cách
   theo phương ngang (mỗi ô 5°, không có độ cao), và chỉ hoạt động ở chế độ Position, không ở
   chế độ nhiệm vụ. PX4 cũng không có driver cho radar tránh vật cản nào (chỉ có driver radar đo
   độ cao Aerotenna uLanding [35]), trong khi ArduPilot hỗ trợ sẵn Hexsoon 77G và Nanoradar MR72
   [30]; với PX4, mọi radar đều đi qua máy tính phụ.
5. **Radar nên thử trước** (mục 5.3, chấm điểm và độ nhạy ở mục 5.4): HS77G và µSharp Patch đứng
   đầu ở cả ba bộ trọng số, cách nhau không quá 2 điểm nên chưa tách được; AWR2944 là thứ ba và là
   cái duy nhất có độ trễ đầu cuối đã đo. Điểm số chỉ sắp thứ tự để mua thử; chốt bằng bốn
   điều kiện nghiệm thu ở cuối mục 5.4. Chi tiết: Hexsoon HS77G bản 80 m và Aerotenna µSharp Patch (hai module
   ghi tầm từ 80 m trở lên và nhẹ, không nêu RCS của vật); bo TI AWR2944 hoặc IWR6843 nếu
   chấp nhận tự làm firmware (có số đo độ trễ và vùng nhìn); Nanoradar MR72 nếu cần vùng nhìn rộng
   (tầm chưa xác minh). Ainstein OA-Lite (ra mắt 11/2026) chưa có thông số.
6. **Cấu hình đề xuất** (mục 6): radar làm chính (ba µSharp Patch hướng trước và hai bên, hoặc
   HS77G / bo AWR2944 sau khi đo), Livox Mid-360 làm phụ để bù vùng nhìn hẹp, vật mảnh và
   kiểm chứng chéo khi trời quang.

## 1. Yêu cầu và giả định

| Hạng mục | Giá trị | Ghi chú |
|---|---|---|
| Nền tảng | multirotor cỡ x500 | Tải hữu ích hạn chế; mục 6 tính khối lượng và điện |
| Vật cản | tĩnh và chậm: tường, cột, cây, dây điện | Vật bay nhanh (máy bay, drone khác) không thuộc phạm vi; xem phụ lục |
| Tầm phát hiện | khoảng 100 m, cần nêu rõ cho vật cỡ nào | Mục 2 |
| Tốc độ bay | tới 12 m/s (mặc định PX4 [5]), muốn xem tới 20 m/s | Mục 2 |
| Xử lý | trên máy bay (companion computer hoặc trong cảm biến) | Mục 3 |

## 2. Cần phát hiện xa bao nhiêu

PX4 giới hạn vận tốc sao cho UAV dừng được trước vật cản, tính cả trễ [3]. Tôi dùng cùng công
thức: quãng cần = v·t + v²/(2a) + biên an toàn, với a = 3 m/s² (MPC_ACC_HOR mặc định [5]),
biên 5 m (tính toán của tôi, tương ứng CP_DIST), t = 0,4 s (CP_DELAY mặc định [5]) hoặc 0,7 s
(trễ cảm biến 0,2 s cộng trễ bám tốc độ 0,5 s, đầu cao của khoảng nêu trong [3]).

### 2.1 Quãng cần để dừng (m)

| Tốc độ | t = 0,4 s | t = 0,7 s | a = 5 m/s², t = 0,4 s |
|---|---|---|---|
| 5 m/s | 11,2 | 12,7 | 9,5 |
| 8 m/s | 18,9 | 21,3 | 14,6 |
| 10 m/s | 25,7 | 28,7 | 19,0 |
| 12 m/s (trần mặc định) | 33,8 | 37,4 | 24,2 |
| 15 m/s | 48,5 | 53,0 | 33,5 |
| 20 m/s | 79,7 | 85,7 | 53,0 |

### 2.2 Tốc độ tối đa (m/s) theo tầm phát hiện

| Tầm phát hiện vật cản | t = 0,4 s | t = 0,7 s |
|---|---|---|
| 10 m | 4,4 | 3,8 |
| 20 m | 8,4 | 7,6 |
| 27 m (radar R21-1 [13]) | 10,4 | 9,6 |
| 36 m (radar DJI Matrice 400, dây 12 mm [26]) | 12,5 | 11,7 |
| 40 m (Mid-360 với vật 10% [8]; radar MR77 [14]) | 13,3 | 12,5 |
| 50 m (radar DJI Matrice 400, dây 21,6 mm [26]) | 15,3 | 14,5 |
| 70 m | 18,6 | 17,8 |
| 80 m (radar Hexsoon HS77G bản 80 m, hãng ghi [31]) | 20,0 | 19,2 |
| 100 m | 22,7 | 21,9 |
| 120 m (radar Aerotenna µSharp Patch, hãng ghi [27]) | 25,1 | 24,3 |
| 190 m (Avia với vật 10% [9]) | 32,1 | 31,3 |

- **100 m dư với tốc độ mặc định.** Tầm 40 m đã đủ để bay hết trần 12 m/s của PX4 [5]. Chỉ khi
  muốn bay trên 15 m/s mới cần 50 m trở lên.
- **Vật mảnh mới là giới hạn, và phụ thuộc mạnh vào độ dày.** Radar tích hợp của DJI Matrice 400
  phát hiện dây 12 mm ở 36 m và dây 21,6 mm ở 50 m, vùng nhìn ±45° [26]; DJI là hãng tự công bố,
  và radar này có thể không dùng được ở một số nước [26]. Radar IWR6843 với dây 1,2 mm phát hiện từ
  khoảng 10 m (cảm biến tầm xa gắn phía trước) hoặc khoảng 7 m (các bo AOP); các tác giả báo cáo
  né dây ở trên 10 m/s với cảm biến phía trước, tối đa 5 m/s với cảm biến còn lại [1]. Hai số
  liệu cách nhau 4–5 lần về tầm, ứng với 4,4 m/s (10 m) và 12,5–15,3 m/s (36–50 m) theo bảng 2.2.
- **Lidar với vật mảnh:** không có nguồn nào đưa số tầm. Một nghiên cứu phát hiện đường dây điện
  bằng Velodyne VLP-16 nhận xét dây điện "khó được lidar phát hiện liên tục" vì kích thước nhỏ
  [22].

### 2.3 Ước tính tầm của radar 24 GHz công suất thấp (tính toán của tôi)

Giả định: λ = 12,4 mm; hệ số tạp âm 12 dB; tích luỹ kết hợp 50 ms; tổn hao hệ thống 6 dB; ngưỡng
phát hiện 13 dB; EIRP 20 dBm; anten thu 20 dBi.

| Diện tích phản xạ (RCS) | 0,01 m² | 0,1 m² | 1 m² |
|---|---|---|---|
| Tầm ước tính, 24 GHz | 94 m | 167 m | 296 m |
| Cùng EIRP và độ lợi anten, 77 GHz (λ = 3,9 mm) | 53 m | 93 m | 166 m |

Radar module công suất thấp vì vậy không bị giới hạn bởi vật lý ở 100 m với vật có RCS từ 0,01 m²
trở lên. Dây và cột mảnh có RCS nhỏ hơn nhiều, tôi chưa có nguồn cho con số cụ thể. Tầm hãng ghi
(27 m và 40 m, mục 5.2) thấp hơn các con số trên; hãng không nêu RCS của vật, nên không so sánh
trực tiếp được.

## 3. Xử lý tại biên: từ cảm biến tới lệnh điều khiển

### 3.1 Chuỗi xử lý và ai làm phần nào

| Bước | Radar (loại chip TI) | Lidar |
|---|---|---|
| Thu và tách mục tiêu | FFT theo tầm, bản đồ tầm–góc, CFAR, Doppler, góc tới, gom cụm DBSCAN, chạy ngay trên DSP của chip [11][12]. Đầu ra là danh sách điểm hoặc cụm thưa | Cảm biến xuất điểm thô: 200 000 điểm/s (Mid-360, cổng Ethernet 100 Mbps [8]), 240 000 điểm/s (Avia [9]), 1 310 720 điểm/s (OS1, 32 kênh [10]) |
| Chuyển sang khung thân máy bay | Máy tính phụ dùng tư thế từ PX4; bài báo [1] chuyển điểm của 6 radar sang khung UAV | Như radar, và cần bù chuyển động trong một khung quét (tính toán của tôi) |
| Lọc | Bài báo [1] không nêu cách loại clutter mặt đất hay vật tĩnh. Cần tự làm (mục 7) | Loại mặt đất, thân máy, lọc nhiễu. Tách điểm mất tối đa 29 ms mỗi khung ở 10 Hz trên máy i7 [22] |
| Giảm xuống dạng PX4 nhận | Với mỗi trong 72 ô phương vị 5°, lấy khoảng cách nhỏ nhất của các điểm nằm trong hành lang độ cao của UAV (tính toán của tôi) | Như radar |
| Điều khiển | Chế độ Position của PX4 hoặc bộ né tự viết (mục 3.2, 3.5) | Như radar |

### 3.2 Giao diện với PX4

- **Thông điệp:** máy tính phụ gửi `OBSTACLE_DISTANCE`: mảng 72 khoảng cách (cm), mỗi phần tử rộng
  `increment` độ, kèm tầm nhỏ nhất và lớn nhất của cảm biến [4]. Có sẵn giá trị
  `MAV_DISTANCE_SENSOR_RADAR` cho trường loại cảm biến [4]. PX4 trong repo chỉ xử lý thông điệp
  này; `OBSTACLE_DISTANCE_3D` chỉ xuất hiện trong định nghĩa MAVLink mà không có chỗ nhận [4].
- **Ô 5°:** mã nguồn dùng 72 ô, mỗi ô 5° [4] (tài liệu của PX4 có chỗ ghi 36 ô, có chỗ ghi 72 ô
  [3]; mã nguồn mới là điều đúng). Một ô 5° rộng 1,75 m ở 20 m và 8,7 m ở 100 m (tính toán của
  tôi), nên khe hẹp hơn thế không đi qua được bằng cơ chế này. Không có chiều cao: vật ở trên hay
  dưới đều bị tính như nhau, nên bước lọc theo hành lang độ cao ở mục 3.1 là bắt buộc.
- **Chế độ hoạt động:** Collision Prevention của PX4 chỉ hoạt động ở chế độ Position với
  `MPC_POS_MODE` = "Acceleration based". Nó giảm vận tốc để dừng được trước vật cản cách
  `CP_DIST`, tính cả tầm của cảm biến theo từng ô và trễ `CP_DELAY` (mặc định 0,4 s) [3][5].
  Hướng không có dữ liệu bị chặn (đặt `CP_GO_NO_DATA` = 1 để cho phép) [3].
- **Mất dữ liệu:** quá 0,5 s không có dữ liệu thì PX4 cảnh báo và đưa vận tốc ngang về 0; sau
  5 s thì chuyển sang chế độ Hold [3].
- **Mức trưởng thành:** tài liệu PX4 ghi cách cài đặt qua máy tính phụ là "chưa được kiểm thử"
  vì dự án máy tính phụ ban đầu đã ngừng bảo trì; lần thử ban đầu dùng 4 m/s và 10 Hz [3]. Cảm
  biến duy nhất PX4 hỗ trợ sẵn không cần máy tính phụ là lidar quay LightWare SF45/B (50 m, cần
  tự bật driver khi build) [6].
- **Chế độ nhiệm vụ:** giao diện lập đường đi cho nhiệm vụ (path planning interface) đã bị gỡ từ
  PX4 v1.15 và tài liệu ghi rõ không nên dùng; tài liệu nêu thư viện PX4 ROS 2 Interface là
  hướng thay thế, cho phép thay hẳn chế độ bay bằng chế độ tự viết [7]. Vậy để né khi bay tự động thì máy tính phụ phải tự
  tạo setpoint.

### 3.3 Ngân sách độ trễ

| Thành phần | Giá trị | Nguồn |
|---|---|---|
| Chờ khung quét | trung bình 50 ms, tối đa 100 ms ở 10 Hz | Mid-360 10 Hz [8]; radar bài báo 10 Hz [1]; EchoFlight 10 Hz [16]. Phép tính của tôi |
| Tách điểm trên máy tính phụ (lidar) | tối đa 29 ms | i7-4720HQ, VLP-16, 10 Hz [22]. Chạy trên ODROID-XU4 chưa có số đo |
| Xử lý radar (AWR2944, Raspberry Pi 4B) | 13,65 ms từ điểm radar tới lệnh: tách điểm 12,34 ms (gom cụm 6,00, ứng viên 2,50, hộp bao 0,71, theo dõi 3,13), kích hoạt 0,51, bộ điều khiển né 0,76, chiếu 0,04. Khi CPU đột biến có thể lên khoảng 25 ms | [25]; radar trong [1] không nêu |
| Truyền tin và MAVLink | chưa có số đo | |
| Trễ cảm biến ngoài (hệ nhìn) | tới 0,2 s | [3] |
| Trễ bám vận tốc của UAV | 0,1–0,5 s | [3] |
| `CP_DELAY` mặc định | 0,4 s | [5] |

- **Kết luận:** trễ do máy tính xử lý nhỏ hơn 30 ms (lidar tối đa 29 ms [22]; radar 13,65 ms
  [25]), so với 100–500 ms trễ vật lý của UAV. Tối ưu phần mềm trên biên ít giá trị hơn việc chọn
  tầm phát hiện và độ giảm tốc.
- **Ví dụ:** tăng từ 10 lên 20 Hz tiết kiệm tối đa khoảng 50 ms (25 ms trung bình; tính toán của
  tôi), tức 0,6 m quãng bay ở 12 m/s, trong khi quãng hãm ở 12 m/s là 24 m.

### 3.4 Phần cứng tính toán

| | Raspberry Pi 4 | ModalAI VOXL 2 | NVIDIA Jetson Orin Nano |
|---|---|---|---|
| Đã dùng cho | 6 radar IWR6843, né dây điện, ROS 2 [1] (không báo tải CPU); radar AWR2944 100 Hz, bộ theo dõi và bộ điều khiển né, UAV 2,0 kg dùng PX4 trên Pixhawk 4: 13,65 ms [25] | Bo tính toán cho UAV | Bộ phát triển "Super" |
| Tính toán | không nêu | Qualcomm QRB5165, 8 lõi, 15 TOPS [24] | 6 lõi Arm A78AE, GPU 1024 nhân, 67 TOPS INT8 [23] |
| Khối lượng | không nêu | 16 g, 70 × 36 mm [24] | không nêu trên trang [23] |
| Công suất | không nêu | 0,1–7 W [24] | 7–25 W [23] |
| PX4 | nối qua cổng nối tiếp USB (PX4 1.13) [1] | PX4 hoặc ArduPilot chạy trên DSP của bo [24] | máy tính phụ qua UART hoặc Ethernet |

Radar đã xử lý sẵn nên chỉ cần máy tính phụ nhỏ (Pi 4 đủ trong [1]). Lidar xuất 0,2–1,3 triệu
điểm/s nên nên dùng bo mạnh hơn.

### 3.5 Thuật toán né trên biên: ví dụ từ bài báo ICRA 2026

Bài báo [1] (video trình bày của họ ở [2]) có một hệ chạy thật, nhỏ gọn:
- **Phần cứng:** khung Holybro QAV250 (250 mm), PX4 1.13 trên Pixhawk 4 Mini, 6 radar TI
  IWR6843 (một IWR6843ISK nhìn trước 120° × 30°, năm IWR6843AOPEVM), Raspberry Pi 4, 10 Hz [1].
- **Thuật toán, không dựng bản đồ**, ba lớp trên các điểm radar thô:
  1. **Lái theo tiếp tuyến** quanh điểm phát hiện trong quả cầu bán kính r_a.
  2. **Phanh gấp:** nếu điểm nằm trong hình nón dài l_H = v²/(2·a_max) + biên và nửa góc khoảng
     0,245 rad thì ra lệnh phanh tối đa.
  3. **Đẩy ra** khi vào quả cầu an toàn r_s.
  Bộ điều khiển gửi vận tốc tới PX4; bài báo không nêu loại thông điệp, các tham số r_a, r_s,
  a_max, biên, và không báo độ trễ hay tải CPU [1].
- **Kết quả:** phát hiện ổn định tới khoảng 10 m, né dây 1,2 mm ở tốc độ trên 10 m/s (cảm biến
  phía trước), sai số khoảng cách trung bình 6,1 cm [1].
- **Hạn chế bài báo tự nêu:** vùng nhìn đo được của các bo AOP chỉ khoảng 75° theo phương vị
  (danh định 120°), tạo điểm mù trong mặt phẳng ngang; chỉ thử trên một đường dây dài khoảng
  35 m, dây mảnh nhất là 1,2 mm [1].

Cách này nhẹ và đủ nhanh cho radar vì không phải dựng bản đồ. Đổi lại, nó chỉ né được những gì
cảm biến thấy ở tầm vài chục mét.

**Bài báo thứ hai, né vật bay nhanh ([25], arXiv 2608.01855, 8/2026):**
- **Phần cứng:** bo radar TI AWR2944PEVM (77–81 GHz, vùng nhìn khoảng ±80° phương vị và ±20°
  góc ngẩng), 100 Hz; UAV 2,0 kg, 52 × 52 × 25 cm, tỉ lệ lực đẩy trên trọng lượng khoảng 3,
  Pixhawk 4 chạy PX4; Raspberry Pi 4B, ROS 2 Galactic [25].
- **Thuật toán:** bộ theo dõi nhiều mô hình tương tác (IMM) và bộ điều khiển dùng hàm rào điều
  khiển, ra thẳng gia tốc né [25]. Tác giả suy ra điều kiện đủ để né: tốc độ cho phép
  v_max = √(2aR) − a·t_cuối (R: tầm phát hiện, a: gia tốc né, t_cuối: độ trễ đầu cuối) [25], là
  cùng dạng với công thức ở mục 2 (chưa có biên an toàn).
- **Đo đạc:** 300 lần ném bóng (đường kính 0,1, 0,2 và 0,3 m, RCS nhỏ nhất khoảng 0,001 m²) trong
  điều kiện sáng và tối, 90 lần trong khói; né thành công khoảng 89% [25]. Độ trễ đầu cuối
  13,65 ms (bảng 3.3).
- **Giới hạn của bài báo tự nêu:** tầm cấu hình tối đa 29,3 m và chỉ xét trong 15 m có đường nhìn
  thẳng; phân tích dùng R ≥ 4 m; bay thử trong nhà, có dây giữ; phát hiện kém ở 0–1 m (trong khói
  6,7–53,3%); sai số vị trí lớn theo hai trục y, z (trung bình tới 0,93 và 0,87 m) do độ phân
  giải tầm khoảng 0,23 m và điểm thưa [25].
- **Liên quan tới bài toán này:** nó chứng minh xử lý radar đầu cuối trên Pi 4B rất nhanh, nhưng
  chỉ cho vật động ở tầm vài mét, không cho vật tĩnh ở 40–100 m.

## 4. So sánh lidar và radar

### 4.1 Theo tiêu chí

| Tiêu chí | Lidar | Radar | Bên có lợi |
|---|---|---|---|
| Độ phân giải góc, mật độ điểm | Mid-360: sai số góc < 0,15° (1σ), 200 000 điểm/s [8]. OS1: ±0,01°, 1,31 triệu điểm/s [10] | R21-1: búp ±15° × ±4° ở −6 dB [13]. Vòng IWR6843: sai số khoảng cách 6,1 cm nhưng vùng nhìn thật hẹp hơn danh định [1]. EchoFlight: độ chính xác bám < 1° × < 1,5° [16] | **Lidar** |
| Tầm với vật nhỏ, cùng cỡ khối lượng | 40 m với vật 10% (Mid-360, 265 g) [8]; 90 m (OS1, 430 g) [10]; 190 m (Avia, 498 g) [9] | 27 m (R21-1, 105 g) [13]; 40 m (MR77, 90 g) [14]; 80–120 m ghi trên µSharp Patch (22–43 g) và HS77G-80 [27][31], không nêu RCS; vật mảnh 1,2 mm khoảng 10 m [1] | **Lidar** có số đo với vật 10%; radar chưa có số đo độc lập |
| Sương, mưa | Sương, tầm nhìn 20–30 m: mật độ điểm trên ô tô ở khoảng 19 m còn 0,36 (VLP-16) và 0,04 (Valeo Scala) so với trời quang; người đi bộ không còn điểm nào khi tầm nhìn dưới 40 m [19]. Giảm 50% hiệu năng phát hiện trong buồng sương [20]. Mưa 25 mm/h: khoảng 35 dB/km [21] | Mưa 25 mm/h: khoảng 10 dB/km ở 77 GHz [21]. Tuyết 10 mm/h: khoảng 6 dB/km [21]. Chưa có số cho 24 GHz | **Radar** |
| Bụi, bẩn trên cửa sổ | Bụi phủ đều trên cửa sổ cảm biến làm tầm tối đa giảm 75% [21] | Chưa có nguồn | Chưa kết luận |
| Ánh nắng | Tầm công bố ở 100 klx (Mid-360 [8], OS1 [10]); Avia như nhau ở 0 và 100 klx [9] | Không phụ thuộc ánh sáng (hiển nhiên về vật lý, không trích nguồn) | Hòa với các cảm biến trên |
| Dây | VLP-16: khó phát hiện liên tục [22]; DJI không đưa số dây cho lidar [26] | 1,2 mm: khoảng 10 m [1]. 12 mm: 36 m; 21,6 mm: 50 m (DJI) [26] | **Radar** (là bên duy nhất có số đo) |
| Khói | Tác giả [25] nhận xét lidar "vẫn khó với khói dày" (nhận xét, không đo) | Trong khói, quả bóng 0,3 m phát hiện 100% khi xa hơn 3 m, nhưng chỉ 6,7–53,3% ở 0–1 m [25] | Radar, với điều kiện vật đủ lớn |
| Khối lượng, điện | Mid-360 265 g, 6,5 W [8]; OS1 430 g, 14–20 W [10]; Avia 498 g, 8–9 W [9] | R21-1 105 g, ≤ 3 W [13]; MR77 90 g (150 g kèm cáp), cần máy tính SkyHub [14]; EchoFlight 817 g, 45 W [16] | **Radar** (mỗi khối) |
| Phủ 360° | Mid-360: 360° × 59° trong một khối 265 g [8] | R21-1 phủ ±15°: cần 12 khối khoảng 1,26 kg (tính toán của tôi) | **Lidar** |
| Tải tính toán | Điểm thô 0,2–1,3 triệu điểm/s [8][9][10]; tách điểm ≤ 29 ms trên i7 [22] | Xử lý sẵn trên chip [11][12]; Pi 4 đủ [1] | **Radar** (nhẹ hơn) |
| Tích hợp PX4 | SF45/B có driver, dùng được không cần máy tính phụ [6] | PX4 không có driver radar tránh vật cản, chỉ có radar đo độ cao Aerotenna uLanding [35]; có kiểu cảm biến radar trong thông điệp [4]. ArduPilot hỗ trợ sẵn Hexsoon 77G (từ 4.6) và Nanoradar MR72 (từ 4.5), đều qua CAN [30]. R21-1 ghi "Pixhawk/ArduPilot", không nhắc PX4 [13]; MR77 cần SkyHub [14] | Hơi nghiêng về lidar |

### 4.2 Thời tiết: tổn hao thêm ở tầm của từng cảm biến (tính toán của tôi)

Giả sử suy hao theo km [21] áp dụng đều trên đường đi và về, tổn hao thêm = 2 × quãng × suy hao
theo km. Mưa 25 mm/h: radar 77 GHz khoảng 10 dB/km, lidar 905 nm khoảng 35 dB/km (tầm nhìn dưới
0,5 km).

| Quãng | Radar 77 GHz | Lidar 905 nm |
|---|---|---|
| 27 m (tầm R21-1 [13]) | 0,5 dB | 1,9 dB |
| 40 m (tầm Mid-360 [8]) | 0,8 dB | 2,8 dB |
| 100 m | 2 dB | 7 dB |
| 190 m (tầm Avia [9]) | 3,8 dB | 13,3 dB |

- **Lidar tầm xa chịu thiệt nhất:** ở tầm công bố của Avia, mưa lớn thêm khoảng 13 dB tổn hao; đổi
  ra tầm còn lại cần mô hình công suất của từng cảm biến, tôi chưa có số đo.
- **Sương dày là kiểu hỏng khác:** tầm nhìn 20–30 m làm mật độ điểm trên ô tô còn 0,04–0,36 và
  tầm nhìn dưới 40 m làm mất hết điểm trên người đi bộ [19]; hiệu năng phát hiện giảm 50% trong
  các thử nghiệm sương của [20].
- **Số liệu radar ở đây là cho 77 GHz.** Chưa có số cho 24 GHz trong các nguồn đã mở.

### 4.3 Kết luận: cái nào tối ưu hơn

| Điều kiện bay | Chọn | Lý do |
|---|---|---|
| Trời quang, cần 360°, vật nhỏ và mảnh | **Lidar** (Mid-360) | Độ phân giải, độ phủ 360°, một khối nhẹ |
| Bay thẳng về phía trước nhanh hơn 12 m/s | **Lidar nhìn trước** (Avia 190 m [9], hoặc OS1 90 m [10], với vật 10%) | Radar module chỉ tới 27–40 m [13][14] |
| Sương dày, mưa lớn, tuyết | **Radar** | Suy hao thấp hơn lidar 905 nm [21] |
| Cần đo vận tốc mục tiêu | **Radar** | Doppler có sẵn trên chip [12] |
| Dây dày (12–21,6 mm) ở 36–50 m | **Radar** (có số đo của DJI [26]) | Lidar không có số tương ứng |
| Dây mảnh (khoảng 1 mm) xa hơn 10 m | **Chưa biết** | Không nguồn nào có số cho lidar; radar là 10 m [1] |

**Quyết định của đề tài: radar làm chính, lidar làm phụ.** Bảng trên là hiệu năng thuần, và nó
cho thấy lidar hơn radar ở phân giải, tầm với vật 10% cùng cỡ khối lượng và độ phủ 360°. Chọn
radar làm chính vì thế là một lựa chọn có giá phải trả, không phải kết luận từ hiệu năng:
- **Radar chính trả giá:** vùng nhìn của từng radar module hẹp (Patch 50°, R21-1 ±15°), phải ghép
  nhiều khối; chưa có số đo độc lập với vật mảnh (mục 5.3); tầm hãng ghi không kèm RCS.
- **Lidar phụ bù đúng các chỗ đó:** phủ 360° bằng một khối, phân giải góc cao để thấy cột và cành
  khi trời quang, và đối chiếu với radar để lọc báo giả (mặt đất, nhiễu).
- **Radar giữ phần việc không thể thay:** sương, mưa, bụi, khói, và dây dày có số đo (mục 4.1–4.2).
- **Khi hai loại bất đồng**, quy tắc đề xuất của tôi: lấy khoảng cách gần hơn (thận trọng) cho
  `OBSTACLE_DISTANCE`, ghi lại cả hai để đo ở mục 7.

## 5. Các cảm biến đã tìm

### 5.1 Lidar

| | Livox Mid-360 [8] | Livox Avia [9] | Ouster OS1 Rev 7 [10] | LightWare SF45/B [6] |
|---|---|---|---|---|
| Tầm (vật 10%) | 40 m (70 m với vật 80%) | 190 m (320 m với vật 80% ở 100 klx) | 90 m (170 m với vật 80%) | khoảng 50 m |
| Vùng nhìn | 360° × (−7° đến 52°) | 70,4° × 77,2° | 360° × 42,4° | quay, khoảng 320° ngang, quét một mặt phẳng |
| Điểm/giây | 200 000 | 240 000 (một lần phản hồi) | 1 310 720 (32 kênh) | |
| Tần số quét | 10 Hz | | 10 Hz (chế độ 1024) | |
| Khối lượng | 265 g | 498 g | 430 g (không nắp) | |
| Công suất | 6,5 W (tối đa 14 W khi tự sưởi) | 8–9 W (16 W khi khởi động) | 14–20 W | |
| Cấp bảo vệ | IP67 | IP67 | IP68, IP69K | |
| Nhiệt độ | −20 đến 55 °C | −20 đến 65 °C | −40 đến 60 °C | |
| Ghi chú | Hãng ghi tầm có thể giảm khi sương dày | | | PX4 có driver, không cần máy tính phụ |

### 5.2 Radar: khảo sát rộng

Không giới hạn theo băng tần hay tầm như bản trước. Tôi chia theo loại sản phẩm vì chúng khác nhau
nhiều về khối lượng, nguồn thông số và cách tích hợp. Độ phân giải tầm phụ thuộc cấu hình chirp,
không chỉ băng tần: chip TI 77–81 GHz có băng thông từ 4 GHz trở lên [28][29], nhưng bài báo [25] cấu
hình để đạt tầm tối đa 29,3 m với độ phân giải khoảng 0,23 m, còn µSharp 24 GHz ghi 22 cm [27].

#### Nhóm A: module radar cho UAV

| | Aerotenna µSharp Patch [27] | Aerotenna µSharp 360° [27] | MicroBrain UAV-R21-1 [13] | SPH MR77 [14] | Hexsoon HS77G [30][31] | Nanoradar MR72 [30] |
|---|---|---|---|---|---|---|
| Băng tần | 24 GHz | 24,00–24,25 GHz | 79 GHz (77–81) | 77 GHz | băng 77 GHz | 77 GHz |
| Tầm | 0,5–120 m, không nêu RCS | 40 m trong bảng thông số, 120 m trong phần mô tả của cùng datasheet | 0,2–27 m (chức năng ghi 1,5–27 m) | tới 40 m | ba bản 20, 40, 80 m (theo tên bản); 0–80 m theo bài đăng của hãng | không có trong tài liệu ArduPilot |
| Vùng nhìn | 50° × 30° (−3 dB) | quét 360° ngang | ±15° × ±4° (−6 dB) | không nêu | 120° theo bài đăng của hãng | 112° (theo tên mục ArduPilot) |
| Tần số | 90 Hz | 80 Hz | 20 Hz | không nêu | không nêu | không nêu |
| Khối lượng, kích thước | 22 g (43 g có vỏ); 76 × 54 × 13 mm | 243 g; 115 × 115 × 67 mm | 105 g; 76 × 71,5 × 19,6 mm | 90 g (150 g kèm cáp) | không nêu | không nêu |
| Công suất | 1,25 W (5 V, 250 mA) | 2,5 W (5 V) | ≤ 3 W | không nêu | BEC ngoài 5–28 V khi dùng nhiều radar | không nêu |
| Giao tiếp | UART; CAN theo yêu cầu | UART | CAN | cần máy tính SkyHub | CAN (ArduPilot chỉ hỗ trợ bản CAN) | CAN (bản nối tiếp không hỗ trợ chính thức) |
| Bộ điều khiển | "bộ điều khiển mã nguồn mở" và OcPoC (datasheet) | như Patch | "Pixhawk/ArduPilot/Open Source", không nhắc PX4 | cần SkyHub | ArduPilot Copter và Rover từ 4.6, tối đa 3 radar | ArduPilot từ 4.5, loại proximity (Plane không dùng) |
| Độ chính xác, độ phân giải | 22 cm | 22 cm | ±0,02 m; 0,12 m | không nêu | không nêu | không nêu |
| Nguồn thông số | datasheet hãng 09/02/2018 | datasheet hãng 23/10/2017; tầm tự mâu thuẫn | trang hãng | trang cửa hàng của hãng, 593 USD | bài đăng của hãng 07/11/2025 và tài liệu ArduPilot; không có datasheet | tài liệu ArduPilot; không mở được datasheet hãng |

- **Aerotenna µSharp:** datasheet nêu công suất phát khoảng 17 dBm (Patch), nhiệt độ −15 đến
  +50 °C [27]. Tôi chưa xác nhận sản phẩm còn được bán.
- **MicroBrain UAV-R121-1:** một nhà bán lẻ ghi tầm 120 m và 170 g; tôi không mở được trang và không
  có datasheet hãng, nên không đưa vào bảng.
- **Hexsoon HS77G:** tài liệu ArduPilot không nêu tầm, vùng nhìn hay khối lượng; các số 20/40/80 m
  và 120° lấy từ tên bản và bài đăng của hãng [30][31], chưa có đo độc lập.

#### Nhóm B: chip và bo thử TI (tự làm firmware)

| | IWR6843AOP [11] | AWR1843AOP [28] | AWR2944 [29] |
|---|---|---|---|
| Băng tần | 60–64 GHz, băng thông 4 GHz | 76–81 GHz, băng thông 4 GHz | 76–81 GHz, băng thông trên 4 GHz |
| Anten, kênh | 4 thu, 3 phát, anten trên gói | 4 thu, 3 phát, anten trên gói | 4 thu, 3–4 phát, anten ngoài |
| Công suất phát, tạp âm | 10 dBm | EIRP 16 dBm; hệ số tạp âm 10 dB | 13,5 dBm |
| Xử lý trên chip | R4F 200 MHz, DSP C674x 600 MHz, bộ tăng tốc FFT/lọc/CFAR | R4F 200 MHz, DSP C674x 600 MHz, bộ tăng tốc radar, RAM 2 MB | R5F 300 MHz, DSP C66x 360 MHz, bộ tăng tốc HWA 2.1, RAM 4 MB |
| Giao tiếp | CAN-FD, UART, SPI, LVDS | 2 CAN (một CAN-FD), UART, SPI, LVDS | 2 CAN-FD, Ethernet 10/100, 4 UART, LVDS 4 làn, CSI-2 |
| Kết quả trong nghiên cứu | 6 bo: dây 1,2 mm khoảng 10 m; vùng nhìn bo AOP đo được khoảng 75° phương vị, 104–135° góc ngẩng [1] | thiết kế tham chiếu chuỗi xử lý tránh vật cản [12]; chưa có kết quả hiện trường | bo AWR2944PEVM: vùng nhìn khoảng ±80° × ±20°, 100 Hz, cấu hình tới 29,3 m, thử tới 15 m; bóng RCS 0,001 m² phát hiện 96–98% ở 3–5 m (trời sáng); 13,65 ms đầu cuối [25] |

Các bo này không có khối lượng hay tầm công bố. Muốn dùng phải tự cấu hình chirp, viết bộ lọc
clutter và đóng gói. Đổi lại, đã có hai bài báo chạy thật trên Raspberry Pi với PX4 [1][25].

#### Nhóm C: radar tích hợp trong UAV thương mại (làm mốc so sánh)

- **DJI Matrice 400** [26]: radar mmWave ±45° (ngang và dọc), 36 m với dây 12 mm và 50 m với dây
  21,6 mm; lidar quay 0,5–100 m ở 100 klx với vật 10%, 360° × 58°, 520 000 điểm/s; thị giác
  0,4–21 m. Radar có thể không dùng được ở một số nước.
- **Mô-đun cảm biến vật cản của Matrice 4** [32]: 225 g, 103,3 × 64 × 85,8 mm, IP55 (khi dùng với
  Matrice 4D), −20 đến 50 °C; lidar quay kết hợp radar mmWave năm hướng; tương thích dòng Matrice
  4 và 4D; 3 180 NZD, đang hết hàng ở nhà bán lẻ này. Trang nhà bán lẻ không nêu tầm.
- **Ý nghĩa:** đây là hệ khép kín của DJI, không dùng được với PX4. Giá trị của chúng là cho thấy
  một nhà sản xuất lớn chọn kết hợp lidar quay với radar, và đưa ra số đo dây dày duy nhất.

#### Nhóm D: radar ảnh 4D cho ô tô

Arbe Phoenix, Continental/Aumovio ARS 548 RDI, Ainstein K-79 và Sensrad Hugin D1 là radar ảnh
4D thiết kế cho ô tô hoặc xe công nghiệp (theo kết quả tìm kiếm). Tôi chưa đọc được datasheet hãng
của nhóm này: trang ARS 548 lỗi chứng chỉ, trang Sensrad bị chặn, và các kết quả tìm kiếm về K-79
không có thông số. Vì vậy không đưa khối lượng, công suất hay tầm. Chưa nguồn nào tôi mở được cho
thấy chúng được dùng trên UAV nhỏ.

#### Nhóm E: radar cho vật bay, radar công nghiệp, và sản phẩm sắp ra

- **Echodyne EchoFlight** [15][16]: 817 g, 45 W, vùng nhìn 120° × 80°, 10 Hz, tầm hàng trăm mét
  đến km với drone và máy bay. Nặng và đắt cho tránh vật cản ở 100 m, không công bố tầm với vật
  tĩnh. Xem phụ lục.
- **Ainstein OA-Lite và OA-Pro** [33]: hãng báo sẽ ra mắt 11/2026. OA-Lite "tích hợp sẵn, công
  suất thấp, cho nhà sản xuất UAV thương mại"; OA-Pro cho nền tảng UAM, AAM và vận tải lớn, tầm xa
  hơn. Chưa có thông số nào: nên theo dõi.
- **InnoSenT iSYS-4001** [18]: 24 GHz công nghiệp, 0,3–150 m cho vật chuyển động (tuỳ RCS), vùng
  nhìn 34° × 49°. Không thiết kế cho UAV và không nêu tầm với vật tĩnh.

#### Nhóm F: radar đo độ cao (loại khỏi bài toán)

Ainstein US-D1: 50 m, 43° × 30°, 100 g, "tối ưu cho bám địa hình" [34]. ArduPilot xếp US-D1,
LR-D1 và Hexsoon 24G vào nhóm cảm biến đo khoảng cách một hướng [30]. Đây không phải cảm biến
tránh vật cản.

### 5.3 Đánh giá radar theo bài toán

Ký hiệu: ✔ đạt theo nguồn; ✘ không đạt; ~ một phần; ? không có nguồn.

| Ứng viên | Tầm từ 40 m (theo hãng) | Số đo vật mảnh | Vùng nhìn ngang từ 90° | Dưới 200 g | Dùng với ArduPilot hoặc PX4 | Datasheet hãng |
|---|---|---|---|---|---|---|
| Aerotenna µSharp Patch [27] | ✔ 120 m | ? | ✘ 50° | ✔ 22–43 g | ~ UART, CAN; PX4 chưa có driver | ✔ (2018) |
| Hexsoon HS77G, bản 80 m [30][31] | ✔ 80 m (bài đăng của hãng) | ? | ✔ 120° (hãng ghi) | ? | ✔ ArduPilot từ 4.6; ✘ PX4 | ✘ |
| Nanoradar MR72 [30] | ? | ? | ✔ 112° | ? | ✔ ArduPilot từ 4.5; ✘ PX4 | ✘ |
| MicroBrain R21-1 [13] | ✘ 27 m | ? | ✘ ±15° | ✔ 105 g | ~ | ✔ trang hãng |
| SPH MR77 [14] | ✔ 40 m | ? | ? | ✔ 90 g | ✘ cần SkyHub | ~ trang cửa hàng |
| TI AWR2944, bo thử [25][29] | ✘ cấu hình 29,3 m, thử 15 m | ~ RCS 0,001 m² phát hiện 96–98% ở 3–5 m | ✔ ±80° (góc ngẩng ±20°) | ? | ~ tự tích hợp qua Pi | ~ chỉ có chip |
| TI IWR6843AOP, bo thử [1][11] | ✘ khoảng 10 m với dây 1,2 mm | ✔ 1,2 mm: 10 m | ✘ khoảng 75° đo được | ? | ~ tự tích hợp qua Pi | ~ chỉ có chip |
| DJI Matrice 400, radar [26] | ✔ 36–50 m với dây dày | ✔ 12 mm: 36 m; 21,6 mm: 50 m | ✘ ±45° | không bán rời | ✘ khép kín | ✔ DJI công bố |
| Echodyne EchoFlight [15][16] | ✔ (vật bay) | ? | ✔ 120° | ✘ 817 g | ~ Ethernet | ✔ |

**Nên thử trước, theo thứ tự:**
1. **Hexsoon HS77G bản 80 m và Aerotenna µSharp Patch.** Hai module ghi tầm từ 80 m trở lên và
   nhẹ (µSharp Patch 22–43 g; µSharp 360° nặng 243 g và datasheet tự mâu thuẫn 40/120 m). HS77G có vùng nhìn 120° và
   dùng được với ArduPilot nhưng chỉ có bài đăng của hãng. Patch có datasheet đầy đủ nhưng từ 2018 và
   chưa xác nhận còn bán. Cả hai không nêu RCS của vật cho tầm đó, nên phải đo.
2. **Bo TI AWR2944 hoặc IWR6843.** Mở và có đường dẫn nghiên cứu (độ trễ 13,65 ms, 100 Hz, tránh
   dây ở trên 10 m/s), nhưng tầm đã thử chỉ 10–15 m và phải tự làm firmware và bộ lọc.
3. **Nanoradar MR72** nếu cần vùng nhìn rộng 112° và đã dùng ArduPilot; tầm chưa xác minh.
4. **Theo dõi Ainstein OA-Lite** (11/2026).
5. **Không phù hợp:** EchoFlight (nặng, tầm xa cho vật bay), radar đo độ cao, và nhóm ô tô (chưa có
   thông số).

**Điều chưa có:** không radar nào ở trên có số đo độc lập về tầm với vật mảnh từ 40 m. Số duy nhất
là của DJI (tự công bố, sản phẩm đóng, dây 12–21,6 mm).

### 5.4 Chọn radar: loại bắt buộc, chấm điểm, độ nhạy

Bảng 5.3 chỉ liệt kê; mục này đưa ra kết luận theo ba bước: loại ứng viên không đạt điều kiện bắt
buộc, chấm điểm có trọng số các ứng viên còn lại, rồi kiểm tra kết quả có đổi khi đổi trọng số
không. Toàn bộ điểm và trọng số là tính toán của tôi; đầu vào là số trong mục 5.2, mỗi điểm có
quy tắc ghi bên dưới nên có thể chấm lại.

**Bước 1. Điều kiện bắt buộc (loại ngay nếu không đạt)**

| Điều kiện | Lý do | Bị loại |
|---|---|---|
| Nặng không quá 500 g mỗi khối (giả định của tôi cho UAV cỡ x500) | Tải hữu ích hạn chế, mục 1 | EchoFlight (817 g, 45 W [16]) |
| Có đường nối tới PX4 qua máy tính phụ, không bị khoá | PX4 không có driver radar tránh vật cản (mục 3.2), nên mọi radar phải đi qua máy tính phụ | DJI Matrice 400 và Matrice 4 (hệ khép kín [26][32]) |
| Là cảm biến tránh vật cản, không phải đo độ cao | Mục 5.2 nhóm F | Ainstein US-D1, LR-D1, Hexsoon 24G |
| Có thông số hoặc nguồn đọc được | Nhóm D chưa có số nào | Arbe Phoenix, ARS 548, Ainstein K-79, Sensrad Hugin D1 (chưa đủ dữ liệu, không phải bị chứng minh là kém) |

Còn lại 8 ứng viên. Khối lượng của HS77G, MR72 và các bo TI chưa có nguồn nên chỉ "chưa bị loại".

**Bước 2. Tiêu chí, trọng số "cân bằng" và quy tắc chấm (0–3)**

| Tiêu chí | Trọng số | 3 điểm | 2 | 1 | 0 |
|---|---|---|---|---|---|
| Tầm (hãng ghi hoặc đã thử) | 25 | ≥ 80 m | 40–79 m | 20–39 m | < 20 m hoặc không có số |
| Số đo vật mảnh | 20 | đo ≥ 36 m | đo 20–35 m | đo có số nhưng < 20 m | không có số |
| Vùng nhìn ngang | 10 | ≥ 110° | 90–109° | 50–89° | < 50° hoặc không nêu |
| Khối lượng mỗi khối | 10 | < 100 g | 100–200 g | 200–500 g | > 500 g hoặc không nêu |
| Nối với PX4/ArduPilot | 15 | PX4 có driver | ArduPilot hỗ trợ sẵn, hoặc có bài báo chạy với PX4 | có UART/CAN nhưng không có driver | khép kín hoặc cần phần cứng riêng |
| Độ tin cậy nguồn | 10 | datasheet chip/hãng hiện hành cùng bài báo đo | datasheet hãng | chỉ bài đăng hoặc trang hãng | không có |
| Xử lý tại biên | 10 | có số đo độ trễ đầu cuối | module xuất sẵn khoảng cách/vật qua CAN hoặc UART, có driver proximity hoặc tần số ghi | cần máy tính ngoài để xử lý | không có |

Với tầm, tôi lấy số **hãng ghi** (không có số đo độc lập cho radar module); vì vậy tiêu chí "độ tin cậy
nguồn" riêng để trừ điểm những số chỉ có từ bài đăng. Số chưa biết chấm 0, tức là chấm thận trọng.

**Bước 3. Điểm từng tiêu chí (tầm, vật mảnh, vùng nhìn, khối lượng, nối PX4, nguồn, tại biên)**

| Ứng viên | Tầm | Vật mảnh | Vùng nhìn | Khối lượng | Nối PX4 | Nguồn | Tại biên | Giải thích điểm khác thường |
|---|---|---|---|---|---|---|---|---|
| µSharp Patch [27] | 3 | 0 | 1 | 3 | 1 | 2 | 2 | 120 m hãng ghi; 50°; 22–43 g; UART/CAN, 90 Hz; datasheet 2018 |
| µSharp 360° [27] | 2 | 0 | 3 | 1 | 1 | 1 | 2 | lấy 40 m (số thấp trong hai số tự mâu thuẫn); 243 g |
| HS77G bản 80 m [30][31] | 3 | 0 | 3 | 0 | 2 | 1 | 2 | 80 m và 120° chỉ từ bài đăng hãng; khối lượng không nêu |
| MR72 [30] | 0 | 0 | 3 | 0 | 2 | 1 | 2 | tầm, khối lượng không có nguồn; 112° theo tên mục ArduPilot |
| R21-1 [13] | 1 | 0 | 0 | 2 | 1 | 2 | 2 | 27 m; ±15°; 105 g |
| MR77 [14] | 2 | 0 | 0 | 3 | 0 | 1 | 1 | 40 m; 90 g; cần SkyHub; vùng nhìn không nêu |
| AWR2944, bo thử [25][29] | 0 | 1 | 3 | 0 | 2 | 3 | 3 | tầm đã thử 15 m; bóng 0,001 m² ở 3–5 m; ±80°; 13,65 ms đầu cuối |
| IWR6843AOP, bo thử [1][11] | 0 | 1 | 1 | 0 | 2 | 3 | 2 | dây 1,2 mm ở khoảng 10 m; vùng nhìn đo được khoảng 75° |

**Tổng điểm /100 (điểm × trọng số / 3) và độ nhạy với trọng số**

Hai bộ trọng số còn lại (tính toán của tôi): *ưu tiên tốc độ* nâng tầm và vật mảnh lên 35 và 30,
hạ khối lượng, tích hợp, nguồn xuống 5; *ưu tiên tích hợp* nâng nối PX4 lên 30 và tại biên lên 15,
hạ tầm xuống 15 và vật mảnh xuống 10.

| Ứng viên | Cân bằng | Ưu tiên tốc độ | Ưu tiên tích hợp |
|---|---|---|---|
| µSharp Patch | **56,7** | 55,0 | 55,0 |
| HS77G bản 80 m | 55,0 | **56,7** | **58,3** |
| AWR2944 (bo thử) | 46,7 | 38,3 | **58,3** |
| µSharp 360° | 45,0 | 45,0 | 46,7 |
| IWR6843AOP (bo thử) | 36,7 | 28,3 | 46,7 |
| R21-1 | 33,3 | 26,7 | 38,3 |
| MR77 | 33,3 | 33,3 | 28,3 |
| MR72 | 30,0 | 21,7 | 43,3 |

**Kết luận từ ba bộ trọng số**

1. **Hai ứng viên đầu cố định: µSharp Patch và HS77G bản 80 m**, luôn đứng đầu hoặc đồng hạng
   đầu, cách nhau không quá 2 điểm. Với dữ liệu hiện có, **không thể tách hai cái này**; cũng
   không nên cố, vì cả hai đều lấy điểm tầm từ số hãng ghi và đều 0 điểm ở vật mảnh.
2. **Bo AWR2944 là ứng viên thứ ba** và bằng HS77G khi ưu tiên tích hợp (58,3), vì là cái duy nhất
   có độ trễ đầu cuối đã đo và đã chạy với PX4 trên Raspberry Pi [25]. Điểm yếu của nó là tầm đã thử
   chỉ 15 m, cùng việc phải tự viết firmware.
3. **R21-1, MR77, MR72 xếp cuối** ở hầu hết các bộ trọng số: R21-1 và MR77 vì vùng nhìn hẹp hoặc
   không rõ và tầm ngắn, MR72 vì thiếu hẳn thông số. MR72 lên 43,3 ở bộ tích hợp chỉ nhờ điểm
   ArduPilot, không phải nhờ hiệu năng.
4. **Điểm số này chỉ sắp thứ tự theo thông tin hiện có, không chứng minh radar nào phát hiện được
   vật mảnh.** Không ứng viên nào có điểm vật mảnh quá 1/3. Nếu ba bộ trọng số cho cùng kết quả đầu
   bảng thì thứ tự này đáng tin cho việc **chọn mua thử**, chưa đủ để **chọn bay**.

**Kết luận chung: loại nào ổn nhất**

| Câu hỏi | Trả lời | Cơ sở |
|---|---|---|
| Cảm biến chính, cảm biến phụ? | **Radar chính, lidar (Mid-360) phụ**, giữ so sánh hiệu năng ở mục 4 | Quyết định của đề tài, mục 4.3 |
| Radar nào mua thử trước? | **Hexsoon HS77G bản 80 m và Aerotenna µSharp Patch**, mỗi loại một cái | Đầu bảng ở cả ba bộ trọng số, không tách được |
| Nếu muốn một đường xử lý biên có số đo thật trên PX4? | **Bo TI AWR2944** (13,65 ms trên Pi 4B [25]) | Duy nhất có độ trễ đầu cuối đã đo |
| Không nên mua | EchoFlight; radar đo độ cao; radar ô tô (đến khi có datasheet) | Bước 1 |

**Tiêu chí nghiệm thu để chốt (đề xuất của tôi, lấy mốc từ số của DJI [26] và bảng 2.2)**

Cho phép bay 12 m/s (trần PX4 [5]) cần phát hiện vật cần né ở ít nhất 36 m (bảng 2.2, t = 0,4 s).
Một radar được chọn để bay khi đạt cả bốn điều kiện đo tại bãi thử (mục 7):
1. Dây 12 mm: phát hiện ổn định ở ≥ 36 m (bằng mốc DJI); nếu không đạt, giới hạn tốc độ theo bảng 2.2.
2. Cột 10 cm hoặc tường: phát hiện ổn định ở ≥ 40 m.
3. Tốc độ báo cáo ≥ 10 Hz và độ trễ đầu cuối từ cảm biến tới `OBSTACLE_DISTANCE` ≤ 50 ms (đo trên
   máy tính phụ; [25] đo được 13,65 ms với bo thử).
4. Số điểm mặt đất lọt vào hành lang độ cao khi bay thấp: không gây dừng giả trong 10 phút bay lượn.

Cái nào đạt hai điều kiện đầu ở tầm xa nhất thì thắng, bất kể điểm số ở trên.

## 6. Cấu hình đề xuất

Khối lượng và điện là tổng giá trị trong các nguồn đã dẫn, không gồm cáp và giá đỡ.

Radar là cảm biến chính; lidar là cảm biến phụ. VOXL 2 tính 16 g và 7 W (suy từ cấu hình cũ:
281 g − 265 g và 13,5 W − 6,5 W).

| | Radar chính | Lidar phụ | Tính toán | Khối lượng | Điện | Dùng khi |
|---|---|---|---|---|---|---|
| **R0. Chỉ radar (mốc so sánh)** | 3 µSharp Patch hướng trước, trái, phải [27] | không | VOXL 2 [24] | khoảng 145 g (43 g mỗi Patch có vỏ) | khoảng 10,75 W (1,25 W mỗi Patch) | đo riêng hiệu năng radar; phủ 3 × 50° |
| **R1. Radar chính, lidar phụ** | như R0 | Livox Mid-360 [8] | VOXL 2 | khoảng 410 g | khoảng 17,25 W | cấu hình đề xuất: radar ra `OBSTACLE_DISTANCE`, lidar bù 360° và vật mảnh khi trời quang |
| **R2. Như R1, radar R21-1** | 2 R21-1 hướng trước và bên [13] | Mid-360 | VOXL 2 | khoảng 491 g | khoảng 19,5 W | phương án mốc; tầm 27 m, hẹp ±15° |
| **R3. Radar HS77G hoặc bo AWR2944** | 1–3 khối | Mid-360 | VOXL 2 hoặc Pi | chưa tính được (khối lượng radar chưa có nguồn) | chưa tính được | sau khi đo ở mục 7, nếu thắng Patch |

- **Vai trò của lidar phụ:** (1) phủ 360° và hướng ngoài vùng nhìn của radar; (2) vật mảnh, cột,
  cành khi trời quang nhờ phân giải góc; (3) đối chiếu để lọc báo giả của radar, đặc biệt mặt đất khi
  bay thấp. Khi radar và lidar bất đồng, lấy khoảng cách gần hơn (đề xuất của tôi) và ghi cả hai.
- **Nếu cần bay trên 12 m/s:** lidar nhìn trước Livox Avia [9] (498 g, 8–9 W) là cách duy nhất có
  số tầm 190 m với vật 10%; nó làm lidar phụ nặng hơn radar chính, nên chỉ thêm khi radar đo ra
  không đủ tầm.

- **Mid-360 có vùng nhìn lệch lên trên** (−7° đến +52°): hợp với bay ngang, không nhìn được dưới
  −7°. Phần dưới chân UAV cần cảm biến khác khi hạ cánh.
- **Tầm của cấu hình bị giới hạn bởi radar chính:** 27 m với R21-1 [13], tức khoảng 10 m/s theo bảng 2.2.
  Với µSharp Patch (120 m hãng ghi) hay HS77G (80 m) chưa có số đo với vật nhỏ, nên chưa tính tốc
  độ. Vùng nhìn hẹp (Patch 50°, R21-1 ±15°) khiến hai radar chỉ phủ hướng bay, không phủ 360°.
- **Giao diện PX4:** chuyển điểm sang 72 ô 5° trong hành lang độ cao và gửi `OBSTACLE_DISTANCE`
  (mục 3.2) cho chế độ Position; với bay tự động thì viết chế độ tự tạo setpoint bằng thư viện
  PX4 ROS 2 Interface [7].

## 7. Cần đo trước khi chọn

Các số dưới đây chưa có nguồn, cần bãi thử:
1. **Tầm phát hiện vật mảnh** (dây 1–3 mm, cột 5–10 cm, cành cây) bằng Mid-360 và bằng radar, ở
   10, 20, 40, 70 m. Đây là số quyết định tốc độ bay tối đa (bảng 2.2).
2. **Mặt đất với radar bay thấp:** bài báo [1] không nêu cách loại clutter; cần biết có bao nhiêu
   điểm mặt đất lọt vào hành lang độ cao và cách loại chúng.
3. **Sương, bụi, mưa** với cùng cấu hình: lidar mất bao nhiêu tầm, radar còn bao nhiêu (ở 24 GHz
   chưa có số).
4. **Độ trễ đầu cuối thực** từ log PX4 để đặt `CP_DELAY` (mặc định 0,4 s [5]).
5. **So sánh trực tiếp các radar ứng viên** (HS77G bản 80 m, µSharp Patch, MR72, bo AWR2944) với
   cùng bộ vật cản (tường, cột 10 cm, dây 1 mm và 12 mm, cành cây) ở cùng khoảng cách. Đây là cách
   duy nhất để có số đo độc lập cho mục 5.3.
6. **Xác nhận µSharp Patch còn bán** và hỏi hãng về RCS của vật cho tầm 120 m; **datasheet hãng**
   của UAV-R121-1 (120 m, 170 g?), MR77 (vùng nhìn, tần số, giao tiếp), HS77G và MR72 (tầm, vùng
   nhìn, khối lượng).

## Phụ lục: radar tầm xa cho vật bay (phạm vi 1 km cũ)

Nếu sau này cần phát hiện máy bay hoặc drone khác ở tầm xa: **Echodyne EchoFlight** [15][16] là
ứng viên duy nhất tôi tìm được có datasheet hiện hành. Băng K 24,45–24,65 GHz; Phantom 4 trên
750 m, Matrice 600 trên 1 km, Cessna trên 2 km; 817 g, 45 W, 18,7 × 12 × 4 cm; vùng nhìn 120° × 80°;
độ chính xác bám < 1° × < 1,5°; cập nhật 10 Hz; Ethernet Gigabit. Hãng không công bố tầm với vật cản tĩnh (dây, cột, cây). Là sản phẩm có
thật: trang hãng [15], datasheet bản 6JL © 2026 [16], bản ghi FCC ID 2ANLB-MESADAA00051 từ
2018 [17].

## Nguồn

Trạng thái truy cập kiểm tra ngày 2026-10-21, trừ khi ghi khác. "Mở được" là công cụ tự động
truy cập thành công. Mục nào ghi "file trong repo" thì đọc ở commit `a69ac4b5ff`.

1. Malle N. H., Ebeid E. *Omnidirectional Solid-State mmWave Radar Perception for UAV Power Line
   Collision Avoidance*. arXiv:2602.03229, 2026; nhận đăng ở IEEE ICRA 2026.
   https://arxiv.org/abs/2602.03229 (HTML: https://arxiv.org/html/2602.03229v1). Mở được.
2. Video trình bày ICRA'26 của bài báo [1] (cùng tiêu đề).
   https://www.youtube.com/watch?v=rJW3eEC-5Ao. Mở được nhưng công cụ chỉ đọc được tiêu đề;
   **không có số liệu nào trong tài liệu này lấy từ video**, chỉ lấy từ bài báo [1].
3. PX4. *Collision Prevention* (file trong repo).
   `docs/en/computer_vision/collision_prevention.md`.
4. PX4 (file trong repo). `msg/ObstacleDistance.msg` (72 phần tử `distances`, giá trị
   `MAV_DISTANCE_SENSOR_RADAR`); `src/lib/collision_prevention/CollisionPrevention.hpp`
   (`BIN_COUNT` = 72, `BIN_SIZE` = 5°); `src/modules/mavlink/mavlink_receiver.cpp` (chỉ xử lý
   `OBSTACLE_DISTANCE`); `src/modules/mavlink/mavlink/message_definitions/v1.0/ardupilotmega.xml`
   (định nghĩa `OBSTACLE_DISTANCE_3D`, không có chỗ nhận).
5. PX4 (file trong repo). Tham số mặc định: `CP_DELAY` 0,4 s, `CP_GUIDE_ANG` 30°, `CP_DIST` −1
   (tắt) trong `src/lib/collision_prevention/collisionprevention_params.yaml`; `MPC_ACC_HOR`
   3,0 m/s² trong `src/modules/mc_pos_control/multicopter_autonomous_params.yaml`;
   `MPC_XY_VEL_MAX` 12 m/s trong
   `src/modules/mc_pos_control/multicopter_position_control_limits_params.yaml`.
6. PX4 (file trong repo). *Lightware SF45/B Rotary Lidar* `docs/en/sensor/sf45_rotating_lidar.md`;
   danh mục cảm biến đo khoảng cách (gồm Ainstein US-D1, tầm khoảng 50 m)
   `docs/en/sensor/rangefinders.md`.
7. PX4 (file trong repo). *Path Planning Interface* (đã gỡ từ v1.15)
   `docs/en/computer_vision/path_planning_interface.md`; thư viện PX4 ROS 2 Interface
   `docs/en/ros2/px4_ros2_interface_lib.md` (chỉ đọc tên file; nội dung chưa đọc).
8. Livox. *Mid-360 Specs*. https://www.livoxtech.com/mid-360/specs. Mở được.
9. Livox. *AVIA Specs*. https://www.livoxtech.com/avia/specs. Mở được.
10. Ouster. *OS1 Mid-Range High-Resolution Imaging Lidar, hardware version Rev 7.0*, datasheet
    REV 02/2025 (PDF).
    https://data.ouster.io/downloads/datasheets/datasheet-rev7-v3p1-os1.pdf. Mở được; tôi trích
    văn bản bằng `pdftotext`.
11. Texas Instruments. *IWR6843AOP* (trang sản phẩm).
    https://www.ti.com/product/IWR6843AOP. Mở được.
12. Texas Instruments. *TIDUEZ2: obstacle detection reference design using AOP mmWave sensor*
    (chuỗi xử lý trên DSP AWR1843AOP).
    https://www.ti.com/document-viewer/lit/html/TIDUEZ2/GUID-886967C4-B9FA-4F2E-8549-CFC6F61291DC.
    Mở được; chỉ phần sơ đồ phần mềm (mục 2.1.1) có trong nội dung đọc được.
13. MicroBrain Intelligent. *UAV-R21-1 obstacle avoidance radar* (trang sản phẩm của hãng).
    https://en.microbrain.com.cn/uav-drone-obstacle-radar. Mở được.
14. SPH Engineering. *MR77 obstacle detection radar for DJI M300/M350 RTK* (cửa hàng của hãng).
    https://shop.sphengineering.com/products/mr77. Mở được.
15. Echodyne. *EchoFlight®: airborne radar* (trang sản phẩm).
    https://www.echodyne.com/radar-systems/echoflight. Mở được (kiểm tra 2026-10-20).
16. Echodyne. *EchoFlight® 4D Airborne Radar – Technical Specification*, bản 6JL, © 2026 (PDF).
    https://cdn.prod.website-files.com/69148cb0d4c3689c365788f0/6a55592ec7c6a2c9d38a1fa4_TS-EchoFlight-Echodyne_6JL.pdf.
    Mở được (kiểm tra 2026-10-20).
17. FCC ID 2ANLB-MESADAA00051, "Airborne detect and avoid radar", Echodyne Corp., cấp lần đầu
    19/07/2018. https://fccid.io/2ANLB-MESADAA00051. Mở được (kiểm tra 2026-10-20).
18. InnoSenT. *iSYS-4001 radar system* (trang sản phẩm).
    https://innosent.de/en/radar-systems/isys-4001-radarsystem. Mở được (kiểm tra 2026-10-20).
19. *Weather Influence and Classification with Automotive Lidar Sensors*. arXiv:1906.07675, 2019
    (VLP-16 và Valeo Scala, khoảng 905 nm; buồng sương tầm nhìn 20–60 m, mưa 55 mm/h).
    https://ar5iv.labs.arxiv.org/html/1906.07675. Mở được; tôi chưa xác nhận danh sách tác giả
    trong lần đọc này.
20. Kutila M., Pyykönen P., Holzhüter H., Colomb M., Duthon P. *Automotive LiDAR performance
    verification in fog and rain*. IEEE ITSC 2018, tr. 1695–1701. DOI 10.1109/ITSC.2018.8569624.
    https://cris.vtt.fi/en/publications/automotive-lidar-performance-verification-in-fog-and-rain/.
    Mở được; chỉ đọc được tóm tắt.
21. Zhang Y., Carballo A., Yang H., Takeda K. *Perception and Sensing for Autonomous Vehicles
    Under Adverse Weather Conditions: A Survey*. ISPRS J. Photogramm. Remote Sens. 196 (2023)
    146–177; arXiv:2112.08936. https://arxiv.org/abs/2112.08936. Mở được; tôi đọc số liệu từ
    PDF đã tải về (mục 3.2 và 3.3 của bài).
22. *LiDAR-Based Real-Time Detection and Modeling of Power Lines for Unmanned Aerial Vehicles*
    (Velodyne VLP-16 trên hexacopter STORK, ODROID-XU4). PMC6515251.
    https://pmc.ncbi.nlm.nih.gov/articles/PMC6515251/. Mở được; tôi chưa xác nhận danh sách tác
    giả và tạp chí trong lần đọc này.
23. NVIDIA. *Jetson Orin Nano Super Developer Kit* (trang sản phẩm).
    https://www.nvidia.com/en-us/autonomous-machines/embedded-systems/jetson-orin/nano-super-developer-kit/.
    Mở được.
24. ModalAI. *VOXL 2* (trang sản phẩm). https://www.modalai.com/products/voxl-2. Mở được.
25. Mandaokar D., Rinner B. *RADAR Perception for Dynamic Obstacle Avoidance onboard small-scale
    Quadrotor UAVs*. arXiv:2608.01855, nộp 03/08/2026 (chưa rõ nơi đăng).
    https://arxiv.org/abs/2608.01855 (HTML: https://arxiv.org/html/2608.01855). Mở được.
26. DJI Enterprise. *Matrice 400 Specs*. https://enterprise.dji.com/matrice-400/specs. Mở được;
    các số đo dây và tầm lidar do DJI công bố.
27. Aerotenna. *µSharp Patch Collision Avoidance Radar*, datasheet 09/02/2018, và *µSharp – 360°
    Sense-and-Avoid Radar*, datasheet 23/10/2017 (PDF, lưu trên Sundance).
    https://www.sundance.com/wp-content/uploads/docs/AerotennauSharppatchDatasheet.pdf và
    https://www.sundance.com/wp-content/uploads/docs/AerotennauSharp360Datasheet.pdf. Mở được; tôi
    trích văn bản bằng `pdftotext`.
28. Texas Instruments. *AWR1843AOP* (trang sản phẩm). https://www.ti.com/product/AWR1843AOP.
    Mở được.
29. Texas Instruments. *AWR2944* (trang sản phẩm). https://www.ti.com/product/AWR2944. Mở được.
30. ArduPilot. *Hexsoon 77G MWW Radar*
    https://ardupilot.org/copter/docs/common-rangefinder-hexsoon-radar.html; *Nanoradar MR72*
    https://ardupilot.org/copter/docs/common-rangefinder-mr72.html; danh mục cảm biến đo khoảng
    cách (Omnidirectional Proximity Rangefinders và Unidirectional Rangefinders)
    https://ardupilot.org/copter/docs/common-rangefinder-landingpage.html. Mở được.
31. Hexsoon (tài khoản của hãng). *HS 77G MMW enables all-weather operation*, diễn đàn ArduPilot,
    07/11/2025. https://discuss.ardupilot.org/t/hs-77g-mmw-enables-all-weather-operation/140730.
    Mở được; bài đăng của hãng, không phải đo độc lập.
32. Ferntech NZ. *DJI Matrice 4 Obstacle Sensing Module* (nhà bán lẻ).
    https://www.ferntech.co.nz/dji-matrice-4-obstacle-sensing-module. Mở được; thông số do nhà bán
    lẻ nêu. Trang DJI Store của sản phẩm này trả về HTTP 404 khi tôi mở.
33. Ainstein. *Ainstein's Obstacle Avoidance Radar: coming soon* (thông báo OA-Lite và OA-Pro).
    https://ainstein.ai/ainsteins-obstacle-avoidance-radar-coming-soon/. Mở được; không có thông
    số, trang không ghi ngày đăng.
34. Sundance (phân phối Ainstein). *US-D1 Long Range Radar Altimeter*.
    https://www.sundance.com/us-d1-2022/. Mở được.
35. PX4 (file trong repo). Driver radar duy nhất:
    `src/drivers/distance_sensor/ulanding_radar/` (Aerotenna uLanding, radar đo độ cao);
    `docs/en/sensor/ulanding_radar.md`.
