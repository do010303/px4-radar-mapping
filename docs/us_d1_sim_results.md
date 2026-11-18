# Kết quả mô phỏng 3 × Ainstein US-D1 với collision prevention của PX4

*Ghi 2026-10-27. Mô hình cảm biến là giả định của tôi ở những chỗ datasheet không nói (xem
`profiles/us_d1.yaml`); kết quả là của mô phỏng, chưa phải của radar thật.*

## Cách chạy lại

    scripts/make_us_d1_x3.sh                                   # 3 mô hình radar 25 Hz
    CP_DIST=5 US_D1=3 PX4_SIM_MODEL=gz_x500_us_d1_up15 PX4_GZ_WORLD=us_d1_sides ./run_sitl.sh
    scripts/cp_test.py --stick 600 --wall-x 27.95              # bay vào cột ở 28 m (Position mode)

Xem trực quan: bỏ `HEADLESS=1` thì có Gazebo (3 khung FOV màu vàng) và RViz (`rviz/us_d1.rviz`:
khung FOV, số đo PX4 nhận từ từng radar, bản đồ 72 ô của CP). `scripts/us_d1_viz.py` đọc MAVLink cổng 14540
nên không tốn CPU của mô phỏng.

## Kết quả (một lần chạy mỗi dòng, `CP_DIST` = 5 m, world `us_d1_sides`)

| Cấu hình | Tường 40 m, gậy 100% | Cột 10 cm ở 28 m, gậy 60% |
|---|---|---|
| Radar nghiêng lên 15° | dừng ở 6,1 m (vào sát 4,5 m), 7,9 m/s | dừng ở 2,6–3,7 m (hai lần chạy), 2,6–2,8 m/s |
| Radar nằm ngang | chưa chạy | **0,8 m, chưa dừng hẳn**, 2,5 m/s |

## Nhận xét

- **Cột mảnh phát hiện quá muộn.** Radar giữa báo tường ở phía sau (40 m, rồi giảm dần) và chỉ báo cột
  khi còn khoảng 4 m. Hai nguyên nhân (đều của mô hình): US-D1 chọn return mạnh nhất, tường lớn
  át cột; và RCS tính theo diện tích tia chiếu tới, không theo RCS thật của trụ mảnh nên cột bị
  đánh giá thấp. Cần đo thật, và cần mô hình dây/cột (xem `paper_malle2026_notes.md`).
- **Nghiêng lên 15° loại được nhiễu đất.** Radar nằm ngang cho 16% số đọc là đất (dưới 20 m, không
  phải cột); radar nghiêng cho 0%. Giá phải trả: mất 15° phía dưới.
- **Vật ở hai bên làm UAV chậm lại dù không chắn đường.** Hộp 2 m ở lệch 9 m (bearing ±26,6°) nằm
  trong búp radar trái/phải, đọc 8 m; CP coi cả búp 43° là "có vật ở 8 m", nên tốc độ tụt từ
  2,5 xuống 0,5 m/s trên đoạn đó. Đây là hệ quả của việc chỉ có một khoảng cách cho cả búp.
- Chưa kiểm khe 2° giữa búp trái và búp giữa (cột 0,5 m ở bearing 22,5°, 30 m): world đã có
  (`gap_post`), bài thử chưa nhắm vào nó.
