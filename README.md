# UAV-SensorFusion
# VDT 2026 UAV Sensor Fusion

Ước lượng góc hướng cho trạm thông tin mặt đất bắt bám UAV bằng bộ lọc Kalman kết hợp gyro, cảm biến từ trường và GNSS hai anten.

## 1. Tổng quan

Dự án xây dựng hệ thống ước lượng góc hướng `yaw` của trạm mặt đất, đồng thời ước lượng độ lệch `gyroBias` của gyro trục Z. Thuật toán cần lựa chọn dữ liệu cảm biến phù hợp theo từng chu kỳ, loại bỏ dữ liệu bất thường và xuất các cờ cho biết cảm biến nào đang được sử dụng.

Ba nguồn dữ liệu có đặc điểm bổ sung cho nhau: gyro cung cấp tốc độ quay để dự đoán chuyển động, cảm biến từ cung cấp góc hướng từ trường, còn GNSS Moving Base Dual Antenna cung cấp phép đo góc hướng với tần số thấp hơn.

README được tổng hợp từ hai tệp:

- `[2026-05-24] VDT 2026 UAV project.docx`: đề bài, giao diện đầu ra, yêu cầu triển khai và tiêu chí chấm điểm. Tác giả đề bài: Vũ Văn Dũng, Trung tâm UAV, VHT.
- `vdt2026_uav_data.csv`: dữ liệu cảm biến dùng để phát triển và kiểm tra thuật toán.

**Trạng thái:** hai tệp đầu vào gồm đề bài và dữ liệu, chưa có mã nguồn MATLAB hoặc kết quả chạy bộ lọc. Mô hình và cấu trúc mã nguồn ở phần dưới là gợi ý triển khai. Các thống kê dữ liệu được tính trực tiếp từ CSV.

## 2. Phạm vi và giả định

Theo đề bài:

- Roll và pitch của trạm mặt đất đủ nhỏ để bỏ qua.
- Góc declination và inclination của từ trường tại khu vực thử nghiệm, theo mô hình WMM, không đáng kể.
- Tốc độ quay thực tế của trạm không vượt quá `300 độ/s`, tương đương khoảng `5,236 rad/s`.
- Động học góc hướng được xấp xỉ bằng `d(yaw)/dt = r`, với `r` là tốc độ quay quanh trục Z.
- Góc hướng từ cảm biến từ được tính bằng `yawMag = -atan2(magY, magX)`.

| Cảm biến | Tần số theo đề bài | Sai số theo đề bài | Vai trò trong hệ thống |
| --- | --- | --- | --- |
| Gyro trục Z | Khoảng 20 Hz | Khoảng 200 độ/giờ | Dự đoán góc hướng theo tốc độ quay |
| Cảm biến từ trường | Khoảng 20 Hz | Khoảng 3–5 độ | Cung cấp phép đo góc hướng từ trường |
| GNSS hai anten | Khoảng 4–5 Hz | Khoảng 0,5–1 độ | Cung cấp phép đo góc hướng GNSS |

Các mức sai số trên là thông tin của đề bài. Không mặc định chúng là độ lệch chuẩn để đưa trực tiếp vào ma trận nhiễu của bộ lọc.

## 3. Dữ liệu đầu vào

### Cấu trúc CSV

Tệp dùng dấu phẩy để phân cách và dấu chấm cho phần thập phân. Mỗi dòng dữ liệu tương ứng một chu kỳ hoạt động của trạm.

```csv
time,gyroZ,magX,magY,gnssHead,isNewGnss,isGoodGnss,gnssHeadAcc
0,-0.00029,0.97,-0.228,0.214,1,1,0.017
```

| Cột | Đơn vị | Ý nghĩa |
| --- | --- | --- |
| `time` | s | Thời gian của đồng hồ hệ thống |
| `gyroZ` | rad/s | Giá trị đo gyro trục Z |
| `magX` | Không nêu trong đề bài | Thành phần từ trường trục X |
| `magY` | Không nêu trong đề bài | Thành phần từ trường trục Y |
| `gnssHead` | rad | Góc hướng đo bằng GNSS Moving Base Dual Antenna |
| `isNewGnss` | Cờ 0/1 | `1`: có dữ liệu GNSS mới; `0`: dữ liệu cũ |
| `isGoodGnss` | Cờ 0/1 | `1`: GNSS báo dữ liệu đáng tin; `0`: GNSS báo dữ liệu xấu |
| `gnssHeadAcc` | rad | Sai số góc hướng GNSS được cung cấp cùng dữ liệu |

### Thống kê của bộ dữ liệu

| Thuộc tính | Giá trị quan sát |
| --- | --- |
| Số dòng dữ liệu, không tính header | 12.000 |
| Số cột | 8 |
| Thời gian bắt đầu | 0 s |
| Thời gian kết thúc | 599,952 s, gần 10 phút |
| Khoảng cách giữa hai mẫu liên tiếp | 0,040–0,060 s |
| Trung vị khoảng lấy mẫu | 0,050 s, tương đương khoảng 20 Hz |
| Số dòng có dữ liệu GNSS mới | 2.950 |
| Trung vị khoảng thời gian giữa các lần GNSS mới | 0,200 s, tương đương khoảng 5 Hz |
| Số dòng GNSS mới và được cờ báo tốt | 2.900 |
| Số dòng GNSS mới và được cờ báo xấu | 50 |
| Số dòng có `isGoodGnss = 0`, gồm cả dữ liệu cũ | 200 |
| Ô trống hoặc giá trị số không hữu hạn | 0 |
| Thời gian không tăng hoặc dòng trùng hoàn toàn | 0 |

Dữ liệu không có cột góc hướng chuẩn độc lập (ground truth). Vì vậy, chưa thể tính sai số tuyệt đối hoặc RMSE so với góc hướng thực chỉ từ CSV này. GNSS cũng có thể lỗi nên không mặc định coi GNSS là ground truth.

### Một số đoạn cần chú ý khi kiểm thử

Các khoảng dưới đây là thời điểm mẫu đầu và mẫu cuối quan sát được, không phải nhãn lỗi đã được xác nhận.

| Khoảng thời gian | Quan sát trực tiếp | Nội dung cần kiểm tra |
| --- | --- | --- |
| 60,003–69,949 s | `gyroZ = 0` liên tục 200 mẫu | Phân biệt dữ liệu lỗi với trạng thái thực sự không quay |
| 119,947–129,947 s | `gyroZ = 0.08739` giữ nguyên trong 201 mẫu | Phát hiện tín hiệu bị giữ hằng dựa trên thời gian và đối chiếu cảm biến khác |
| 180,000–189,946 s | `gyroZ = 5.41012 rad/s`, vượt giới hạn 300 độ/s | Loại bỏ hoặc xử lý dữ liệu quá dải |
| 239,998–249,948 s | `isGoodGnss = 0` trong 200 dòng | Xử lý GNSS báo chất lượng xấu |
| Giữa 269,804 và 279,996 s | Hai lần GNSS mới cách nhau 10,192 s | Xử lý khoảng thiếu cập nhật GNSS |
| 359,996–369,950 s | `sqrt(magX^2 + magY^2) > 2` trong 200 mẫu | Phát hiện độ lớn từ trường lệch xa giá trị kỳ vọng gần 1 |

Mốc `magNorm > 2` ở bảng chỉ mô tả đoạn dữ liệu quan sát được. Ngưỡng chấp nhận cảm biến từ phải được lựa chọn và giải thích trong thiết kế thuật toán.

## 4. Đầu ra yêu cầu

Mỗi chu kỳ cần trả về ít nhất các trường sau:

| Trường | Đơn vị | Ý nghĩa |
| --- | --- | --- |
| `yaw` | rad | Góc hướng ước lượng |
| `gyroBias` | rad/s | Độ lệch ước lượng của gyro trục Z |
| `isUseGyro` | Cờ logic | Gyro có được sử dụng để fusion trong chu kỳ hay không |
| `isUseMag` | Cờ logic | Cảm biến từ có được sử dụng để fusion trong chu kỳ hay không |
| `isUseGnss` | Cờ logic | GNSS có được sử dụng để fusion trong chu kỳ hay không |

Có thể xuất thêm trường chẩn đoán như innovation, độ bất định hoặc lý do loại cảm biến. Quy ước các cờ sử dụng cần thống nhất với dữ liệu thực sự được chấp nhận trong từng chu kỳ.

## 5. Hướng triển khai thuật toán

### Tiền xử lý và kiểm tra cảm biến

- Tính `dt` từ chênh lệch `time` của hai mẫu liên tiếp vì chu kỳ lấy mẫu có dao động.
- Kiểm tra gyro quá dải, bằng 0 kéo dài hoặc giữ hằng bất thường. Giá trị 0 riêng lẻ có thể hợp lệ và cần được xem xét theo ngữ cảnh.
- Tính `magNorm = sqrt(magX^2 + magY^2)` và kiểm tra độ lệch so với 1 trước khi dùng góc hướng từ trường. Đề bài cho phép tự chọn ngưỡng.
- Xử lý mẫu GNSS mới theo `isNewGnss`, tránh cập nhật lặp lại cùng một phép đo ở các chu kỳ dữ liệu cũ.
- Kết hợp `isGoodGnss`, `gnssHeadAcc` và kiểm tra tính nhất quán với dự đoán. Theo đề bài, GNSS có thể báo tốt trong khi dữ liệu thực tế xấu.
- Duy trì trạng thái và thời gian bất thường của từng cảm biến để phát hiện tín hiệu bị giữ hằng, mất cập nhật và phục hồi.

### Mô hình Kalman gợi ý

Có thể bắt đầu với trạng thái gồm góc hướng và bias gyro:

```text
x = [yaw; gyroBias]
u = gyroZ

yaw(k+1)      = yaw(k) + (gyroZ(k) - gyroBias(k)) * dt
gyroBias(k+1) = gyroBias(k) + nhiễu biến thiên bias

F = [1, -dt;
     0,   1]
B = [dt; 0]
H = [1, 0]
```

Ở mỗi chu kỳ, dùng gyro hợp lệ để dự đoán trạng thái và hiệp phương sai. Sau đó cập nhật lần lượt bằng góc hướng từ cảm biến từ và/hoặc GNSS được chấp nhận. Ma trận nhiễu quá trình `Qd`, nhiễu đo `R`, hiệp phương sai khởi tạo và ngưỡng loại ngoại lai cần được thiết kế, tune và giải thích. Cũng cần xác định cách khởi tạo và tiếp tục ước lượng khi một cảm biến bị loại.

Các phép đo góc có chu kỳ nên sai lệch giữa phép đo và dự đoán cần được quy về khoảng `[-pi, pi)` trước khi cập nhật. Ví dụ bằng các hàm MATLAB cơ bản:

```matlab
innovation = mod(measurement - prediction + pi, 2*pi) - pi;
```

Nếu sử dụng `gnssHeadAcc` để xây dựng `R`, cần nêu rõ cách diễn giải trường này. Đề bài mô tả đây là sai số heading nhưng chưa xác định rõ nó là độ lệch chuẩn hay một chỉ số sai số khác.

Sơ đồ xử lý tổng quát:

```text
Mẫu CSV theo thời gian
    -> Kiểm tra thời gian và tình trạng cảm biến
    -> Dự đoán yaw và gyroBias
    -> Kiểm tra phép đo góc và cập nhật Kalman
    -> Xuất yaw, gyroBias và các cờ sử dụng cảm biến
    -> Lưu kết quả và vẽ đồ thị
```

## 6. Yêu cầu mã nguồn và cách sử dụng

### Môi trường

- MATLAB R2020b hoặc mã nguồn tương thích đầy đủ với R2020b.
- Không sử dụng hàm chỉ xuất hiện ở phiên bản MATLAB mới hơn.
- Đề bài không chỉ định toolbox bắt buộc. Nếu triển khai cần toolbox, phải ghi rõ phụ thuộc trong hướng dẫn chạy.
- Dùng Draw.io để vẽ sơ đồ khối trong tài liệu thiết kế thuật toán.

### Cấu trúc tệp đề xuất

Tên hai tệp `.m` dưới đây là gợi ý; chúng chưa có trong hai tệp đầu vào.

```text
sensor-fusion/
    README.md
    [2026-05-24] VDT 2026 UAV project.docx
    vdt2026_uav_data.csv
    YawNav.m
    run_yaw_nav.m
    algorithm_design.<định dạng tài liệu được chọn>
```

Theo yêu cầu đề bài, phần mã MATLAB bàn giao gồm đúng hai tệp `.m`:

1. **Tệp định nghĩa class**, ví dụ `YawNav.m`: ngoài constructor, chỉ có đúng hai phương thức public là `pushNewData()` và `getEst()`. Các hàm hỗ trợ có thể đặt private. Đề bài chưa quy định chi tiết tham số và kiểu trả về của hai phương thức.
2. **Tệp script chạy**, ví dụ `run_yaw_nav.m`: chia thành đúng ba section MATLAB bằng `%%`:
   - Đọc CSV.
   - Dùng vòng `for` gọi `pushNewData()` và `getEst()` theo từng chu kỳ.
   - Vẽ kết quả ước lượng để so sánh.

Áp dụng coding convention theo yêu cầu đề bài, tham chiếu chuẩn Google C++: class như `YawNav`, biến như `yawEst`, hàm như `pushNewData()`, hằng số như `MEAS_NOISE_MAG`.

### Đọc và kiểm tra dữ liệu trên MATLAB

Đặt CSV trong Current Folder của MATLAB rồi chạy:

```matlab
data = readtable('vdt2026_uav_data.csv');
numSamples = height(data);
dt = diff(data.time);
yawMag = mod(-atan2(data.magY, data.magX), 2*pi);
```

Sau khi hoàn thiện hai tệp `.m`, đặt chúng cùng thư mục CSV và chạy script xử lý đã chọn. Với cấu trúc gợi ý ở trên:

```matlab
run('run_yaw_nav.m');
```

Lệnh chạy script chỉ sử dụng được sau khi đã bổ sung phần triển khai MATLAB.

## 7. Kết quả cần trình bày và hồ sơ bàn giao

### Ba đồ thị bắt buộc

Tất cả đồ thị dùng trục hoành là thời gian tính bằng giây và có đầy đủ title, legend.

| Hình | Nội dung | Đơn vị và cách thể hiện |
| --- | --- | --- |
| 1 | Góc hướng từ cảm biến từ, GNSS và bộ ước lượng | Độ, biểu diễn trong khoảng 0–360 độ |
| 2 | `gyroBias` ước lượng | Độ/s |
| 3 | Các cờ sử dụng gyro, cảm biến từ và GNSS | Giá trị 0/1 theo thời gian |

Đầu ra thuật toán vẫn dùng radian và radian/giây. Chỉ chuyển đổi sang độ và độ/giây khi hiển thị. Có thể dùng `mod(yaw * 180/pi, 360)` cho hình góc hướng và `gyroBias * 180/pi` cho hình bias. Hình 3 sử dụng các trường `isUseGyro`, `isUseMag`, `isUseGnss` tương ứng với các nhãn useGyro, useMagnet, useGnss trong đề bài.

### Tài liệu thiết kế thuật toán

Bên cạnh hai tệp `.m`, cần nộp một tài liệu thiết kế gồm:

1. Đầu vào và đầu ra của bài toán.
2. Các công thức toán học của thuật toán.
3. Mục đích của các hàm trong class.
4. Sơ đồ khối thuật toán được vẽ bằng Draw.io.
5. Kết quả chạy kèm đủ ba hình yêu cầu.
6. Đánh giá, nhận xét và đề xuất cải tiến.

Khi đánh giá, cần xem xét tính liên tục của góc hướng khi qua mốc 0/360 độ, khả năng ước lượng bias, phản ứng khi cảm biến bất thường và khả năng phục hồi sau khi dữ liệu tốt trở lại. Các chỉ số sai số so với một cảm biến phải ghi rõ cảm biến tham chiếu và điều kiện dữ liệu được chấp nhận.

### Tiêu chí chấm điểm theo đề bài

| Hạng mục | Điểm |
| --- | ---: |
| Tiền xử lý dữ liệu: gyro 1,5; cảm biến từ 1,5; phần khác 1 | 4,0 |
| Bộ lọc Kalman: mô hình 1; predict 1; update 1; xử lý bất thường 1 | 4,0 |
| Tune tham số hợp lý và kết quả đạt yêu cầu | 0,5 |
| Mã nguồn sạch, lập trình hướng đối tượng | 1,0 |
| Tài liệu trình bày tốt | 0,5 |
| **Tổng** | **10,0** |
