dataTable = readtable('vdt2026_uav_data.csv');
numSamples = height(dataTable);             % Đếm dòng data không tính dòng không chứa dữ liệu

timeArray        = dataTable.time;          % Thời gian (giây)
gyroZArray       = dataTable.gyroZ;         % Tốc độ góc trục Z (rad/s)
magXArray        = dataTable.magX;          % Từ kế trục X
magYArray        = dataTable.magY;          % Từ kế trục Y
gnssHeadArray    = dataTable.gnssHead;      % Góc hướng GNSS (rad)
isNewGnssArray   = dataTable.isNewGnss;     % Cờ báo có gói tin GNSS mới (1/0)
isGoodGnssArray  = dataTable.isGoodGnss;    % Cờ báo chất lượng GNSS tốt (1/0)
gnssHeadAccArray = dataTable.gnssHeadAcc;   % Độ chính xác động của GNSS (rad)

% Khởi tạo đối tượng từ Class định nghĩa dữ liệu
yawNavFilter = YawNav();

% Khởi tạo các mảng chứa kết quả đầu ra (Pre-allocation bộ nhớ)
yawEst        = zeros(numSamples, 1);
gyroBiasEst   = zeros(numSamples, 1);
isUseGyroLog  = zeros(numSamples, 1);
isUseMagLog   = zeros(numSamples, 1);
isUseGnssLog  = zeros(numSamples, 1);

% Vòng lặp For duyệt qua từng chu kỳ hoạt động động học của hệ thống
for i = 1:numSamples
    % 1. Đẩy dữ liệu thô vào chu kỳ xử lý và tính ma trận Kalman
    yawNavFilter.pushNewData(timeArray(i), gyroZArray(i), magXArray(i), magYArray(i), ...
                             gnssHeadArray(i), isNewGnssArray(i), isGoodGnssArray(i), gnssHeadAccArray(i));
                         
    % 2. Trích xuất giá trị trạng thái ước lượng từ các phương thức Public
    [yaw, gyroBias, isUseGyro, isUseMag, isUseGnss] = yawNavFilter.getEst();
    
    % 3. Lưu trữ dữ liệu lịch sử phục vụ cho việc vẽ đồ thị
    yawEst(i)        = yaw;
    gyroBiasEst(i)   = gyroBias;
    isUseGyroLog(i)  = isUseGyro;
    isUseMagLog(i)   = isUseMag;
    isUseGnssLog(i)  = isUseGnss;
end

% Quy đổi hằng số từ Radian sang Degrees
RAD_TO_DEG = 180 / pi;

Fontsize = 15;
% --- HÌNH 1: ĐỒ THỊ GÓC YAW QUY ĐỔI VỀ [0 ĐẾN 360 ĐỘ] ---
fig1 = figure(1);

% Chuyển đổi toàn bộ dữ liệu góc đo đạc và ước lượng sang thang [0, 360] độ
yawEstDeg  = mod(yawEst * RAD_TO_DEG, 360); % Góc ước lượng
yawGnssDeg = mod(gnssHeadArray * RAD_TO_DEG, 360); % Góc GNSS đo được
yawMagDeg  = mod(-atan2(magYArray, magXArray) * RAD_TO_DEG, 360); % Tính góc thô từ kế

plot(timeArray, yawMagDeg, 'g-', 'LineWidth', 2); hold on;
plot(timeArray, yawGnssDeg, 'b-.', 'LineWidth', 1.5);
plot(timeArray, yawEstDeg, 'r-.', 'LineWidth', 1.5);

grid on;
xlim([timeArray(1), timeArray(end)]);
ylim([0, 360]);
title('Figure 1: Heading Angle Estimation','Interpreter','latex', 'FontSize',Fontsize);
xlabel('Time (s)','Interpreter','latex','FontSize',Fontsize);
ylabel('Heading Angle (degree)','Interpreter','latex','FontSize',Fontsize);
legend('Raw Magnetometer', 'Raw GNSS Heading', 'Estimated Yaw (YawNav)','Interpreter','latex','FontSize',6);
%exportgraphics(fig1, 'Hinh1_YawComparison.pdf', 'ContentType', 'image', 'Resolution', 600);

% --- HÌNH 2: ĐỒ THỊ GYRO BIAS ƯỚC LƯỢNG (ĐƠN VỊ ĐỘ/S) ---
fig2 = figure(2);

% Đổi đơn vị của sai số trôi từ rad/s sang độ/s 
gyroBiasEstDeg = gyroBiasEst * RAD_TO_DEG;

plot(timeArray, gyroBiasEstDeg, 'm-', 'LineWidth', 2);
grid on;
xlim([timeArray(1), timeArray(end)]);
title('Figure 2: Gyro Bias axis Z','Interpreter','latex','FontSize',Fontsize);
xlabel('Time (s)','Interpreter','latex','FontSize',Fontsize);
ylabel('Gyro Bias (degree/s)','Interpreter','latex','FontSize',Fontsize);
legend('Estimated Gyro Bias','Interpreter','latex', 'FontSize',8,'Location', 'best');
%exportgraphics(fig2, 'Hinh2_GyroBias.pdf', 'ContentType', 'image', 'Resolution', 600);

% --- HÌNH 3: CÁC CỜ TRẠNG THÁI SỬ DỤNG CẢM BIẾN (USEGYRO, USEMAGNET, USEGNSS) ---
fig3 = figure(3);

plot(timeArray, isUseGyroLog + 2.4, 'Color', [0.85 0.33 0.1], 'LineWidth', 2); hold on;
plot(timeArray, isUseMagLog + 1.2, 'Color', [0.47 0.67 0.19], 'LineWidth', 2);
plot(timeArray, isUseGnssLog, 'Color', [0 0.45 0.74], 'LineWidth', 2);

grid on;
xlim([timeArray(1), timeArray(end)]);
ylim([-0.5, 4]);
set(gca, 'YTick', [0 1 1.2 2.2 2.4 3.4], 'YTickLabel', {'REJECT', 'USE GNSS', 'REJECT', 'USE MAGNET', 'REJECT', 'USE GYRO'},'FontSize', 7);
title('Figure 3: Health Flag','Interpreter','latex','FontSize',Fontsize);
xlabel('Time (s)','Interpreter','latex','FontSize',Fontsize);
ylabel('State','Interpreter','latex','FontSize',Fontsize);
legend('isUseGyro', 'isUseMag', 'isUseGnss', 'Location', 'northeast','Interpreter','latex','FontSize',7);
%exportgraphics(fig3, 'Hinh3_HealthFlags.pdf', 'ContentType', 'image', 'Resolution', 600);