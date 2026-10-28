classdef YawNav < handle

    % =====================================================================
    % PHẦN PRIVATE
    % =====================================================================

    properties (Access = private)
        % Khai báo các hằng số ngưỡng theo cấu hình phần cứng

        MAX_GYRO_RATE = 5.2;          % Tốc độ tối đa 300 độ/s quy ra rad/s (300 * pi / 180) ~ 5.236 rad, chọn 5.2 rad
        MAG_NORM_TOLERANCE = 0.1;     % Ngưỡng sai số cho phép của từ trường (so với 1) 
        MAX_TIME_STEP = 0.5;          % Ngưỡng giới hạn thời gian (giây) để phát hiện rớt gói tin
        MAX_GNSS_ACC = 0.025          % Sai số của GNSS trong khoảng 0.5 - 1 độ, tương đương 0.0087 rad - 0.025 rad

        % Các biến lưu trữ trạng thái của chu kỳ trước

        prevTime = NaN;               % Khởi tạo NaN để nhận biết vòng lặp đầu tiên

        % Các biến lưu trữ trạng thái cho LPF
        MAG_LPF_ALPHA = 0.15;         % Hệ số LPF
        magX_filt = NaN;
        magY_filt = NaN;

        % Các cờ trạng thái báo cho Kalman biết có được dùng cảm biến không

        isUseGyro = false;
        isUseMag = false;
        isUseGnss = false;
        isInitialized = false;        % Cờ đánh dấu đã khóa góc ban đầu chưa
        
        % Các biến tạm để giữ giá trị (sau này sẽ dùng cho Kalman)

        currentDt = 0;

        % Các biến trạng thái check lỗi data = const quá lâu

        STUCK_TIME_THRESHOLD = 0.5;  % Ngưỡng thời gian kẹt tối đa (0.5 giây = khoảng 10 chu kỳ), thoải mái cho tần số của GNSS 4-5Hz

        % --- Biến lưu trữ trạng thái để so sánh ---
        prevGyroZ = NaN;             % Lưu giá trị Gyro của chu kỳ trước
        prevMagX = NaN;             % Lưu giá trị MagX của chu kỳ trước
        prevMagY = NaN;             % Lưu giá trị MagY của chu kỳ trước
        prevGnss = NaN;             % Lưu giá trị GNSS của chu kỳ trước
        stuckGyroTime = 0;           % Bộ đếm thời gian bị kẹt cho Gyro
        stuckMagTime = 0;           % Bộ đếm thời gian bị kẹt cho Mag
        stuckGnssTime = 0;           % Bộ đếm thời gian bị kẹt cho GNSS
        

        % Khởi tạo các ma trận cho Kalman Filter

        x = [0 ; 0];
        P = eye(2) * 0.1;
        Q = [1e-3, 0; 0, 1e-6];
        Rmag = (5*pi/180)^2; % Mag sai số khoảng 3-5 độ,
        

        A = [0 -1 ; 0 0];
        B = [1 ; 0];
        H = [1 0];

    end

    % =====================================================================
    % PHẦN PUBLIC 
    % =====================================================================

    methods (Access = public)
        % ==================KALMAN FILTER==================================
        function obj = YawNav()
            % -----------------
        end
        
        % ===================pushNewData===================================

        function pushNewData(obj, time, gyroZ, magX, magY, gnssHead, isNewGnss, isGoodGnss, gnssHeadAcc)
            
            % TIỀN XỬ LÝ THỜI GIAN ---
            obj.currentDt = obj.preprocessTime(time);
            
            % Nếu dt <= 0 (lỗi data hoặc lần chạy đầu tiên), ngắt không làm gì tiếp
            if obj.currentDt <= 0
                return; 
            end
            
            % TIỀN XỬ LÝ CÁC CẢM BIẾN ---
            obj.preprocessGyro(gyroZ);
            obj.preprocessMagnet(magX, magY);
            obj.preprocessGnss(isNewGnss, isGoodGnss, gnssHeadAcc);

            % Khởi tạo x và P
            if ~obj.isInitialized
                % Đợi cho đến khi chu kỳ có dữ liệu Magnet sạch đầu tiên xuất hiện
                if obj.isUseMag
                    % Nạp thẳng góc Từ kế (đã qua LPF) làm trạng thái Yaw gốc
                    obj.x(1) = -atan2(obj.magY_filt, obj.magX_filt);
                    obj.x(2) = 0;                  % Khởi tạo Gyro Bias bằng 0
                    obj.P = eye(2) * 0.1;          % Đặt P ban đầu nhỏ vì độ tự tin lúc này rất cao
                    obj.isInitialized = true;      % Khóa cờ khởi tạo lại
                    
                elseif obj.isUseGnss
                    % Phòng hờ nếu GNSS nhảy vào trước và báo tốt
                    obj.x(1) = gnssHead;
                    obj.x(2) = 0;
                    obj.P = eye(2) * 0.1;
                    obj.isInitialized = true;
                end
                
                % Nếu nhịp này vừa kích hoạt khởi tạo xong -> Thoát ngay 
                % để chu kỳ kế tiếp bắt đầu chạy Predict/Update dựa trên gốc này
                if obj.isInitialized
                    return;
                end
            end
            
            % Predict
            if obj.isUseGyro
                obj.predictKalman(gyroZ);
            else
                % Nếu Gyro hỏng/kẹt, vẫn ÉP chạy Predict với vận tốc góc bằng 0
                % Nhiệm vụ: Giữ nguyên góc cũ nhưng kích hoạt Q*dt để P tăng đều độ bất định
                obj.predictKalman(0);
            end

            % Update Magnet
            if obj.isUseMag
                z_mag = -atan2(obj.magY_filt, obj.magX_filt);
                obj.updateKalman(z_mag, obj.Rmag);
            end

            % Update GNSS
            if obj.isUseGnss
                Rgnss = gnssHeadAcc^2;   
                obj.updateKalman(gnssHead, Rgnss); 
            end
        end
        
        % ===================getEst========================================

        function [yaw, gyroBias, isUseGyro, isUseMag, isUseGnss] = getEst(obj)
            yaw = obj.x(1);        % Xuất góc Yaw ước lượng (rad)
            gyroBias = obj.x(2);   % Xuất sai số trôi Gyro ước lượng (rad/s)
            
            isUseGyro = obj.isUseGyro;
            isUseMag = obj.isUseMag;
            isUseGnss = obj.isUseGnss;
        end
    end
    


    % =====================================================================
    % Preprocessing delta T, gyroZ, magX, magY, GNSS (Private), Predict,
    % Update,....
    % =====================================================================



    methods (Access = private)
        % ==================Xử lý Delta T==================================

        function dt = preprocessTime(obj, currentTime)
            if isnan(obj.prevTime)
                obj.prevTime = currentTime;
                dt = 0; % Lần chạy đầu tiên
                return;
            end
            
            dt = currentTime - obj.prevTime;
            obj.prevTime = currentTime; 
            
            % Xử lý lỗi gói tin (bất thường)
            if dt <= 0 || dt > obj.MAX_TIME_STEP
                dt = 0; % Ép về 0 để bỏ qua chu kỳ này
            end
        end
        
        % ===============Xử lý Gyro========================================

        function preprocessGyro(obj, gyroZ)
            obj.isUseGyro = true; % Mặc định là tin tưởng
                
            % Check Gyro lỗi
            if isnan(gyroZ)
                obj.isUseGyro = false;
                return;
            end
            
            % Nếu vận tốc góc vượt ngưỡng 300 độ/s -> Nhiễu đỉnh (Spike) -> Từ chối
            if abs(gyroZ) > obj.MAX_GYRO_RATE
                obj.isUseGyro = false;
                return;
            end

            % Check data giữ const
            if isnan(obj.prevGyroZ)
                % Lần chạy đầu tiên: Khởi tạo giá trị
                obj.prevGyroZ = gyroZ;
                obj.stuckGyroTime = 0;
            else
                % Nếu giá trị mới GIỐNG HỆT giá trị cũ
                if (gyroZ == obj.prevGyroZ)
                    % Cộng dồn thời gian bị kẹt
                    obj.stuckGyroTime = obj.stuckGyroTime + obj.currentDt;
                else
                    % Dữ liệu có dao động (bình thường) -> Reset bộ đếm
                    obj.stuckGyroTime = 0;
                end
                
                % Lưu lại giá trị để dành so sánh cho chu kỳ tiếp theo
                obj.prevGyroZ = gyroZ;
                
                % KIỂM TRA NGƯỠNG: Nếu kẹt quá lâu -> Ngắt cờ
                if obj.stuckGyroTime > obj.STUCK_TIME_THRESHOLD
                    obj.isUseGyro = false;
                end
            end
        end
        
        % ==============Xử lý magX, magY, magNorm==========================

        function preprocessMagnet(obj, magX, magY)
            obj.isUseMag = true; % Mặc định là tin tưởng
            
            if isnan(obj.prevMagX * obj.prevMagY)
                obj.prevMagX = magX;
                obj.prevMagY = magY;
                obj.stuckMagTime = 0;
            else

                % Ngưỡng lọc nhiễu nền để xác định xem trục kia có đang thực sự chuyển động không
                MOVING_THRESHOLD = 0.0005; 
                
                % Tình huống 1: Treo toàn tập I2C (Cả 2 trục cùng giữ nguyên một số)
                isBothStuck = (magX == obj.prevMagX) && (magY == obj.prevMagY);
                
                % Tình huống 2: Liệt trục X (X đứng im tuyệt đối, nhưng Y biến thiên rõ rệt)
                isXStuck = (magX == obj.prevMagX) && (abs(magY - obj.prevMagY) > MOVING_THRESHOLD);
                
                % Tình huống 3: Liệt trục Y (Y đứng im tuyệt đối, nhưng X biến thiên rõ rệt)
                isYStuck = (magY == obj.prevMagY) && (abs(magX - obj.prevMagX) > MOVING_THRESHOLD);
                
                % Tình huống 4: Đứt dây tín hiệu (Ít nhất 1 trong 2 trục sụt về số 0 tuyệt đối)
                isDeadZero = (magX == 0) || (magY == 0);

                if isBothStuck || isXStuck || isYStuck || isDeadZero
                    obj.stuckMagTime = obj.stuckMagTime + obj.currentDt;
                else
                    obj.stuckMagTime = 0; % Dữ liệu có dao động -> Reset
                end
                
                % Lưu lại cho chu kỳ sau
                obj.prevMagX = magX;
                obj.prevMagY = magY;
                
                % Nếu kẹt quá lâu -> Ngắt cờ và Thoát
                if obj.stuckMagTime > obj.STUCK_TIME_THRESHOLD
                    obj.isUseMag= false;
                    return; 
                end
            end
            
            % Tính magNorm
            magNorm = sqrt(magX^2 + magY^2);
            
            % Nếu vector từ trường lệch quá xa 1 -> Từ trường đang bị nhiễu -> Từ chối
            if abs(magNorm - 1.0) > obj.MAG_NORM_TOLERANCE
                obj.isUseMag = false;
                return;
            end

            % Nếu magNorm tốt, cho magX, magY qua LPF rồi mới tính atan
            if isnan(obj.magX_filt)
                % Khởi tạo giá trị bộ lọc ở chu kỳ đầu tiên có dữ liệu hợp lệ
                obj.magX_filt = magX;
                obj.magY_filt = magY;
            else
                % Phương trình LPF bậc 1: y_k = alpha * x_k + (1 - alpha) * y_k-1
                obj.magX_filt = obj.MAG_LPF_ALPHA * magX + (1 - obj.MAG_LPF_ALPHA) * obj.magX_filt;
                obj.magY_filt = obj.MAG_LPF_ALPHA * magY + (1 - obj.MAG_LPF_ALPHA) * obj.magY_filt;
            end
        end
        
        % ===================Xử lý GNSS====================================

        function preprocessGnss(obj, isNewGnss, isGoodGnss, gnssHeadAcc)

            obj.isUseGnss = false;
            
            % Check điều kiện cờ báo tốt và gnssHeadAcc trong ngưỡng an
            % toàn
            if isNewGnss == 1 && isGoodGnss == 1 && gnssHeadAcc < obj.MAX_GNSS_ACC
                    obj.isUseGnss = true;
            end

        end

        % ==================Predict Kalman=================================

        function predictKalman(obj, gyroZ)

            dt = obj.currentDt;
            
            % x += (Ax + Bu) * dt
            x_dot = obj.A * obj.x + obj.B * gyroZ;
            obj.x = obj.x + x_dot * dt;
            
            % P += (AP + PA^T + Q) * dt
            P_dot = obj.A * obj.P + obj.P * obj.A' + obj.Q;
            obj.P = obj.P + P_dot * dt;

            % Ép P đối xứng, xác định dương trong trường hợp bị fail điều
            % kiện
            obj.P = 0.5 * (obj.P + obj.P');

            % Giới hạn góc Yaw luôn nằm trong khoảng [-pi, pi]
            obj.x(1) = wrapToPi(obj.x(1));
        end

        % ==================Update Kalman==================================

        function updateKalman(obj, z, R)
            % S = H * P * H^T + R
            S = obj.H * obj.P * obj.H' + R;
            
            % K = P * H^T * S^-1
            K = (obj.P * obj.H') / S;
            
            % y = z - H * x
            y = z - obj.H * obj.x;
            y = wrapToPi(y); % Xử lý bất thường vượt ranh giới góc
            
            d2 = (y^2) / S;
            GATE_THRESHOLD = 25; 
            
            if d2 > GATE_THRESHOLD
                return; 
            end

            % Cập nhật trạng thái và hiệp phương sai
            obj.x = obj.x + K * y;
            obj.P = obj.P - K * obj.H * obj.P;

             % Ép P đối xứng, xác định dương trong trường hợp bị fail điều
            % kiện
            obj.P = 0.5 * (obj.P + obj.P');
            
            obj.x(1) = wrapToPi(obj.x(1));
        end
    end

end
