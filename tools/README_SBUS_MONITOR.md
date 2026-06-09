# SBUS Monitor - Hướng dẫn sử dụng

## Giới thiệu
`sbus_monitor.c` là ứng dụng Windows console để đọc và hiển thị giá trị 16 channels từ SBUS receiver (XM+) thông qua cổng COM.

**Đặc điểm:**
- Đọc dữ liệu từ COM port (baud: 100000, 8 bits, even parity, 2 stop bits)
- Parse SBUS frame (25 bytes)
- Hiển thị 16 channels với cập nhật tại chỗ (không in liên tục)
- Chỉ cập nhật khi có thay đổi giá trị

## Biên dịch

### Cách 1: Sử dụng MinGW (nếu có cài đặt)
```bash
gcc -o sbus_monitor.exe sbus_monitor.c
```

### Cách 2: Sử dụng Visual Studio Command Line
```bash
cl sbus_monitor.c
```

### Cách 3: Tải file exe đã biên dịch
File `sbus_monitor.exe` có thể chạy trực tiếp trên Windows 10/11.

## Sử dụng

### Cú pháp cơ bản:
```bash
sbus_monitor.exe [COM_PORT]
```

### Ví dụ:

**1. Sử dụng COM port mặc định (COM3):**
```bash
sbus_monitor.exe
```

**2. Sử dụng COM port cụ thể (ví dụ COM6):**
```bash
sbus_monitor.exe COM6
```

**3. Từ PowerShell:**
```powershell
.\sbus_monitor.exe COM6
```

## Tìm COM port của receiver

### Trên Windows:
1. **Device Manager:**
   - Nhấn `Win + X` → chọn `Device Manager`
   - Mở `Ports (COM & LPT)`
   - Tìm UART device hoặc CH340/CP2102 (tùy loại USB-to-UART)
   - Ghi nhớ COM port (ví dụ: COM6)

2. **Từ Command Line:**
   ```bash
   mode
   ```
   Hoặc liệt kê tất cả COM ports:
   ```powershell
   Get-WmiObject Win32_SerialPort | Select-Object Name, Description
   ```

## Hướng dẫn từng bước

### Bước 1: Chuẩn bị hardware
- Kết nối receiver SBUS (XM+) qua USB-to-UART adapter tới máy tính
- Cấp nguồn cho receiver
- Kiểm tra COM port trong Device Manager

### Bước 2: Biên dịch ứng dụng
```bash
cd tools
gcc -o sbus_monitor.exe sbus_monitor.c
```

### Bước 3: Chạy monitor
```bash
./sbus_monitor.exe COM6
```
(thay `COM6` bằng COM port của bạn)

### Bước 4: Đặt remote control
- Chuyển động các stick, nhấn các nút
- Xem giá trị 16 channels cập nhật trên màn hình

### Output ví dụ:
```
SBUS FRAME: 0F 7C E0 12 34 56 78 9A BC DE F0 12 34 56 78 9A BC DE F0 12 34 56 78 9A
CH 1:  124
CH 2:  124
CH 3:  124
CH 4:  124
CH 5:  124
CH 6:  124
CH 7:  124
CH 8:  636
CH 9:  200
CH10:    0
CH11: 1084
CH12:  500
CH13:  496
CH14:   86
CH15:  496
CH16:  496
```

## Lưu ý quan trọng

1. **Baud rate cố định: 100000** (không thể thay đổi từ dòng lệnh)
   - Nếu receiver khác dùng baud rate khác, sửa code dòng:
   ```c
   dcb.BaudRate = 100000;  // Thay đổi con số này
   ```

2. **Inverted SBUS:** Code này giả sử receiver đã có cấu hình inverted trong firmware ESP32
   - Nếu dữ liệu không hợp lệ, kiểm tra UART inversion trên ESP32

3. **Nếu không nhận được dữ liệu:**
   - Kiểm tra COM port đúng hay chưa
   - Kiểm tra receiver có cấp nguồn không
   - Kiểm tra kết nối dây USB/UART
   - Xem debug trên ESP32 monitor song song

## File liên quan
- `src/xm_plus.c` - Code phía ESP32 để gửi dữ liệu SBUS
- `include/xm_plus.h` - Header file SBUS

## Xóa cơ sở
- SBUS là protocol 25 bytes: 1 byte header (0x0F) + 23 bytes data + 1 byte flags
- Mỗi channel là 11 bits, nên 16 channels × 11 bits ≈ 23 bytes data
- Parse logic từ file `xm_plus.c`
