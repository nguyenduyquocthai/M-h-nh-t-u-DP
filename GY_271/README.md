# GY-271 (la bàn HMC5883L / QMC5883L) + ESP32

Đọc cảm biến từ trường 3 trục GY-271 bằng ESP32 qua I2C, tính **hướng la bàn** (độ) và hiển thị trên máy tính.

Thư mục này là một dự án PlatformIO **độc lập** với phần lidar ở thư mục gốc.

> **Chú ý:** nạp code này sẽ **ghi đè firmware lidar** trên cùng board ESP32. Muốn chạy lại lidar, nạp lại từ thư mục gốc bằng `pio run -t upload`.

---

## 1. Sơ đồ đấu nối

| Chân GY-271 | Nối tới ESP32 | Chức năng |
|:---:|:---:|---|
| **VCC** | **3V3** | Cấp nguồn 3,3 V |
| **GND** | **GND** | Mass |
| **SCL** | **GPIO33** | Xung nhịp I2C |
| **SDA** | **GPIO32** | Dữ liệu I2C |
| DRDY | Không nối | Báo dữ liệu sẵn sàng, chưa dùng |

```text
      GY-271                          ESP32
  ┌─────────────┐                ┌──────────────┐
  │ VCC  ───────┼────────────────┤ 3V3          │
  │ GND  ───────┼────────────────┤ GND          │
  │ SCL  ───────┼────────────────┤ GPIO33       │
  │ SDA  ───────┼────────────────┤ GPIO32       │
  │ DRDY (để hở)│                │              │
  └─────────────┘                └──────────────┘
```

Lưu ý khi đấu dây:

- Cấp **3V3**, không cấp 5 V và không cấp từ VIN.
- **SDA vào GPIO32, SCL vào GPIO33.** Đừng cắm ngược. Chương trình sẽ báo `!!! THAY cam bien khi DAO chan` nếu phát hiện cắm ngược.
- Module GY-271 thường có sẵn điện trở kéo lên cho SDA/SCL, không cần thêm điện trở ngoài.
- Dây ngắn, cắm chắc. Nên **hàn chân header** vào module. Chân cắm hờ là nguyên nhân phổ biến nhất khiến cảm biến lúc có lúc mất (xem mục 7).
- Muốn dùng chân khác, sửa `I2C_SDA_PIN` và `I2C_SCL_PIN` ở đầu `src/main.cpp` rồi nạp lại. Mặc định của ESP32 là SDA = GPIO21, SCL = GPIO22. Trên board đã thử, chân GPIO22 cho kết quả không ổn định nên dự án dùng GPIO32/33.

### Chip và địa chỉ I2C

Module GY-271 có thể dùng một trong hai chip, chương trình tự nhận:

| Chip | Địa chỉ I2C | Cách nhận biết |
|---|:---:|---|
| HMC5883L | `0x1E` | Thanh ghi ID đọc ra `H`, `4`, `3` |
| QMC5883L | `0x0D` | Thanh ghi ID `0x0D` = `0xFF` |

Module trong lần thử nghiệm dùng chip **HMC5883L**.

---

## 2. Cài đặt

Làm theo mục 4 của [README gốc](../README.md): driver CP210x, VS Code với PlatformIO, và gói Python `pyserial matplotlib numpy` (cho màn hình giám sát). Dự án này **không cần thư viện Arduino ngoài**, chỉ dùng `Wire` và `Preferences` có sẵn trong ESP32.

---

## 3. Nạp code cho ESP32

1. Đấu dây như mục 1, cắm USB, kiểm tra Device Manager có cổng COM.
2. Đóng mọi chương trình đang giữ cổng COM (Serial Monitor, `compass_viewer.py`...).
3. Nạp bằng một trong hai cách:
   - Mở thư mục **`GY_271`** bằng VS Code (làm thư mục dự án riêng) rồi bấm **Upload** của PlatformIO.
   - Hoặc từ thư mục gốc dự án:
     ```powershell
     pio run -d GY_271 -t upload --upload-port COM12
     ```
     Thay `COM12` bằng cổng của bạn. Nếu không có lệnh `pio`, dùng `%USERPROFILE%\.platformio\penv\Scripts\pio.exe`.
4. Chờ `Hash of data verified` và `[SUCCESS]`. Nếu báo `Failed to connect`, giữ nút **BOOT** trên board lúc bắt đầu nạp.

Cấu hình trong `platformio.ini`: board `esp32dev`, framework Arduino, `monitor_speed = 115200`.

---

## 4. Xem dữ liệu

### Cách A: Serial Monitor (115200 baud)

```powershell
pio device monitor -d GY_271
```

Kết quả đúng:

```text
=== GY-271 + ESP32 ===
Phat hien HMC5883L tai 0x1E
Chua hieu chuan (go 'c' de hieu chuan).
Lenh: c = hieu chuan | r = xoa hieu chuan | h = huong dan
X=   34.0 Y=  -54.8 Z=    5.7 uT | |B|= 64.7 uT | Huong= 301.9 deg (NW)  [chua hieu chuan]
```

| Trường | Ý nghĩa |
|---|---|
| `X`, `Y`, `Z` | Cường độ từ trường theo 3 trục (µT, microTesla) |
| `\|B\|` | Độ lớn tổng của từ trường. Từ trường Trái Đất khoảng 25–65 µT, ngoài khoảng này thường do có kim loại/nam châm gần cảm biến |
| `Huong` | Hướng la bàn, 0° = Bắc, tăng theo chiều kim đồng hồ (90° Đông, 180° Nam, 270° Tây) |
| `[chua hieu chuan]` | Chưa hiệu chuẩn, hướng có thể lệch nhiều |

Lệnh gõ trong Serial Monitor:

| Lệnh | Tác dụng |
|:---:|---|
| `c` | Hiệu chuẩn 20 giây |
| `r` | Xóa hiệu chuẩn |
| `h` | In lại hướng dẫn |

### Cách B: Màn hình giám sát bằng đồ họa

```powershell
python GY_271\compass_viewer.py --port COM12
```

- Bên trái là **kim la bàn**, bên phải là **đồ thị X/Y/Z** theo thời gian (60 giây gần nhất).
- Nhấn phím **`c`** trên cửa sổ để hiệu chuẩn, **`r`** để xóa hiệu chuẩn.
- Đóng cửa sổ để nhả cổng COM.

---

## 5. Hiệu chuẩn (làm một lần)

Từ trường quanh cảm biến luôn bị lệch bởi dây điện, vỏ kim loại, pin... nên cần hiệu chuẩn để hướng đúng.

1. Gõ `c` (hoặc nhấn `c` trên màn hình giám sát).
2. Trong **20 giây**, xoay cảm biến chậm theo mọi hướng, như vẽ số 8 trong không khí và lật nghiêng. Đừng để đứng yên.
3. Chương trình tự báo `Hieu chuan xong va da luu`. Kết quả (độ lệch và hệ số tỉ lệ từng trục) lưu vào flash, **còn nguyên sau khi tắt nguồn**.
4. Gắn cảm biến đúng vị trí cuối cùng rồi hiệu chuẩn trong chính vị trí đó, vì từ trường của vật gần nó cũng ảnh hưởng.

Kiểm tra: đặt cảm biến nằm ngang, xoay một vòng tròn, hướng phải đi qua 0–360° đều. So với la bàn điện thoại (đặt xa nam châm, kim loại).

---

## 6. Cách hoạt động

- **Giao tiếp I2C** (100 kHz) trên GPIO32/33. Chương trình quét bus, tự nhận HMC5883L (`0x1E`) hoặc QMC5883L (`0x0D`) và cấu hình chip chạy đo liên tục.
- **HMC5883L**: ±1,3 gauss (1090 LSB/gauss), trung bình 8 mẫu, 15 Hz. Thứ tự thanh ghi dữ liệu là **X, Z, Y** (đặc thù của chip này).
- **QMC5883L**: ±8 gauss (3000 LSB/gauss), 200 Hz. Thứ tự thanh ghi là X, Y, Z.
- Đổi số thô sang µT: 1 gauss = 100 µT.
- **Hiệu chuẩn** dùng giá trị nhỏ nhất/lớn nhất mỗi trục khi xoay: độ lệch (hard-iron) = trung điểm, hệ số tỉ lệ (soft-iron đơn giản) = bán kính trung bình / bán kính từng trục.
- **Hướng** = `atan2(Y, X)` đổi ra độ, cộng `DECLINATION_DEG` (độ lệch từ trường tại nơi bạn ở) và `HEADING_OFFSET_DEG` (bù nếu gắn cảm biến lệch trục), rồi đưa về khoảng 0–360°.
- **Tự phục hồi**: nếu 2 giây không có dữ liệu mới, chương trình thực hiện phục hồi bus I2C (đưa 9 xung lên SCL), quét lại và khởi tạo lại chip.

Các thông số ở đầu `src/main.cpp`:

| Macro | Mặc định | Ý nghĩa |
|---|:---:|---|
| `I2C_SDA_PIN` / `I2C_SCL_PIN` | 32 / 33 | Chân I2C |
| `I2C_FREQ_HZ` | 100000 | Tốc độ I2C |
| `DECLINATION_DEG` | 0 | Độ lệch từ trường (độ, Đông là dương). Việt Nam khoảng -1° |
| `HEADING_OFFSET_DEG` | 0 | Bù hướng khi gắn lệch trục |
| `PRINT_INTERVAL_MS` | 200 | Chu kỳ in dữ liệu |
| `CALIBRATION_MS` | 20000 | Thời gian hiệu chuẩn |

---

## 7. Xử lý lỗi

Khi không thấy cảm biến, chương trình in mỗi 2 giây:

```text
Khong tim thay HMC5883L (0x1E) hay QMC5883L (0x0D). Kiem tra day ...
Quet I2C: (khong thay thiet bi nao)
  dia chi 0x1E -> ma loi Wire 2
  dia chi 0x0D -> ma loi Wire 2
Chan doan (khong pull-up noi): SDA=CAO SCL=THAP
```

| Dấu hiệu | Ý nghĩa và cách xử lý |
|---|---|
| `SDA=CAO SCL=CAO` nhưng vẫn `khong thay thiet bi nao` | Nguồn và hai dây tín hiệu có vẻ tốt nhưng chip không trả lời: kiểm tra dây **GND**, thử module khác |
| `SCL=THAP` (hoặc `SDA=THAP`) | Dây đó **không tới module** hoặc module không có nguồn/GND. Cắm chặt lại, thay dây, hàn chân header |
| Kết quả đo lúc cao lúc thấp | Tiếp xúc chập chờn: **hàn chân header**, cố định dây |
| Mã lỗi Wire = 2 | Không có thiết bị nào trả lời ở địa chỉ đó |
| Mã lỗi Wire = 5 | Timeout: bus bị treo (SDA/SCL bị giữ ở mức thấp). Rút nguồn module vài giây rồi cắm lại |
| `!!! THAY cam bien khi DAO chan` | SDA và SCL đang cắm ngược chân |
| `Mat du lieu tu cam bien ... Da ket noi lai` | Cảm biến mất rồi tự nối lại: dây chưa chắc |
| `Hieu chuan that bai: du lieu khong doi` | Khi hiệu chuẩn bạn đã không xoay cảm biến, thử lại |
| Hướng sai nhiều, `\|B\|` ngoài 25–65 µT | Có kim loại, nam châm, loa, dây dòng lớn gần cảm biến; hiệu chuẩn lại ở đúng vị trí gắn |
| Upload báo `port doesn't exist` / `Access is denied` | Chương trình khác đang giữ COM, đóng nó (ví dụ `compass_viewer.py`) |

Cảm biến chạy ổn khi: `Phat hien ... tai 0x...` xuất hiện, dữ liệu in đều đặn khoảng 5 dòng/giây, không có dòng `Mat du lieu`.

---

## 8. Giới hạn

- Hướng chỉ đúng khi cảm biến **nằm ngang**. Chưa có bù nghiêng (cần thêm cảm biến gia tốc).
- Cần hiệu chuẩn và cần cảm biến cách xa kim loại, nam châm, motor, dây dòng lớn (ví dụ lidar đang quay gần đó có thể gây nhiễu).
- Cần `DECLINATION_DEG` đúng nếu muốn so sánh với hướng Bắc địa lý. Mặc định 0 cho ra hướng Bắc từ.
- Chương trình chỉ in ra USB, chưa gửi dữ liệu lên server qua Wi-Fi.

---

## 9. Toàn bộ code nạp cho ESP32

Đây là nội dung file [`src/main.cpp`](src/main.cpp) (file này là bản chính; đoạn dưới chỉ để tham khảo nhanh).

<details>
<summary>Bấm để xem toàn bộ <code>src/main.cpp</code></summary>

```cpp
/*
 * Doc cam bien la ban GY-271 (HMC5883L hoac QMC5883L) bang ESP32, khong can thu vien ngoai.
 *
 * Day noi:
 *   GY-271 VCC -> 3V3 ESP32
 *   GY-271 GND -> GND ESP32
 *   GY-271 SCL -> GPIO33
 *   GY-271 SDA -> GPIO32
 *   GY-271 DRDY -> de ho
 *
 * Chuong trinh tu nhan chip:
 *   HMC5883L: dia chi I2C 0x1E (thanh ghi ID = 'H','4','3')
 *   QMC5883L: dia chi I2C 0x0D (thanh ghi ID 0x0D = 0xFF)
 *
 * Lenh qua Serial Monitor (115200 baud, ket thuc dong bat ky):
 *   c : hieu chuan 20 giay - xoay cam bien du moi huong (ve so 8 trong khong gian)
 *   r : xoa hieu chuan
 *   h : in lai huong dan
 * Ket qua hieu chuan duoc luu vao bo nho flash, giu lai sau khi tat nguon.
 *
 * Luu y: huong (heading) chi dung khi cam bien nam NGANG, khong co bu nghieng.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <math.h>

// ---------------- Cau hinh ----------------
#define I2C_SDA_PIN 32
#define I2C_SCL_PIN 33
#define I2C_FREQ_HZ 100000       // 100 kHz: ben hon 400 kHz khi day dai/tiep xuc kem

// Do lech tu truong tai dia diem cua ban (do, dong la +). Tra cuu "magnetic declination" theo toa do.
// Viet Nam khoang -1 do. De 0 neu khong can chinh xac tuyet doi.
#define DECLINATION_DEG 0.0f
// Cong them vao huong neu gan cam bien lech truc so voi huong "phia truoc" cua ban (do).
#define HEADING_OFFSET_DEG 0.0f

#define PRINT_INTERVAL_MS 200
#define CALIBRATION_MS 20000

// ---------------- Chip ----------------
enum ChipType { CHIP_NONE, CHIP_HMC5883L, CHIP_QMC5883L };
static ChipType chip = CHIP_NONE;
static uint8_t  chipAddr = 0;
static float    uTperLsb = 0;          // he so doi so tho sang microTesla

// HMC5883L
#define HMC_ADDR 0x1E
#define HMC_REG_CONFIG_A 0x00
#define HMC_REG_CONFIG_B 0x01
#define HMC_REG_MODE     0x02
#define HMC_REG_DATA     0x03          // X MSB, X LSB, Z MSB, Z LSB, Y MSB, Y LSB
#define HMC_REG_STATUS   0x09
#define HMC_REG_ID_A     0x0A

// QMC5883L
#define QMC_ADDR 0x0D
#define QMC_REG_DATA     0x00          // X LSB, X MSB, Y LSB, Y MSB, Z LSB, Z MSB
#define QMC_REG_STATUS   0x06
#define QMC_REG_CONTROL1 0x09
#define QMC_REG_CONTROL2 0x0A
#define QMC_REG_PERIOD   0x0B
#define QMC_REG_ID       0x0D

// ---------------- Khoi phuc bus I2C ----------------
// Khi day chap chon, cam bien co the giu SDA o muc thap va treo bus. Day 9 xung len SCL de no nha SDA,
// roi tao dieu kien STOP, sau do khoi dong lai Wire.
static void i2cRestart() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT_PULLUP);
  pinMode(I2C_SCL_PIN, OUTPUT);
  digitalWrite(I2C_SCL_PIN, HIGH);
  delayMicroseconds(10);
  for (int i = 0; i < 9 && digitalRead(I2C_SDA_PIN) == LOW; i++) {
    digitalWrite(I2C_SCL_PIN, LOW);  delayMicroseconds(10);
    digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  }
  // STOP: SDA len cao khi SCL cao
  pinMode(I2C_SDA_PIN, OUTPUT);
  digitalWrite(I2C_SDA_PIN, LOW);  delayMicroseconds(10);
  digitalWrite(I2C_SCL_PIN, HIGH); delayMicroseconds(10);
  digitalWrite(I2C_SDA_PIN, HIGH); delayMicroseconds(10);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(I2C_FREQ_HZ);
  Wire.setTimeOut(50);
}

// Chan doan day: module GY-271 co dien tro keo len 3V3 san, nen khi nguon + day tot thi SDA va SCL
// doc duoc muc CAO ngay ca khi ESP32 khong bat pull-up noi.
static void diagnoseLines() {
  Wire.end();
  pinMode(I2C_SDA_PIN, INPUT);
  pinMode(I2C_SCL_PIN, INPUT);
  delay(5);
  int sdaHigh = 0, sclHigh = 0;
  for (int i = 0; i < 20; i++) {
    sdaHigh += digitalRead(I2C_SDA_PIN);
    sclHigh += digitalRead(I2C_SCL_PIN);
    delay(1);
  }
  Serial.printf("Chan doan (khong pull-up noi): SDA=%s SCL=%s\n", sdaHigh >= 18 ? "CAO" : "THAP", sclHigh >= 18 ? "CAO" : "THAP");
  if (sdaHigh < 18 || sclHigh < 18)
    Serial.println("  -> Day SDA/SCL (hoac 1 trong 2) khong toi module, hoac module khong co nguon 3V3/GND.");
  else
    Serial.println("  -> Nguon va day tin hieu co ve tot nhung chip khong tra loi: kiem tra chip co bi hong/gia khong.");
}

// ---------------- I2C co ban ----------------
static bool writeReg(uint8_t addr, uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

static bool readRegs(uint8_t addr, uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;   // repeated start
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

static bool devicePresent(uint8_t addr) {
  Wire.beginTransmission(addr);
  return Wire.endTransmission() == 0;
}

static void scanBus() {
  Serial.print("Quet I2C: ");
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (devicePresent(a)) {
      Serial.printf("0x%02X ", a);
      found++;
    }
  }
  if (!found) Serial.print("(khong thay thiet bi nao)");
  Serial.println();
  if (!found) {
    // Ma loi Wire: 0 = OK, 2 = NACK dia chi (khong co thiet bi / khong tra loi), 3 = NACK du lieu,
    //              4 = loi khac, 5 = TIMEOUT (bus bi treo, thuong do SCL/SDA bi giu muc thap)
    const uint8_t addrs[2] = {HMC_ADDR, QMC_ADDR};
    for (uint8_t a : addrs) {
      Wire.beginTransmission(a);
      Serial.printf("  dia chi 0x%02X -> ma loi Wire %d\n", a, Wire.endTransmission());
    }
  }
}

// ---------------- Khoi tao chip ----------------
static bool initHMC() {
  uint8_t id[3];
  if (!readRegs(HMC_ADDR, HMC_REG_ID_A, id, 3)) return false;
  if (id[0] != 'H' || id[1] != '4' || id[2] != '3') return false;
  bool ok = writeReg(HMC_ADDR, HMC_REG_CONFIG_A, 0x70)   // trung binh 8 mau, 15 Hz
         && writeReg(HMC_ADDR, HMC_REG_CONFIG_B, 0x20)   // +-1.3 Ga: 1090 LSB/Gauss
         && writeReg(HMC_ADDR, HMC_REG_MODE, 0x00);      // do lien tuc
  if (!ok) return false;
  chip = CHIP_HMC5883L;
  chipAddr = HMC_ADDR;
  uTperLsb = 100.0f / 1090.0f;                           // 1 Gauss = 100 microTesla
  return true;
}

static bool initQMC() {
  uint8_t id;
  if (!readRegs(QMC_ADDR, QMC_REG_ID, &id, 1)) return false;
  if (id != 0xFF) return false;
  bool ok = writeReg(QMC_ADDR, QMC_REG_CONTROL2, 0x80)   // soft reset
         && (delay(10), writeReg(QMC_ADDR, QMC_REG_PERIOD, 0x01))
         && writeReg(QMC_ADDR, QMC_REG_CONTROL1, 0x1D);  // OSR512, +-8G, 200 Hz, lien tuc
  if (!ok) return false;
  chip = CHIP_QMC5883L;
  chipAddr = QMC_ADDR;
  uTperLsb = 100.0f / 3000.0f;                           // +-8G: 3000 LSB/Gauss
  return true;
}

static bool initSensor() {
  chip = CHIP_NONE;
  if (devicePresent(HMC_ADDR) && initHMC()) return true;
  if (devicePresent(QMC_ADDR) && initQMC()) return true;
  return false;
}

// ---------------- Doc du lieu ----------------
// Tra ve true neu co mau moi; x,y,z la so tho
static bool readRaw(int16_t &x, int16_t &y, int16_t &z) {
  uint8_t b[6], st;
  if (chip == CHIP_HMC5883L) {
    if (!readRegs(chipAddr, HMC_REG_STATUS, &st, 1) || !(st & 0x01)) return false;
    if (!readRegs(chipAddr, HMC_REG_DATA, b, 6)) return false;
    x = (int16_t)((b[0] << 8) | b[1]);
    z = (int16_t)((b[2] << 8) | b[3]);                   // HMC: thu tu X, Z, Y
    y = (int16_t)((b[4] << 8) | b[5]);
    if (x == -4096 || y == -4096 || z == -4096) return false;   // tran thang do
    return true;
  }
  if (chip == CHIP_QMC5883L) {
    if (!readRegs(chipAddr, QMC_REG_STATUS, &st, 1) || !(st & 0x01)) return false;
    if (!readRegs(chipAddr, QMC_REG_DATA, b, 6)) return false;
    x = (int16_t)((b[1] << 8) | b[0]);
    y = (int16_t)((b[3] << 8) | b[2]);
    z = (int16_t)((b[5] << 8) | b[4]);
    return true;
  }
  return false;
}

// ---------------- Hieu chuan (hard-iron + soft-iron don gian) ----------------
static Preferences prefs;
static float offs[3] = {0, 0, 0};       // microTesla
static float scale[3] = {1, 1, 1};
static bool  calibrated = false;

static bool calibrating = false;
static uint32_t calStart = 0;
static float calMin[3], calMax[3];

static void loadCalibration() {
  prefs.begin("gy271", true);
  calibrated = prefs.getBool("cal", false);
  if (calibrated) {
    for (int i = 0; i < 3; i++) {
      offs[i]  = prefs.getFloat(i == 0 ? "ox" : i == 1 ? "oy" : "oz", 0);
      scale[i] = prefs.getFloat(i == 0 ? "sx" : i == 1 ? "sy" : "sz", 1);
    }
  }
  prefs.end();
  Serial.println(calibrated ? "Da nap hieu chuan da luu." : "Chua hieu chuan (go 'c' de hieu chuan).");
}

static void saveCalibration() {
  prefs.begin("gy271", false);
  prefs.putBool("cal", calibrated);
  for (int i = 0; i < 3; i++) {
    prefs.putFloat(i == 0 ? "ox" : i == 1 ? "oy" : "oz", offs[i]);
    prefs.putFloat(i == 0 ? "sx" : i == 1 ? "sy" : "sz", scale[i]);
  }
  prefs.end();
}

static void startCalibration() {
  calibrating = true;
  calStart = millis();
  for (int i = 0; i < 3; i++) { calMin[i] = 1e9f; calMax[i] = -1e9f; }
  Serial.printf("HIEU CHUAN %d giay: xoay cam bien cham theo moi huong (ve so 8, lat nghieng), dung dung tu...\n",
                CALIBRATION_MS / 1000);
}

static void finishCalibration() {
  calibrating = false;
  float avgRadius = 0, radius[3];
  for (int i = 0; i < 3; i++) {
    radius[i] = (calMax[i] - calMin[i]) / 2.0f;
    avgRadius += radius[i];
  }
  avgRadius /= 3.0f;
  if (radius[0] < 1 || radius[1] < 1 || radius[2] < 1) {
    Serial.println("Hieu chuan that bai: du lieu khong doi - co dang xoay cam bien khong?");
    return;
  }
  for (int i = 0; i < 3; i++) {
    offs[i]  = (calMax[i] + calMin[i]) / 2.0f;
    scale[i] = avgRadius / radius[i];
  }
  calibrated = true;
  saveCalibration();
  Serial.printf("Hieu chuan xong va da luu. offset = (%.1f, %.1f, %.1f) uT, scale = (%.2f, %.2f, %.2f)\n",
                offs[0], offs[1], offs[2], scale[0], scale[1], scale[2]);
}

static void clearCalibration() {
  calibrated = false;
  for (int i = 0; i < 3; i++) { offs[i] = 0; scale[i] = 1; }
  saveCalibration();
  Serial.println("Da xoa hieu chuan.");
}

static void printHelp() {
  Serial.println("Lenh: c = hieu chuan | r = xoa hieu chuan | h = huong dan");
}

// ---------------- Huong ----------------
static const char *compassName(float deg) {
  static const char *names[] = {"N", "NE", "E", "SE", "S", "SW", "W", "NW"};
  return names[(int)((deg + 22.5f) / 45.0f) % 8];
}

// ---------------- Arduino ----------------
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== GY-271 + ESP32 ===");
  i2cRestart();

  while (!initSensor()) {
    Serial.println("Khong tim thay HMC5883L (0x1E) hay QMC5883L (0x0D). Kiem tra day SDA=GPIO32, SCL=GPIO33, VCC=3V3, GND.");
    scanBus();
    diagnoseLines();
    // Thu dao chan: neu thay thiet bi thi SDA va SCL dang bi cam nguoc
    Wire.end();
    Wire.begin(I2C_SCL_PIN, I2C_SDA_PIN);
    Wire.setClock(I2C_FREQ_HZ);
    Wire.setTimeOut(50);
    if (devicePresent(HMC_ADDR) || devicePresent(QMC_ADDR))
      Serial.println("!!! THAY cam bien khi DAO chan: day SDA va SCL dang bi cam NGUOC (SDA phai vao GPIO32, SCL vao GPIO33).");
    delay(2000);
    i2cRestart();            // thu giai phong bus neu bi treo
  }
  Serial.printf("Phat hien %s tai 0x%02X\n", chip == CHIP_HMC5883L ? "HMC5883L" : "QMC5883L", chipAddr);
  loadCalibration();
  printHelp();
}

void loop() {
  static uint32_t lastPrint = 0, lastGood = millis();
  static float lastX = 0, lastY = 0, lastZ = 0;
  static bool haveSample = false;

  // Lenh tu Serial
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == 'c' || ch == 'C') startCalibration();
    else if (ch == 'r' || ch == 'R') clearCalibration();
    else if (ch == 'h' || ch == 'H') printHelp();
  }

  int16_t rx, ry, rz;
  if (readRaw(rx, ry, rz)) {
    lastGood = millis();
    float x = rx * uTperLsb, y = ry * uTperLsb, z = rz * uTperLsb;   // uT tho
    if (calibrating) {
      float v[3] = {x, y, z};
      for (int i = 0; i < 3; i++) {
        if (v[i] < calMin[i]) calMin[i] = v[i];
        if (v[i] > calMax[i]) calMax[i] = v[i];
      }
    }
    lastX = (x - offs[0]) * scale[0];
    lastY = (y - offs[1]) * scale[1];
    lastZ = (z - offs[2]) * scale[2];
    haveSample = true;
  }

  if (calibrating && millis() - calStart >= CALIBRATION_MS) finishCalibration();

  // Mat cam bien (rut day, nhieu...) -> thu khoi tao lai
  if (millis() - lastGood > 2000) {
    Serial.println("Mat du lieu tu cam bien, thu khoi phuc bus I2C va khoi tao lai...");
    i2cRestart();
    scanBus();
    if (initSensor()) Serial.println("Da ket noi lai cam bien.");
    lastGood = millis();
  }

  if (haveSample && millis() - lastPrint >= PRINT_INTERVAL_MS) {
    lastPrint = millis();
    float heading = atan2f(lastY, lastX) * 180.0f / (float)M_PI + DECLINATION_DEG + HEADING_OFFSET_DEG;
    while (heading < 0) heading += 360.0f;
    while (heading >= 360.0f) heading -= 360.0f;
    float magnitude = sqrtf(lastX * lastX + lastY * lastY + lastZ * lastZ);
    if (calibrating) {
      Serial.printf("[HIEU CHUAN %2lu s] X=%7.1f Y=%7.1f Z=%7.1f uT\n",
                    (unsigned long)((CALIBRATION_MS - (millis() - calStart)) / 1000), lastX, lastY, lastZ);
    } else {
      Serial.printf("X=%7.1f Y=%7.1f Z=%7.1f uT | |B|=%5.1f uT | Huong=%6.1f deg (%s)%s\n",
                    lastX, lastY, lastZ, magnitude, heading, compassName(heading),
                    calibrated ? "" : "  [chua hieu chuan]");
    }
  }
  delay(2);
}
```

</details>

Cấu hình PlatformIO (`platformio.ini`):

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
```
