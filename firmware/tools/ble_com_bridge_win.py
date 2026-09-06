#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
جسر BLE <-> منفذ COM افتراضي لجهاز M5Stack "CH05" (GeoScan3D عبر BLE UART)

يمرّر البايتات الخام بالاتجاهين بين خدمة BLE UART (Nordic UART Service) الخاصة
بالفيرموير وطرف واحد من زوج منافذ COM افتراضي أنشأته أداة com0com. هذا يجعل أي
برنامج يتوقّع منفذ تسلسلي كلاسيكي (مثل Visualizer 3D) قادراً على العمل بدون أي
تعديل عليه، رغم أن شريحة ESP32-S3 لا تدعم Bluetooth Classic/SPP فعلياً.

الإعداد على ويندوز:
  1) تثبيت com0com (https://com0com.sourceforge.net/) وإنشاء زوج منافذ افتراضي،
     مثلاً COM10 <-> COM11.
  2) pip install -r requirements.txt
  3) توجيه Visualizer 3D إلى COM10 (الطرف الذي لا يستخدمه هذا السكربت).
  4) تشغيل: python ble_com_bridge_win.py --com COM11

السكربت لا يفهم محتوى البروتوكول (حزم 34-بايت، أوامر THR/CAL/...)، فقط ينقل
البايتات كما هي في الاتجاهين — لذلك يبقى Visualizer 3D يتحدث بنفس البروتوكول
الذي يعرفه أصلاً دون أي تغيير في تحليل البيانات لديه.
"""

import argparse
import asyncio
from typing import Optional

from bleak import BleakClient, BleakScanner
import serial

# نفس المعرّفات المعرّفة في firmware/GeoScan3D_v9/src/main.cpp (BLE_SERVICE_UUID/TX/RX)
NUS_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # notify: من الجهاز إلى الحاسوب
NUS_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # write: من الحاسوب إلى الجهاز
WRITE_CHUNK = 20  # آمن بغض النظر عن تفاوض MTU (ATT الافتراضي = 23 بايت، أي 20 حمولة)


async def find_device(name: str, timeout: float):
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name == name, timeout=timeout
    )
    if device is None:
        raise RuntimeError(f"لم يتم العثور على جهاز BLE باسم '{name}' خلال {timeout:.0f} ثانية")
    return device


async def run_bridge(com_port: str, device_name: str, address: Optional[str],
                      baud: int, scan_timeout: float):
    target = address or await find_device(device_name, scan_timeout)
    ser = serial.Serial(com_port, baudrate=baud, timeout=0)
    print(f"[+] تم فتح منفذ COM: {com_port}")

    async with BleakClient(target) as client:
        print(f"[+] تم الاتصال بـ BLE: {client.address}")

        def on_notify(_, data: bytearray) -> None:
            ser.write(bytes(data))

        await client.start_notify(NUS_TX_UUID, on_notify)
        print("[+] الجسر يعمل الآن — Ctrl+C للإيقاف")

        loop = asyncio.get_event_loop()
        try:
            while True:
                data = await loop.run_in_executor(None, ser.read, 256)
                for i in range(0, len(data), WRITE_CHUNK):
                    await client.write_gatt_char(
                        NUS_RX_UUID, data[i:i + WRITE_CHUNK], response=False
                    )
                await asyncio.sleep(0.01)
        finally:
            await client.stop_notify(NUS_TX_UUID)
            ser.close()


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--com", required=True,
                   help="طرف منفذ COM الافتراضي الذي يستخدمه هذا الجسر (أعطِ الطرف الآخر لـ Visualizer 3D)")
    p.add_argument("--name", default="CH05", help="اسم جهاز BLE المُعلَن (افتراضي: CH05)")
    p.add_argument("--address", default=None,
                   help="عنوان MAC ثابت للجهاز (اختياري، يتخطى البحث بالاسم)")
    p.add_argument("--baud", type=int, default=115200,
                   help="سرعة فتح منفذ COM المحلي (لا تؤثر عبر BLE، فقط لفتح pyserial)")
    p.add_argument("--scan-timeout", type=float, default=10.0,
                   help="مهلة البحث عن الجهاز بالثواني")
    args = p.parse_args()

    try:
        asyncio.run(run_bridge(args.com, args.name, args.address, args.baud, args.scan_timeout))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
