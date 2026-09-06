#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
CH05_Bridge — ملف واحد شامل لتشغيل جسر BLE <-> منفذ COM لجهاز M5Stack "CH05"

لا تحتاج لتثبيت أي مكتبة يدوياً: هذا الملف يتحقق من وجود bleak وpyserial،
ويثبّتهما تلقائياً عند أول تشغيل إن كانا غير موجودين.

طريقة الاستخدام الأسهل (نقرة واحدة):
    شغّل ملف Run_CH05_Bridge.bat بجانب هذا الملف — سيسألك عن رقم منفذ COM
    ثم يعمل مباشرة.

طريقة الاستخدام من سطر الأوامر (اختياري، لمن يفضّلها):
    python CH05_Bridge.py --com COM11
    python CH05_Bridge.py --com COM11 --name CH05 --address AA:BB:CC:DD:EE:FF

الخلفية: شريحة ESP32-S3 داخل M5Stack CoreS3 لا تملك بلوتوث كلاسيكي (SPP)،
لذا لا يمكن لجهاز CH05 (BLE) فتح منفذ COM حقيقي من تلقاء نفسه. هذا السكربت
يتصل بالجهاز عبر BLE UART (Nordic UART Service) وينقل البايتات الخام بين
BLE وأحد طرفي زوج منافذ COM افتراضي أنشأته أداة com0com — فيتصرف الطرف
الآخر من الزوج تماماً كمنفذ تسلسلي حقيقي أمام أي برنامج مثل Visualizer 3D.
"""

import argparse
import asyncio
import subprocess
import sys
from typing import Optional

# ---- تثبيت المكتبات المطلوبة تلقائياً عند أول تشغيل ------------------------
def ensure_dependency(module_name: str, pip_name: str) -> None:
    try:
        __import__(module_name)
    except ImportError:
        print(f"[*] تثبيت المكتبة المطلوبة '{pip_name}' لأول مرة... الرجاء الانتظار")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "--quiet", pip_name])


ensure_dependency("bleak", "bleak>=0.21")
ensure_dependency("serial", "pyserial>=3.5")

from bleak import BleakClient, BleakScanner  # noqa: E402
import serial  # noqa: E402

# نفس المعرّفات المعرّفة في firmware/GeoScan3D_v9/src/main.cpp (BLE_SERVICE_UUID/TX/RX)
NUS_TX_UUID = "6e400002-b5a3-f393-e0a9-e50e24dcca9e"  # notify: من الجهاز إلى الحاسوب
NUS_RX_UUID = "6e400003-b5a3-f393-e0a9-e50e24dcca9e"  # write: من الحاسوب إلى الجهاز
WRITE_CHUNK = 20  # آمن بغض النظر عن تفاوض MTU (ATT الافتراضي = 23 بايت، أي 20 حمولة)


async def find_device(name: str, timeout: float):
    print(f"[*] البحث عن جهاز BLE باسم '{name}' (حتى {timeout:.0f} ثانية)...")
    device = await BleakScanner.find_device_by_filter(
        lambda d, adv: d.name == name, timeout=timeout
    )
    if device is None:
        raise RuntimeError(f"لم يتم العثور على جهاز BLE باسم '{name}' — تأكد أنه مُشغَّل وقريب")
    return device


async def run_bridge(com_port: str, device_name: str, address: Optional[str],
                      baud: int, scan_timeout: float) -> None:
    target = address or await find_device(device_name, scan_timeout)
    ser = serial.Serial(com_port, baudrate=baud, timeout=0)
    print(f"[+] تم فتح منفذ COM: {com_port}")

    async with BleakClient(target) as client:
        print(f"[+] تم الاتصال بـ BLE: {client.address}")

        def on_notify(_, data: bytearray) -> None:
            ser.write(bytes(data))

        await client.start_notify(NUS_TX_UUID, on_notify)
        print("[+] الجسر يعمل الآن — وجّه Visualizer 3D إلى الطرف الآخر من زوج com0com")
        print("[+] اترك هذه النافذة مفتوحة، اضغط Ctrl+C للإيقاف")

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


def prompt_missing_args(args: argparse.Namespace) -> None:
    """وضع تفاعلي: يُستخدم عند التشغيل بنقرتين بدون أي معطيات سطر أوامر."""
    if not args.com:
        args.com = input("أدخل رقم منفذ COM الخاص بهذا الجسر (مثال: COM11): ").strip()
    if not args.name:
        name = input("اسم جهاز BLE (اتركه فارغاً لاستخدام الافتراضي CH05): ").strip()
        args.name = name or "CH05"


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--com", default=None,
                   help="طرف منفذ COM الافتراضي الذي يستخدمه هذا الجسر (أعطِ الطرف الآخر لـ Visualizer 3D)")
    p.add_argument("--name", default=None, help="اسم جهاز BLE المُعلَن (افتراضي: CH05)")
    p.add_argument("--address", default=None,
                   help="عنوان MAC ثابت للجهاز (اختياري، يتخطى البحث بالاسم)")
    p.add_argument("--baud", type=int, default=115200,
                   help="سرعة فتح منفذ COM المحلي (لا تؤثر عبر BLE، فقط لفتح pyserial)")
    p.add_argument("--scan-timeout", type=float, default=10.0,
                   help="مهلة البحث عن الجهاز بالثواني")
    args = p.parse_args()

    prompt_missing_args(args)

    try:
        asyncio.run(run_bridge(args.com, args.name, args.address, args.baud, args.scan_timeout))
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        print(f"[!] خطأ: {exc}")
        input("اضغط Enter للإغلاق...")


if __name__ == "__main__":
    main()
