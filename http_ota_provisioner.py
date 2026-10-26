# pyright: reportUndefinedVariable=false
# pylint: disable=undefined-variable
Import("env")

import os
import urllib.request
import urllib.error

# 1. Read custom camera options from active platformio.ini environment
cam_id = env.GetProjectOption("custom_cam_id", "ESP32_R1_C1_CAM1")
shelf_id = env.GetProjectOption("custom_shelf_id", "SHELF_R1_C1")
ip_octet = env.GetProjectOption("custom_ip_octet", "101")
ota_mode = env.GetProjectOption("custom_ota_mode", "")

# Inject macro definitions for src/main.cpp
env.Append(CPPDEFINES=[
    ("CUSTOM_DEVICE_ID", env.StringifyMacro(cam_id)),
    ("CUSTOM_SHELF_ID", env.StringifyMacro(shelf_id)),
    ("CUSTOM_IP_OCTET", int(ip_octet))
])


def http_ota_upload(source, target, env):
    firmware_path = str(source[0])
    upload_port = env.GetProjectOption("upload_port", f"192.168.0.{ip_octet}")
    url = f"http://{upload_port}/update"

    print(f"[HTTP OTA] Uploading {firmware_path} to {url} ({cam_id})...")
    with open(firmware_path, "rb") as f:
        firmware_data = f.read()

    boundary = "----ESP32CamOTAUpdateBoundary"
    filename = os.path.basename(firmware_path)
    header = (
        f"--{boundary}\r\n"
        f'Content-Disposition: form-data; name="update"; filename="{filename}"\r\n'
        f"Content-Type: application/octet-stream\r\n\r\n"
    ).encode("utf-8")
    footer = f"\r\n--{boundary}--\r\n".encode("utf-8")
    body = header + firmware_data + footer

    req = urllib.request.Request(
        url,
        data=body,
        headers={
            "Content-Type": f"multipart/form-data; boundary={boundary}",
            "Content-Length": str(len(body))
        },
        method="POST"
    )

    try:
        with urllib.request.urlopen(req, timeout=60) as resp:
            resp_text = resp.read().decode("utf-8", errors="ignore")
            print(f"[HTTP OTA] Success ({resp.status}): {resp_text}")
            return 0
    except urllib.error.URLError as e:
        print(f"[HTTP OTA] Upload failed: {e}")
        return 1


# 2. Override UPLOADCMD when custom_ota_mode == "http"
if ota_mode == "http":
    env.Replace(
        UPLOAD_PROTOCOL="custom",
        UPLOADCMD=http_ota_upload
    )
