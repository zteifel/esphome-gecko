#!/usr/bin/env python3
"""Reflash (or read) the Arduino I2C proxy over WiFi, through the ESP32.

The ESP32 exposes its UART to the Arduino via ESPHome's serial_proxy. This script
bridges that to a local TCP port and runs avrdude against it. While the bridge is
subscribed, the gecko_spa component pauses the spa protocol and resets the Arduino,
so its bootloader is listening when avrdude starts talking.

Requires: pip install aioesphomeapi pyyaml, and avrdude (PlatformIO's tool-avrdude works).
The ESPHome config must have a serial_proxy on the Arduino UART, linked with
gecko_spa's serial_proxy_id.

Examples:
  # Read-only test that also backs up the current firmware:
  tools/arduino_ota.py --host 192.168.1.123 --secrets esphome/secrets.yaml -- -U flash:r:backup.hex:i
  # Flash new firmware:
  tools/arduino_ota.py --host 192.168.1.123 --secrets esphome/secrets.yaml -- -U flash:w:arduino/arduino-i2c-proxy-atmega328p.hex:i
"""

import argparse
import asyncio
import os
import shutil
import sys

import aioesphomeapi
import yaml

# Time from subscribing (which triggers the Arduino reset) until the bootloader listens:
# 100 ms reset pulse plus bootloader start-up. Optiboot then waits about 1 s for data.
BOOTLOADER_DELAY_S = 0.25


def find_avrdude(explicit):
    if explicit:
        return explicit
    found = shutil.which("avrdude")
    if found:
        return found
    pio = os.path.expanduser("~/.platformio/packages/tool-avrdude/avrdude")
    if os.path.exists(pio):
        return pio
    sys.exit("avrdude not found; install it or pass --avrdude")


async def run(args):
    key = args.key
    if not key and args.secrets:
        with open(args.secrets) as f:
            key = yaml.safe_load(f).get("api_encryption_key")

    api = aioesphomeapi.APIClient(args.host, args.api_port, None, noise_psk=key)
    await api.connect(login=True)
    print(f"Connected to {args.host}")

    tcp_writer = None
    pending = bytearray()  # avrdude bytes held back until the bootloader is ready
    ready = False

    def on_serial(msg):
        if msg.instance == args.instance and tcp_writer is not None:
            tcp_writer.write(bytes(msg.data))

    unsub_data = api.subscribe_serial_proxy_data(on_serial)
    done = asyncio.Event()

    async def handle(reader, writer):
        nonlocal tcp_writer, ready
        tcp_writer = writer
        await api.serial_proxy_subscribe_await_response(args.instance)
        await api.serial_proxy_configure_await_response(args.instance, args.baud)
        print(f"Serial proxy subscribed at {args.baud} baud; Arduino resetting into bootloader")

        async def release():
            nonlocal ready
            await asyncio.sleep(BOOTLOADER_DELAY_S)
            ready = True
            if pending:
                api.serial_proxy_write(args.instance, bytes(pending))
                pending.clear()

        release_task = asyncio.create_task(release())
        try:
            while data := await reader.read(4096):
                if ready:
                    api.serial_proxy_write(args.instance, data)
                else:
                    pending.extend(data)
        finally:
            release_task.cancel()
            tcp_writer = None
            writer.close()
            done.set()

    server = await asyncio.start_server(handle, "127.0.0.1", 0)
    port = server.sockets[0].getsockname()[1]

    cmd = [find_avrdude(args.avrdude), "-c", "arduino", "-p", "atmega328p",
           "-P", f"net:127.0.0.1:{port}"]
    if args.avrdude_conf:
        cmd += ["-C", args.avrdude_conf]
    cmd += args.avrdude_args
    print("Running:", " ".join(cmd))
    proc = await asyncio.create_subprocess_exec(*cmd)
    rc = await proc.wait()

    # Unsubscribing hands the UART back to gecko_spa, which resets the Arduino again
    await asyncio.wait_for(done.wait(), 5)
    server.close()
    try:
        await api.serial_proxy_configure_await_response(args.instance, 115200)
        await api.serial_proxy_unsubscribe_await_response(args.instance)
    finally:
        unsub_data()
        await api.disconnect()
    print("avrdude exit code", rc)
    return rc


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--host", required=True, help="ESP32 address")
    p.add_argument("--api-port", type=int, default=6053)
    p.add_argument("--key", help="API encryption key (or use --secrets)")
    p.add_argument("--secrets", help="secrets.yaml containing api_encryption_key")
    p.add_argument("--instance", type=int, default=0, help="serial_proxy instance index")
    p.add_argument("--baud", type=int, default=115200,
                   help="bootloader baud: 115200 (new Nano bootloader) or 57600 (old)")
    p.add_argument("--avrdude", help="path to avrdude")
    p.add_argument("--avrdude-conf", help="path to avrdude.conf")
    p.add_argument("avrdude_args", nargs=argparse.REMAINDER, help="extra avrdude args after --")
    args = p.parse_args()
    if args.avrdude_args[:1] == ["--"]:
        args.avrdude_args = args.avrdude_args[1:]
    sys.exit(asyncio.run(run(args)))


if __name__ == "__main__":
    main()
