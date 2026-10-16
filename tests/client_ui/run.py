#!/usr/bin/env python3
"""Compile real UI/parser/Topic sources with UART/time substitutes; never changes firmware presets."""
from pathlib import Path
import subprocess

root = Path(__file__).resolve().parents[2]
tests = Path(__file__).resolve().parent
build = root / "build" / "tests" / "client_ui"
build.mkdir(parents=True, exist_ok=True)
referee = root / "User_File/Device/Peripheral/Referee"
app = root / "User_File/Application/ClientUI"
center = root / "User_File/System/MessageCenter"
includes = [f"-I{p}" for p in (tests / "stubs", referee, app, center)]
flags = ["-g", "-O1", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-fno-pie"]
sources = [referee / "crc_ref.c", referee / "referee_26.c", referee / "referee_ui_sender.c",
           app / "ClientUI.cpp", app / "ClientUI_Capture.cpp", center / "message_center.cpp", tests / "test_client_ui.cpp"]
objects = []
for source in sources:
    obj = build / (source.name + ".o")
    compiler, standard = ("gcc", "-std=c11") if source.suffix == ".c" else ("g++", "-std=c++17")
    subprocess.run([compiler, standard, *flags, *includes, "-c", str(source), "-o", str(obj)], check=True)
    objects.append(str(obj))
binary = build / "client_ui_test"
subprocess.run(["g++", *flags, "-no-pie", *objects, "-o", str(binary)], check=True)
for case in ("protocol", "identity", "capture", "layout", "schedule", "expiry", "recovery"):
    subprocess.run([str(binary), case], check=True)
