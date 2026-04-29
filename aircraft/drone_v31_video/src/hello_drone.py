#!/usr/bin/env python3
"""Простейший тест: проверка, что контейнер работает"""
import os, sys, time

def main():
    print("🚀 Drone Video Pipeline v0.1")
    print(f"📁 Workspace: {os.getcwd()}")
    print(f"🔧 Python: {sys.version.split()[0]}")
    print(f"🎯 Архитектура: {os.uname().machine}")
    
    # Эмуляция цикла обработки кадров
    for frame in range(1, 11):
        print(f"📦 Frame #{frame} processed")
        time.sleep(0.5)
    
    print("✅ Test completed successfully!")
    return 0

if __name__ == "__main__":
    sys.exit(main())