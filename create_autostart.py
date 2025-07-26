import os
import subprocess
from pathlib import Path


def create_start_script(app_path: Path, venv_path: Path, script_path: Path):
    print("🔧 Erstelle Startskript...")
    content = f"""#!/bin/bash
source {venv_path}/bin/activate
python3 {app_path}/AppServer.py
"""
    with open(script_path, "w") as f:
        f.write(content)
    os.chmod(script_path, 0o755)
    print(f"✅ Startskript erstellt unter: {script_path}")


def create_systemd_service(script_path: Path, service_name="appserver"):
    print("🛠️  Erstelle systemd-Dienst...")

    service_path = Path(f"/etc/systemd/system/{service_name}.service")
    content = f"""[Unit]
Description=Autostart für AppServer.py
After=network.target

[Service]
Type=simple
ExecStart={script_path}
WorkingDirectory={script_path.parent}
User=pi
Restart=on-failure

[Install]
WantedBy=multi-user.target
"""

    with open(service_path, "w") as f:
        f.write(content)
    print(f"✅ systemd-Service-Datei geschrieben nach: {service_path}")

    # systemd neu laden und Dienst aktivieren
    subprocess.run(["sudo", "systemctl", "daemon-reload"], check=True)
    subprocess.run(["sudo", "systemctl", "enable", service_name], check=True)
    subprocess.run(["sudo", "systemctl", "start", service_name], check=True)
    print("🚀 Dienst gestartet und beim Boot aktiviert.")


def main():
    user_home = Path.home()
    app_dir = Path(__file__).resolve().parent
    venv_path = app_dir / ".env"
    script_path = user_home / "start_appserver.sh"

    create_start_script(app_dir, venv_path, script_path)
    create_systemd_service(script_path)


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("⚠️  Bitte mit sudo ausführen:\n   sudo python3 setup_autostart.py")
    else:
        main()
