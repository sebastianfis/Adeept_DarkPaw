import subprocess
import os
from pathlib import Path


def remove_systemd_service(service_name="appserver"):
    service_path = Path(f"/etc/systemd/system/{service_name}.service")

    print("🧯 Stoppe Dienst, falls er läuft...")
    subprocess.run(["systemctl", "stop", service_name], check=False)

    print("🚫 Deaktiviere Autostart...")
    subprocess.run(["systemctl", "disable", service_name], check=False)

    if service_path.exists():
        print(f"🗑️  Entferne systemd-Service-Datei: {service_path}")
        service_path.unlink()
    else:
        print(f"⚠️  {service_path} existiert nicht (wurde vielleicht schon entfernt).")

    print("🔄 Lade systemd neu...")
    subprocess.run(["systemctl", "daemon-reload"], check=True)
    print("✅ Autostart wurde erfolgreich entfernt.")


def remove_start_script(script_name="start_appserver.sh"):
    script_path = Path.home() / script_name
    if script_path.exists():
        print(f"🗑️  Entferne Startskript: {script_path}")
        script_path.unlink()
    else:
        print(f"⚠️  {script_path} existiert nicht.")


def main():
    remove_systemd_service()
    remove_start_script()


if __name__ == "__main__":
    if os.geteuid() != 0:
        print("⚠️  Bitte mit sudo ausführen:\n   sudo python3 remove_autostart.py")
    else:
        main()