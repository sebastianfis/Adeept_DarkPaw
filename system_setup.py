import os
import sys
import subprocess
from pathlib import Path


# Function to check if virtualenv is installed
def check_virtualenv():
    try:
        subprocess.check_call([sys.executable, '-m', 'virtualenv', '--version'])
    except subprocess.CalledProcessError:
        print("virtualenv not found. Installing...")
        subprocess.check_call([sys.executable, '-m', 'pip', 'install', 'virtualenv'])


# Function to create a virtual environment
def create_virtualenv(env_path):
    if not env_path.exists():
        print(f"Erstelle Virtual Environment unter {env_path}")
        subprocess.check_call([sys.executable, "-m", "venv", str(env_path)])
    else:
        print(f"Virtual Environment {env_path} existiert bereits.")


def install_packages(env_path, package_list):
    pip_path = env_path / "bin" / "pip"
    for pkg in package_list:
        print(f"Installiere {pkg}...")
        subprocess.check_call([str(pip_path), "install", pkg])


def update_pci_config():
    config_path = Path("/boot/firmware/config.txt")
    if config_path.exists():
        with config_path.open("r+") as f:
            lines = f.readlines()
            if "dtparam=pciex1_gen=3\n" not in lines:
                print("Füge PCIe-Einstellung hinzu...")
                f.write("\ndtparam=pciex1_gen=3\n")
            else:
                print("PCIe-Einstellung bereits gesetzt.")
    else:
        print(f"{config_path} nicht gefunden!")


def system_update():
    os.system("sudo apt update && sudo apt full-upgrade -y")
    os.system("sudo apt-get -y clean")
    os.system("sudo apt-get -y autoremove")


def install_hailo():
    os.system("sudo apt install -y hailo-tappas-core=3.29.1 hailort=4.18.0 hailo-dkms=4.18.0-2")
    os.system("sudo apt-mark hold hailo-tappas-core hailort hailo-dkms")


def download_hailort_wheel():
    wheel_url = "http://dev-public.hailo.ai/2025_01/hailort-4.18.0-cp311-cp311-linux_aarch64.whl"
    target_path = Path.home() / "Downloads" / "hailort-4.18.0-cp311-cp311-linux_aarch64.whl"
    os.system(f'wget "{wheel_url}" -O "{target_path}"')
    return str(target_path)


def main():
    this_path = Path(__file__).resolve().parent
    venv_path = this_path / ".env"

    update_pci_config()
    system_update()
    install_hailo()
    wheel_file = download_hailort_wheel()

    create_virtualenv(venv_path)

    install_packages(venv_path, [wheel_file])  # hailort .whl

    packages = [
        "numpy==1.23.5",
        "opencv-python-headless==4.10.0.82",
        "requests",
        "rpi5_ws2812",
        "loguru",
        "aiohttp[speedups]",
        "supervision==0.19.0"
    ]
    install_packages(venv_path, packages)

    print(f"\n✅ Setup abgeschlossen. Aktiviere das venv mit:\n  source {venv_path}/bin/activate")


if __name__ == "__main__":
    main()
