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
        subprocess.check_call([sys.executable, "-m", "venv", str(env_path), "--system-site-packages"])
    else:
        print(f"Virtual Environment {env_path} existiert bereits.")


def install_requirements(env_path, requirements_file):
    os.system("python -m ensurepip --upgrade")
    pip_path = env_path / "bin" / "pip"
    print(f"Installiere Requirements aus {requirements_file}...")
    subprocess.check_call([str(pip_path), "install", "-r", str(requirements_file)])


def update_pci_config():
    config_path = "/boot/firmware/config.txt"
    line = "dtparam=pciex1_gen=3"

    try:
        with open(config_path, "r") as f:
            if line in f.read():
                print("PCIe-Einstellung bereits gesetzt.")
                return
    except FileNotFoundError:
        print(f"{config_path} nicht gefunden!")
        return

    print("Füge PCIe-Einstellung hinzu...")
    os.system(f'echo "{line}" | sudo tee -a {config_path}')

def enable_spi():
    print("Aktiviere SPI Interface...")
    os.system("sudo raspi-config nonint do_spi 0")


def system_update():
    os.system("sudo apt update && sudo apt full-upgrade -y")
    os.system("sudo apt-get -y clean")
    os.system("sudo apt-get -y autoremove")

def install_gstreamer():
    os.system("sudo apt install -y gstreamer1.0-nice")
    os.system("""sudo apt install -y \
        gstreamer1.0-plugins-bad \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-base \
        gstreamer1.0-libav""")


def install_hailo():
    os.system("sudo apt update")
    os.system("sudo apt install -y dkms")
    os.system("sudo apt install -y hailo-all")
    os.system("sudo apt-mark hold dkms hailo-all")
    os.system("sudo apt-mark hold hailort hailo-dkms hailo-tappas-core")


def install_hailo_pci_driver():
    print("🔧 Building Hailo PCIe driver v4.23.0 from source...")

    # Save original working directory
    original_cwd = os.getcwd()

    # Install build dependencies
    os.system("sudo apt install -y git build-essential raspberrypi-kernel-headers dkms")

    # Clone repo & checkout exact version
    os.system("cd /tmp && rm -rf hailort-drivers && git clone https://github.com/hailo-ai/hailort-drivers.git")
    os.system("cd /tmp/hailort-drivers && git checkout v4.23.0")

    # Build & install driver
    os.system("cd /tmp/hailort-drivers/linux/pcie && make all && sudo make install")

    # Load kernel module
    os.system("sudo depmod -a")
    os.system("sudo modprobe hailo_pci")

    # Return to original directory
    os.chdir(original_cwd)

    print("✅ Hailo PCIe driver v4.23.0 installed.")


def main():
    this_path = Path(__file__).resolve().parent
    venv_path = this_path / ".env"
    requirements_file = this_path / "requirements.txt"

    # update_pci_config()
    system_update()
    enable_spi()
    update_pci_config()

    install_gstreamer()
    install_hailo()
    install_hailo_pci_driver()

    create_virtualenv(venv_path)

    install_requirements(venv_path, requirements_file)

    print(f"\n✅ Setup abgeschlossen. Aktiviere das venv mit:\n  source {venv_path}/bin/activate")


if __name__ == "__main__":
    main()
