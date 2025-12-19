sudo apt update && sudo apt full-upgrade -y

# time-fix & locale
sudo apt install -y ntpdate
sudo ntpdate pool.ntp.org

sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8 ru_RU ru_RU.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo 'export LANG=en_US.UTF-8' >> ~/.bashrc
echo 'export LC_ALL=en_US.UTF-8' >> ~/.bashrc

# Настройка конфигурации boot для Hailo-8 Lite
sudo lspci -vvv | grep -i "LnkSta\|Hailo"
lspci | grep -i hailo

sudo nano /boot/firmware/config.txt
dtparam=pciex1
dtparam=pciex1_gen=3

dtparam=fan_temp0=60000
dtparam=fan_temp0_hyst=5000
dtparam=fan_temp0_speed=125
dtoverlay=disable-bt
# &&&(пока не ясно нужно ли и сколько памяти выделять)
gpu_mem=16

# Обновление прошивки RPi(если требуется)
sudo rpi-eeprom-update
sudo apt install -y rpi-eeprom
sudo rpi-eeprom-update -a

# swap 4GB
free -h
sudo fallocate -l 4G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile

echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

sudo reboot

# проверка 
free -h
sudo lspci -vvv | grep -i "LnkSta\|Hailo"

# Настройка параметров свопа(опционально)
sudo nano /etc/sysctl.conf
vm.swappiness=10
vm.vfs_cache_pressure=50
sudo sysctl -p


# Сборка драйвера ядра
# Установка зависимостей для сборки и !заголовки ядра
sudo apt install -y build-essential dkms linux-headers-$(uname -r) git wget curl cmake
git clone https://github.com/hailo-ai/hailort-drivers.git

cd ~/hailort-drivers
git checkout hailo8
# Скачивание прошивки
chmod +x download_firmware.sh
./download_firmware.sh
sudo mkdir -p /lib/firmware/hailo
sudo cp ~/hailort-drivers/hailo8_fw.4.23.0.bin /lib/firmware/hailo/hailo8_fw.bin
# Проверка наличия прошивки
ls -la /lib/firmware/hailo/
# Перезагрузка
sudo rmmod hailo_pci
sudo modprobe hailo_pci

cd linux/pcie
sudo apt install -y build-essential linux-headers-$(uname -r)
make all
sudo make install

# Настройка размера страницы для RPi5
echo "options hailo_pci force_desc_page_size=4096" | sudo tee /etc/modprobe.d/hailo_pci.conf

# Автозагрузка модуля
echo "hailo_pci" | sudo tee /etc/modules-load.d/hailo.conf

# Создать правило udev для доступа без sudo
echo 'KERNEL=="hailo*", MODE="0666"' | sudo tee /etc/udev/rules.d/99-hailo.rules
sudo udevadm control --reload-rules
sudo udevadm trigger
ls -la /dev/hailo*

# Перезагрузка
sudo rmmod hailo_pci
sudo modprobe hailo_pci

# Проверка
lsmod | grep hailo
sudo dmesg | grep -i hailo
ls -la /dev/hailo*

sudo reboot

### Установка HailoRT Runtime

Шаг 1: Скачивание HailoRT
https://hailo.ai/developer-zone/software-downloads/?product=ai_accelerators&device=hailo_8_8l

HailoRT – Ubuntu package (deb) for arm64

scp "C:\Users\Admin\Downloads\hailort_4.23.0_arm64.deb" pi@raspberrypi:/home/pi/
ls -la hailort_*.deb

# Установка HailoRT
sudo dpkg -i hailort_*_arm64.deb
sudo apt install -f -y

hailortcli fw-control identify

# Ожидаемый вывод:
Executing on device: 0000:01:00.0
Identifying board
Control Protocol Version: 2
Firmware Version: 4.23.0 (release,app,extended context switch buffer)
Logger Version: 0
Board Name: Hailo-8
Device Architecture: HAILO8L
Serial Number: HLDDLBB243501312
Part Number: HM21LB1C2LAE
Product Name: HAILO-8L AI ACC M.2 B+M KEY MODULE EXT TMP