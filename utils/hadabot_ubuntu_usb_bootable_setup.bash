#! /bin/bash

# With a terminal from Ubuntu, run "wget -O hadabot_setup.bash https://raw.githubusercontent.com/hadabot/hadabot_main/master/utils/hadabot_ubuntu_usb_bootable_setup.bash"

# "Try Ubuntu" mounts 4gb to file location where docker lives. So we will create a symbolic link to a different location with more storage space
sudo mkdir /var/log/var_lib_docker
cd /var/lib
sudo ln -s ../log/var_lib_docker docker
cd ~

sudo add-apt-repository universe
sudo apt-get update

# Install docker
sudo apt-get install -y \
    apt-transport-https \
    ca-certificates \
    curl \
    gnupg-agent \
    software-properties-common
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo apt-key add -
sudo add-apt-repository \
   "deb [arch=amd64] https://download.docker.com/linux/ubuntu \
   $(lsb_release -cs) \
   stable"
sudo apt-get update
sudo apt-get install -y docker-ce docker-ce-cli containerd.io
sudo usermod -aG docker ubuntu

# "Try Ubuntu" does not like it when docker users aufs as the Docker storage driver
# When using default aufs, "docker run hello-world" will cause a weird invalid arg aufs error.
sudo cp /lib/systemd/system/docker.service /lib/systemd/system/docker.service.orig
sed 's/ExecStart=\/usr\/bin\/dockerd -H/ExecStart=\/usr\/bin\/dockerd --storage-driver=devicemapper -H/g' /lib/systemd/system/docker.service.orig > docker.service
sudo mv docker.service /lib/systemd/system/docker.service
sudo systemctl daemon-reload
sudo systemctl restart docker.service

# Install docker-compose
sudo curl -L "https://github.com/docker/compose/releases/download/1.27.4/docker-compose-$(uname -s)-$(uname -m)" -o /usr/local/bin/docker-compose
sudo chmod +x /usr/local/bin/docker-compose

# Install necessary python components
sudo apt-get install -y python3-pip

# Install the tools needed to flash the ESP32
echo "export PATH=$PATH:~/.local/bin" >> ~/.bashrc
pip3 install -U esptool adafruit-ampy
wget -O esp32-micropython.bin https://micropython.org/resources/firmware/esp32-idf3-20200902-v1.13.bin
sudo usermod -aG dialout ubuntu

# Clone the hadabot repo
git clone https://github.com/hadabot/hadabot_main.git

echo ""
echo "------------------"
echo ""
echo "We are ready for step 2 of the Hadabot Bootable USB setup."
echo ""
echo "1. Using a quality micro-USB cable, connect your ESP32 to your computer."
echo ""
echo "2. Once you have the ESP32 connected, execute the next setup script by typing the command:"
echo ""
echo "    $ bash hadabot_main/utils/hadabot_ubuntu_bootable_usb_setup_part_02.bash"
echo ""

# Login to another shell to activate dialout and docker permission group
su ubuntu
