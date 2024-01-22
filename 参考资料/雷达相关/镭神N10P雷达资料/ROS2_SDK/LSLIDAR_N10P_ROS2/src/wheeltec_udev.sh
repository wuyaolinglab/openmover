echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller.rules
echo  'KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60",ATTRS{serial}=="0001", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="0002", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_controller"' >/etc/udev/rules.d/wheeltec_controller2.rules
echo  'KERNEL=="ttyCH343USB*", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d4",ATTRS{serial}=="54B8001974", MODE:="0777", GROUP:="dialout", SYMLINK+="wheeltec_lidar"' >/etc/udev/rules.d/wheeltec_lidar2.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="GENERAL WEBCAM",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="GENERAL WEBCAM: GENERAL WEBCAM",ATTR{index}=="0",MODE:="0777",SYMLINK+="RgbCam"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Astra Pro HD Camera: Astra Pro ",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Pro"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="USB 2.0 Camera: USB Camera",ATTR{index}=="0",MODE:="0777",SYMLINK+="Astra_Dabai"' >>/etc/udev/rules.d/camera.rules
echo  'SUBSYSTEM=="video4linux",ATTR{name}=="Intel(R) RealSense(TM) Depth Ca",ATTR{index}=="0",MODE:="0777",SYMLINK+="realsense"' >>/etc/udev/rules.d/camera.rules
service udev reload
sleep 2
service udev restart


