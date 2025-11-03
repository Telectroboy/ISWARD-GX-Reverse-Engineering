Extraction of dmesg when camera is pluged:

[104242.604942] usb 1-2: new high-speed USB device number 6 using xhci_hcd
[104242.744480] usb 1-2: New USB device found, idVendor=05e3, idProduct=0610, bcdDevice=60.60
[104242.744503] usb 1-2: New USB device strings: Mfr=0, Product=1, SerialNumber=0
[104242.744513] usb 1-2: Product: USB2.0 Hub
[104242.750735] hub 1-2:1.0: USB hub found
[104242.751493] hub 1-2:1.0: 4 ports detected
[104243.040708] usb 1-2.2: new high-speed USB device number 7 using xhci_hcd
[104243.180472] usb 1-2.2: New USB device found, idVendor=2bc5, idProduct=050e, bcdDevice= 1.00
[104243.180496] usb 1-2.2: New USB device strings: Mfr=2, Product=1, SerialNumber=3
[104243.180506] usb 1-2.2: Product: USB 2.0 Camera
[104243.180513] usb 1-2.2: Manufacturer: Sonix Technology Co., Ltd.
[104243.180519] usb 1-2.2: SerialNumber: AU1GC3201K7
[104243.238543] mc: Linux media interface: v0.10
[104243.295809] videodev: Linux video capture interface: v2.00
[104243.351653] usb 1-2.2: Found UVC 1.00 device USB 2.0 Camera (2bc5:050e)
[104243.379280] usbcore: registered new interface driver uvcvideo
[104243.655888] usb 1-2: USB disconnect, device number 6
[104243.680826] usb 1-2.2: USB disconnect, device number 7
[104243.936811] usb 1-2: new high-speed USB device number 8 using xhci_hcd
[104244.076407] usb 1-2: New USB device found, idVendor=05e3, idProduct=0610, bcdDevice=60.60
[104244.076432] usb 1-2: New USB device strings: Mfr=0, Product=1, SerialNumber=0
[104244.076442] usb 1-2: Product: USB2.0 Hub
[104244.079431] hub 1-2:1.0: USB hub found
[104244.080107] hub 1-2:1.0: 4 ports detected
[104244.369026] usb 1-2.2: new high-speed USB device number 9 using xhci_hcd
[104244.508291] usb 1-2.2: New USB device found, idVendor=2bc5, idProduct=050e, bcdDevice= 1.00
[104244.508304] usb 1-2.2: New USB device strings: Mfr=2, Product=1, SerialNumber=3
[104244.508308] usb 1-2.2: Product: USB 2.0 Camera
[104244.508311] usb 1-2.2: Manufacturer: Sonix Technology Co., Ltd.
[104244.508314] usb 1-2.2: SerialNumber: AU1GC3201K7
[104244.521600] usb 1-2.2: Found UVC 1.00 device USB 2.0 Camera (2bc5:050e)
[104244.620816] usb 1-2.4: new high-speed USB device number 10 using xhci_hcd
[104244.712423] usb 1-2.4: New USB device found, idVendor=2bc5, idProduct=060e, bcdDevice= 0.01
[104244.712447] usb 1-2.4: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[104244.712457] usb 1-2.4: Product: ORBBEC Depth Sensor
[104244.712465] usb 1-2.4: Manufacturer: Orbbec(R)
[104249.397892] usb 1-2: USB disconnect, device number 8
[104249.397913] usb 1-2.2: USB disconnect, device number 9
[104249.400441] usb 1-2.4: USB disconnect, device number 10
[104249.680971] usb 1-2: new high-speed USB device number 11 using xhci_hcd
[104249.820069] usb 1-2: New USB device found, idVendor=05e3, idProduct=0610, bcdDevice=60.60
[104249.820094] usb 1-2: New USB device strings: Mfr=0, Product=1, SerialNumber=0
[104249.820104] usb 1-2: Product: USB2.0 Hub
[104249.823872] hub 1-2:1.0: USB hub found
[104249.824570] hub 1-2:1.0: 4 ports detected
[104250.112841] usb 1-2.2: new high-speed USB device number 12 using xhci_hcd
[104250.256781] usb 1-2.2: New USB device found, idVendor=2bc5, idProduct=050e, bcdDevice= 1.00
[104250.256807] usb 1-2.2: New USB device strings: Mfr=2, Product=1, SerialNumber=3
[104250.256820] usb 1-2.2: Product: USB 2.0 Camera
[104250.256828] usb 1-2.2: Manufacturer: Sonix Technology Co., Ltd.
[104250.256836] usb 1-2.2: SerialNumber: AU1GC3201K7
[104250.270577] usb 1-2.2: Found UVC 1.00 device USB 2.0 Camera (2bc5:050e)
[104250.368774] usb 1-2.4: new high-speed USB device number 13 using xhci_hcd
[104250.464540] usb 1-2.4: New USB device found, idVendor=2bc5, idProduct=060e, bcdDevice= 0.01
[104250.464549] usb 1-2.4: New USB device strings: Mfr=1, Product=2, SerialNumber=0
[104250.464552] usb 1-2.4: Product: ORBBEC Depth Sensor
[104250.464554] usb 1-2.4: Manufacturer: Orbbec(R)



USB 2.0 Camera: USB Camera (usb-0000:00:10.0-3.2):
	/dev/video0
	/dev/video1
	/dev/media0
➜  ~ v4l2-ctl --list-devices                          
v4l2-ctl --all -d /dev/video0
v4l2-ctl --all -d /dev/video1

USB 2.0 Camera: USB Camera (usb-0000:00:10.0-3.2):
	/dev/video0
	/dev/video1
	/dev/media0
Driver Info:
	Driver name      : uvcvideo
	Card type        : USB 2.0 Camera: USB Camera
	Bus info         : usb-0000:00:10.0-3.2
	Driver version   : 6.12.48
	Capabilities     : 0x84a00001
		Video Capture
		Metadata Capture
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x04200001
		Video Capture
		Streaming
		Extended Pix Format
Media Driver Info:
	Driver name      : uvcvideo
	Model            : USB 2.0 Camera: USB Camera
	Serial           : AU1GC3201K7
	Bus info         : usb-0000:00:10.0-3.2
	Media version    : 6.12.48
	Hardware revision: 0x00000100 (256)
	Driver version   : 6.12.48
Interface Info:
	ID               : 0x03000002
	Type             : V4L Video
Entity Info:
	ID               : 0x00000001 (1)
	Name             : USB 2.0 Camera: USB Camera
	Function         : V4L2 I/O
	Flags            : default
	Pad 0x01000007   : 0: Sink
	  Link 0x02000013: from remote pad 0x100000a of entity 'Extension 4' (Video Pixel Formatter): Data, Enabled, Immutable
Priority: 2
Video input : 0 (Camera 1: ok)
Format Video Capture:
	Width/Height      : 640/480
	Pixel Format      : 'MJPG' (Motion-JPEG)
	Field             : None
	Bytes per Line    : 0
	Size Image        : 614989
	Colorspace        : sRGB
	Transfer Function : Default (maps to sRGB)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Full Range)
	Flags             : 
Crop Capability Video Capture:
	Bounds      : Left 0, Top 0, Width 640, Height 480
	Default     : Left 0, Top 0, Width 640, Height 480
	Pixel Aspect: 1/1
Selection Video Capture: crop_default, Left 0, Top 0, Width 640, Height 480, Flags: 
Selection Video Capture: crop_bounds, Left 0, Top 0, Width 640, Height 480, Flags: 
Streaming Parameters Video Capture:
	Capabilities     : timeperframe
	Frames per second: 25.000 (25/1)
	Read buffers     : 0

User Controls

                     brightness 0x00980900 (int)    : min=-64 max=64 step=1 default=0 value=0
                       contrast 0x00980901 (int)    : min=0 max=64 step=1 default=32 value=32
                     saturation 0x00980902 (int)    : min=0 max=128 step=1 default=64 value=64
                            hue 0x00980903 (int)    : min=-40 max=40 step=1 default=0 value=0
        white_balance_automatic 0x0098090c (bool)   : default=1 value=1
                          gamma 0x00980910 (int)    : min=72 max=500 step=1 default=100 value=100
                           gain 0x00980913 (int)    : min=0 max=100 step=1 default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=2 default=1 value=1 (50 Hz)
				0: Disabled
				1: 50 Hz
				2: 60 Hz
      white_balance_temperature 0x0098091a (int)    : min=2800 max=6500 step=1 default=4600 value=4600 flags=inactive
                      sharpness 0x0098091b (int)    : min=0 max=6 step=1 default=3 value=3
         backlight_compensation 0x0098091c (int)    : min=0 max=65535 step=1 default=0 value=0

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=3 value=3 (Aperture Priority Mode)
				1: Manual Mode
				3: Aperture Priority Mode
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=5000 step=1 default=157 value=157 flags=inactive
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=1
Driver Info:
	Driver name      : uvcvideo
	Card type        : USB 2.0 Camera: USB Camera
	Bus info         : usb-0000:00:10.0-3.2
	Driver version   : 6.12.48
	Capabilities     : 0x84a00001
		Video Capture
		Metadata Capture
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x04a00000
		Metadata Capture
		Streaming
		Extended Pix Format
Media Driver Info:
	Driver name      : uvcvideo
	Model            : USB 2.0 Camera: USB Camera
	Serial           : AU1GC3201K7
	Bus info         : usb-0000:00:10.0-3.2
	Media version    : 6.12.48
	Hardware revision: 0x00000100 (256)
	Driver version   : 6.12.48
Interface Info:
	ID               : 0x03000005
	Type             : V4L Video
Entity Info:
	ID               : 0x00000004 (4)
	Name             : USB 2.0 Camera: USB Camera
	Function         : V4L2 I/O
Priority: 2
Format Metadata Capture:
	Sample Format   : 'UVCH' (UVC Payload Header Metadata)
	Buffer Size     : 10240
