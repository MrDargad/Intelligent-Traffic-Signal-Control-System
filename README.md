## Intelligent Traffic Signal Control System with Real-time Optimization

To Do :  
  1. Download Repo.
  2. Connect Arduino Uno to PC using connector and Initialize serial communication with Arduino.
  3. Get 3 Traffic Light Sets and attach Pins to Arduino as mentioned in TrafficSignalController.ino line 1-3.
  4. Upload TrafficSignalController.ino to Arduino using Arduino IDE.
  5. In Line 47 of main.py
        "arduino = serial.Serial('COM14', 9600, timeout=1)"
        change COM14 -> Serial Port to which Arduino is connected.
  6. Download IP Webcam on different Mobiles connected to came WIFI as PC and replace with you IP camera URL inside "camera_urls" on line 16 of main.py.
  7. READY -> 3 Windows will open at a interval of 20 secs which will capture feed from respective mobile.
