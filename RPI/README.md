# Team Dyaus: RPI Codes Guide

This guide provides instructions for setting up and operating the **Main Computer of Science Payload (Raspberry Pi)**
for the **Graphical User Interface (GUI)** developed by **Team Dyaus (335592)** for the **Teknofest 2024 Model Satellite
Competition**.

---

## Team Members

- **Dhir Tejaskumar Gandhi** (Team Captain)
- **Jaydeep Sanjay Solanki**
- **Jishan Abbasbhai Seta**
- **Satyam Alpesh Rana**
- **Keshav Vyas**

---

## Core Files for the Ground Station UI

**Note:** Modifications to these files should be performed only by individuals who are familiar with their
functionality. Unauthorized changes may impact overall system performance. It is recommended to avoid editing these
files unless necessary.

1. **[maincode.py](./maincode.py)**  
   This is the primary script for initiating telemetry for the ground station. It waits for a start command from the
   GUI. In the event of a system restart, it resumes from the last packet count and retains initial values, which aids
   in altitude estimation (initial pressure).

2. **[install_setup_libraries.sh](./install_setup_libraries.sh)**  
   Transfer this file to the Raspberry Pi and execute it. This script will automatically install all required libraries.
   After installation, copy `maincode.py` to the Raspberry Pi and run it using the command: `sudo python maincode.py`.

3. **[rtc_set_time.py](./rtc_set_time.py)**  
   Run this script initially to set the RTC time to `Europe/Istanbul`. This ensures that the system maintains accurate
   time even without an internet connection.

---

## Procedure for Running `.sh` Files

1. **Transfer the `.sh` File to the Raspberry Pi:**
    - Use a method such as SCP (Secure Copy Protocol), SSH, or a USB drive to move the `install_setup_libraries.sh` file
      to your Raspberry Pi.

2. **Set Execute Permissions:**
    - Open a terminal on the Raspberry Pi.
    - Navigate to the directory where you have placed the `install_setup_libraries.sh` file.
    - Run the following command to make the script executable:
      ```bash
      chmod +x install_setup_libraries.sh
      ```

3. **Execute the Script:**
    - In the same terminal window, run the script using the following command:
      ```bash
      ./install_setup_libraries.sh
      ```
    - This command will execute the script and automatically install all required libraries.

4. **Verify Installation:**
    - After the script completes, ensure that all necessary libraries are installed by checking for any error messages
      in the terminal. You may also verify individual library installations if required.

---

## Additional Instructions

1. **Running `rtc_set_time.py`:**
    - Open a terminal on the Raspberry Pi.
    - Navigate to the directory where `rtc_set_time.py` is located.
    - Execute the script with the following command:
      ```bash
      sudo python rtc_set_time.py
      ```
    - This will set the Real-Time Clock (RTC) to `Europe/Istanbul` if connected to the internet. Ensure the internet
      connectivity is proper on the Raspberry Pi for setting the RTC time.

2. **Running `maincode.py`:**
    - After installing the libraries and setting the RTC time, navigate to the directory containing `maincode.py`.
    - Run the script with the following command:
      ```bash
      sudo python maincode.py
      ```
    - This will start the telemetry process. The script will wait for a start command from the GUI and will handle
      telemetry data as per the requirements.