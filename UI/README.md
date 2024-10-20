# Team Dyaus: Ground Station User Interface Guide

This guide outlines the setup and operation of the **Ground Station User Interface (UI)**, developed by **Jaydeep
Solanki** for the **Teknofest 2024 Model Satellite Competition**.

## Team Members

- **Dhir Tejaskumar Gandhi** (Team Captain)
- **Jaydeep Sanjay Solanki**
- **Jishan Abbasbhai Seta**
- **Satyam Alpesh Rana**
- **Keshav Vyas**
- **Vraj Prajapati**

---

## Core Files for the Ground Station UI

1. **[mainwindow.py](./mainwindow.py)**  
   Main script to launch the Ground Station UI. Refer to the [Modifications Section](#modifying-the-main-file) for safe
   editing instructions.

2. **[csv_file_simulation_gui.py](./csv_file_simulation_gui.py)**  
   Simulates satellite telemetry data from a CSV file for testing purposes. See
   the [CSV Simulation Guide](#csv-simulation-guide) for details.

3. **[requirements.txt](./requirements.txt)**  
   Lists the Python dependencies required to run the Ground Station UI. Install these to ensure smooth operation.

> **Note:**  
> Modify other files only if you're familiar with their functionality. Changes can affect the overall performance.

---

## Setup Instructions

### 1. Install Python 3.10 or Higher

Ensure you have **Python 3.10+** installed on your system. Check the version by running the following command in your
terminal:

```bash
python --version
```

### 2. Create a Virtual Environment

A virtual environment ensures project dependencies remain isolated. Here's how to create one:

- Install `virtualenv` (if not already installed):
  ```bash
  pip install virtualenv
  ```

- Create the virtual environment:
  ```bash
  python -m venv venv
  ```

### 3. Activate the Virtual Environment

- **For Windows**:
  ```bash
  venv\Scripts\activate
  ```

- **For macOS/Linux**:
  ```bash
  source venv/bin/activate
  ```

#### Permission Issues?

- **Windows**:  
  Run Command Prompt or PowerShell as **Administrator** and retry activation.

- **macOS/Linux**:  
  Use `sudo` for elevated privileges:
  ```bash
  sudo source venv/bin/activate
  ```

### 4. Install Dependencies

Once the virtual environment is activated, install the required Python packages with:

```bash
pip install -r requirements.txt
```

### 5. Launch the Ground Station UI

To run the Ground Station interface, execute the following command:

```bash
python mainwindow.py
```

The UI may take a few moments to initialize and create a backup folder. Once launched, you can begin monitoring
satellite telemetry data in real-time by clicking start button.

---

## CSV Simulation Guide

To test the Ground Station UI, simulate telemetry data using the CSV Simulation tool. Follow these steps:

1. **Activate the virtual environment** (refer to the previous instructions).
2. Run the CSV simulation script:
   ```bash
   python csv_file_simulation_gui.py
   ```

3. Ensure the CSV follows this format:

   ```csv
   Packet_count,Satellite_Status,Error_Code,Mission_Time,Pressure1,Pressure2,Altitude1,Altitude2,Altitude_Difference,Descent_rate,Temperature,Battery_Voltage,Gps_Latitude,Gps_Longitude,Gps_Altitude,Pitch,Roll,Yaw,LNLN,Iot_Data,Team_Number
   ```

4. **Select the CSV file** by clicking the `Select File` button in the interface. The file path will appear once
   selected.
   <br> ![File Selection](./Readme%20Files/image.png)
   <br> ![File Selection](./Readme%20Files/image-1.png)
5. **Start the simulation** by pressing the "Start" button. The data will now flow into the UI.
   <br> ![Start Simulation](./Readme%20Files/image-2.png)

6. **Monitor the data** in real-time as it updates in the interface.
   <br> ![Data Flow](./Readme%20Files/image-3.png)

---

## Modifying the Main File

### Editing `mainwindow.py`

To safely edit the `mainwindow.py` file:

1. Open the file in a code editor like VS Code or Notepad.


2. Search for the IP address using `Ctrl+F`.
   <br> ![Open File](./Readme%20Files/image-4.png)

3. Replace the existing IP address with the new one by copying the current IP and using `Ctrl+H` (or `Ctrl+R` in
   Notepad) to replace it throughout the file.
   <br> ![IP Replacement](./Readme%20Files/image-5.png)

---

## Troubleshooting

### 1. Dependency Issues

- Ensure you have **Python 3.10+** installed.
- Verify that the virtual environment is activated.
- If problems persist, reinstall dependencies:
  ```bash
  pip install -r requirements.txt
  ```

### 2. UI Not Launching

- Check for missing or corrupted files in `mainwindow.py` or `requirements.txt`.
- Reinstall missing packages:
  ```bash
  pip install -r requirements.txt
  ```

### 3. Virtual Environment Won't Activate

- On **Windows**, try running the command prompt as Administrator.
- On **macOS/Linux**, use `sudo` for elevated privileges.

---

