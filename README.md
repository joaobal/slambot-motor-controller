# Slambot motor controller

Dual Motor Control with DRV8833 and Quadrature Encoders. Arduino's PID library is used to track the desired speed.
It can also receive commands for desired speeds from Serial2.

<video src="https://github.com/user-attachments/assets/7352acf8-bd2c-49ef-9ffd-7940c12b713f" width="500" autoplay loop controls></video>

(Plot legend):

blue/green: Desired speeds in RPM for motors A and B

orange/yellow: Actual speeds in RPM of motors A and B

## Tests

Tested on an ESP32-WROVER connected to a DRV8833 while receiving commands from a Jetson Nano.

<details>
<summary><strong>Motor and encoder</strong></summary>
<br>
<video src="https://github.com/user-attachments/assets/034389ae-8e27-44af-abb2-48d03f2daa28" width="50" controls></video>
</details>

<details>
<summary><strong>Test 1: Fixed target speed at 150 rpm</strong></summary>
<br>
<video src="https://github.com/user-attachments/assets/15d31798-dd34-4917-aaf1-a3f835ccd7e8" width="500" controls></video>
</details>

<details>
<summary><strong>Test 2: Ki too high, causing overshoot (Ki=8)</strong></summary>
<br>
<video src="https://github.com/user-attachments/assets/503d3979-9e19-4a31-a9ac-fd471540c5ee" width="500" controls></video>
</details>

<details>
<summary><strong>Test 3: Linear changing target speeds</strong></summary>
<br>
<video src="https://github.com/user-attachments/assets/6b8aa7f4-847b-4749-82fc-5ccdda8520d9" width="500" controls></video>
</details>

<details>
<summary><strong>Test 4: Sending commands from Jetson to ESP32</strong></summary>
<br>
<video src="https://github.com/user-attachments/assets/7352acf8-bd2c-49ef-9ffd-7940c12b713f" width="500" controls></video>
</details>
