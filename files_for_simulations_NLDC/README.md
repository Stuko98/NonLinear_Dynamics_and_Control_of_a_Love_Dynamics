# 💘 Love Dynamics Control Systems – *Gone with the Wind*

This project contains the implementation of several control systems for a nonlinear system describing **Love Dynamics**, inspired by the movie *Gone with the Wind*.

The goal is to compare the behavior of the controlled system under nominal conditions and in the presence of disturbances, uncertainties, or unmodelled dynamics.

> 🧠 **Tiny honesty disclaimer:** this README is not entirely the result of my own heroic typing skills.  
> A friendly AI assistant gave me a hand in making it cleaner, more ordered, and slightly less chaotic.  
> The control systems, however, are still where the real drama happens.

---

## 🎛️ Implemented Controllers

The following control systems have been implemented:

1. **Linear Controller**
   - State Feedback Controller with Integral Action

2. **IOFBL Controller**
   - Input-Output Feedback Linearization Controller

3. **ISFBL Controller**
   - Input-State Feedback Linearization Controller

4. **SMC Controller**
   - Sliding Mode Control

5. **MCS Controller**
   - Minimal Control Synthesis

---

## ⚙️ Simulation Scenarios

Each controller can be tested under different system configurations:

- **Nominal system**
- **External plant disturbance**
- **Unmodelled dynamics**
- **Parametric variation**
  - 10% parametric variation on the parameter:

    ```
    k = 15
    ```

It is also possible to apply a **combination** of disturbances, uncertainties, and unmodelled dynamics to the control system.

---

## 🚀 How to Run a Simulation

### 1. Extract the simulation files

Extract the entire content of the ZIP archive:

```
files_for_simulations_NLDC.zip
```

Make sure that all folders and files are extracted before opening MATLAB or Simulink.

---

### 2. Choose the controller to test

Select the folder corresponding to the control system you want to simulate:

| Controller | Folder |
|---|---|
| **Linear Controller** | `nl_lin_ref` |
| **IOFBL Controller** | `iofbl` |
| **ISFBL Controller** | `isfbl` |
| **SMC Controller** | `smc` |
| **MCS Controller** | `mrac_mcs` |

---

### 3. Choose the system configuration in Simulink

Open the `.slx` file related to the selected controller and configure the system according to the desired scenario.

---

#### ✅ Nominal system

1. Open the corresponding `.slx` file.
2. Set the **Multiport Switch** in the `plant disturbances` area to:

   ```
   no disturbance
   ```

   corresponding to **channel 1**.

3. Double-click on the block:

   ```
   nonlinear system
   ```

4. Follow the instructions inside the block.

---

#### 🧩 System with unmodelled dynamics

1. Open the corresponding `.slx` file.
2. Set the **Multiport Switch** in the `plant disturbances` area to:

   ```
   no disturbance
   ```

   corresponding to **channel 1**.

3. Double-click on the block:

   ```
   nonlinear system
   ```

4. Follow the instructions inside the block to enable the **unmodelled dynamics**.

---

#### 📉 System with parametric uncertainty on parameter `k`

1. Open the corresponding `.slx` file.
2. Set the **Multiport Switch** in the `plant disturbances` area to:

   ```
   no disturbance
   ```

   corresponding to **channel 1**.

3. Double-click on the block:

   ```
   nonlinear system
   ```

4. Follow the instructions inside the block to apply a **10% parametric variation** on the parameter:

   ```
   k = 15
   ```

---

#### 🌊 System with plant disturbance `d(t)`

1. Open the corresponding `.slx` file.
2. Set the **Multiport Switch** in the `plant disturbances` area to the desired disturbance:

   ```
   channel 2 -> delayed steps
   channel 3 -> delayed sine waves
   channel 4 -> delayed band-limited white noises
   ```

3. Double-click on the block:

   ```
   nonlinear system
   ```

4. Follow the instructions inside the block.

---

## ▶️ Open the Initialization File

After configuring the Simulink model:

1. Close the `.slx` file.
2. Open the corresponding MATLAB initialization file:

   ```
   init_....m
   ```

3. Run the file by clicking the **Run** icon.

---

## 📌 Quick Summary

```text
1. Extract the entire content of files_for_simulations_NLDC.zip
2. Choose the controller to test
3. Open the folder of the selected controller
4. Open the corresponding .slx file
5. Configure possible disturbances, uncertainties, or unmodelled dynamics
6. Double-click on the nonlinear system block
7. Follow the instructions inside the block
8. Close the .slx file
9. Open the init_....m file
10. Click Run
11. Enjoy the show
```

---

## 🎬 Enjoy

Once the system has been configured and the initialization file has been executed:

```
Enjoy the show.
```
