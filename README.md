
---

# 🧠 Control of a Differential Mobile Robot using EEG and EMG Signals

## 📝 Abstract

This study introduces a hybrid **EEG-EMG BCI system** for robotic navigation.

* **Motor imagery (EEG)** controls left and right movements.
* **Jaw clenching (EMG)** triggers an emergency stop.
  This system offers an efficient, hands-free interface for assistive robotic applications.

---

## 🎯 Objective

1. Move the robot **left** or **right** using **EEG** (motor imagery).
2. **Stop** the robot using **EMG** (Jaw clenching).

---

## 💡 Motivation

We aim to implement this concept on a **wheelchair** for individuals with **paralysis below the head**, helping to restore their **independence and mobility**.


---

## ⚙️ Methodology

### EEG (Motor Imagery) – OpenBCI

Electrodes placed over the motor cortex:

| Color  | Channel | Function   |
| ------ | ------- | ---------- |
| Green  | CH\_4   | Left Hand  |
| Red    | CH\_7   | Left Hand  |
| Orange | CH\_6   | Right Hand |
| Brown  | CH\_8   | Right Hand |

* **Left hand imagery → Move Left**
* **Right hand imagery → Move Right**

📄 [OpenBCI Data Format Documentation](https://docs.openbci.com/Cyton/CytonDataFormat/)

---

### EMG (Jaw Clenching) – Upside Down Labs

* **Jaw Clench → Stop Robot**

📄 [Muscle BioAmp Shield Documentation](https://docs.upsidedownlabs.tech/hardware/bioamp/muscle-bioamp-shield/index.html#step-6-visualise-emg-signals-on-laptop)

---

### Robot Platform

* **TurtleBot 5**

---

## 🧪 Procedure

* **Data Acquisition** (EEG + EMG)
* **Preprocessing**

  * Bandpass filter (8–30 Hz, mu + beta band)
  * Power Spectral Density (PSD) extraction
* **Model Training**

  * Algorithm: **Random Forest Classifier**
* **Testing**

  * Achieved Accuracy: **78%**
* **Model Deployment** on the robot

---

## 🎮 Classes Used

* **Left Hand**
* **Both Hands**
* **Right Hand**
* **Resting State**

---

## 📊 Results

* **Model Accuracy:** 78%

---

## 💬 Discussion

### 🔴 Limitations

* Requires a **large dataset** for training.
* Needs **adequate user training** with **visual feedback**.
* Faced **time constraints** during development.

---


### Video
[![Watch the video](https://img.youtube.com/vi/B6f8p4evgH8/maxresdefault.jpg)](https://www.youtube.com/watch?v=B6f8p4evgH8)
