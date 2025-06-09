## Technical Document: Autonomous Flight Controller for Fixed-Wing UAVs based on Cube Orange+ and STANAG 4671 Airworthiness Requirements

**Document ID:** UAV-FC-STANAG-2023-001
**Version:** 1.0
**Date:** October 26, 2023

---

**Table of Contents**

1.  Introduction
2.  Autonomous Flight Controller System Description (Cube Orange+)
    2.1. System Architecture Overview
    2.2. Core Components: Cube Orange+ Flight Controller
    2.3. Flight Control System (FCS) Software (ArduPilot)
3.  Autonomous Flight Modes
    3.1. Auto Take-Off
    3.2. Return-to-Land (RTL)
    3.3. Automatic Flight Stabilization
4.  Self-Positioning Systems
    4.1. GPS (Global Positioning System)
    4.2. Optical Flow Sensors
    4.3. Sensor Fusion for Enhanced Positioning
5.  Radio Control (RC) Mode
    5.1. Remote Control Operation
    5.2. Long-Range Transmitter Integration
    5.3. Safety and Failsafe Protocols
6.  Certification Process by STANAG 4671
    6.1. Overview of STANAG 4671
    6.2. Certification Process and Applicability
    6.3. Compliance Considerations for Key System Aspects
        6.3.1. General Airworthiness (Subpart A)
        6.3.2. UAV Flight Performance (Subpart B)
        6.3.3. UAV Structure (Subpart C)
        6.3.4. Equipment and System Safety (Subpart F)
        6.3.5. Command and Control Data Link (Subpart H)
        6.3.6. UAV Control Station (Subpart I)
    6.4. Documentation and Traceability
7.  Conclusion
8.  References
9.  Glossary

---

### 1. Introduction

This document describes the technical specifications and operational capabilities of an autonomous flight controller for a fixed-wing Unmanned Aerial Vehicle (UAV), leveraging the Cube Orange+ as its central processing unit. The primary objective is to detail the system's functionalities, including autonomous flight modes, self-positioning capabilities, and radio control operation. Furthermore, this document outlines the rigorous process for achieving airworthiness certification in accordance with NATO Standardization Agreement (STANAG) 4671, "Unmanned Aerial Vehicle Systems Airworthiness Requirements (USAR)," highlighting the relevant sections of the standard that govern the design, construction, and operation of such a system.

### 2. Autonomous Flight Controller System Description (Cube Orange+)

#### 2.1. System Architecture Overview

The autonomous flight control system forms the brain of the fixed-wing UAV, responsible for interpreting sensor data, executing flight commands, and maintaining stable flight. It integrates various hardware components and runs sophisticated software to enable both autonomous and remotely controlled operations.

#### 2.2. Core Components: Cube Orange+ Flight Controller

The Cube Orange+, a high-performance flight controller from CubePilot (formerly Pixhawk), serves as the central processing unit (CPU) for the UAV's flight control system. Its key features include:
*   **Processor:** STM32H753, a high-performance ARM Cortex-M7 microcontroller, providing substantial processing power for complex flight algorithms, sensor fusion, and mission management.
*   **Redundant IMUs:** Features triple redundant Inertial Measurement Units (IMUs), consisting of accelerometers, gyroscopes, and magnetometers. This redundancy significantly enhances reliability and fault tolerance, critical for airworthiness certification.
*   **Redundant Barometers:** Includes dual barometers for accurate altitude measurement and redundancy.
*   **Vibration Isolation:** The Cube Orange+ is designed with a robust vibration isolation system, protecting its sensitive IMUs from engine and airframe vibrations, which is crucial for stable flight and accurate sensor readings.
*   **Peripheral Connectivity:** Comprehensive set of ports for connecting external sensors (GPS, optical flow, airspeed), radio telemetry, ESCs, servos, and payload components.
*   **Robustness:** Built for demanding applications, offering enhanced reliability over consumer-grade flight controllers.

#### 2.3. Flight Control System (FCS) Software (ArduPilot)

The Cube Orange+ typically runs the open-source ArduPilot firmware, a highly capable and mature software suite designed for various UAV platforms, including fixed-wing aircraft. ArduPilot provides the core algorithms for flight stabilization, navigation, and execution of autonomous missions. Its open-source nature allows for extensive customization and rigorous community-driven testing, contributing to its reliability.

### 3. Autonomous Flight Modes

The flight controller supports a range of autonomous flight modes, enabling the UAV to perform complex missions with minimal human intervention, while incorporating safety protocols. These modes are often categorized under the "automatic" or "autonomous" definitions in STANAG 4671 (Annex A - Glossary, page A-1):
*   **Automatic:** "The execution of a predefined process or event that requires UAV System crew initiation."
*   **Autonomous:** "The execution of predefined processes or events that do not require direct UAV System crew initiation and/or intervention."

#### 3.1. Auto Take-Off

This mode automates the entire take-off sequence, from ground roll to a safe climb to a pre-defined altitude.
*   **Process:** Upon activation, the flight controller automatically applies throttle, manages rudder and elevator inputs to maintain a straight ground roll, initiates rotation at the calculated speed (`VR` - USAR.51(a)), and executes a climb to a designated safe altitude (`15m (50ft)` - USAR.53(b)).
*   **Parameters:** Key parameters such as take-off speed, climb gradient, and initial turn radius are pre-programmed or dynamically calculated based on environmental conditions and UAV weight.
*   **STANAG 4671 Reference:** USAR.U1490(e) "Take-off" explicitly defines the automatic take-off mode, including control of "UAV runway steering flightpath, speed, configuration, engine settings and UAV flightpath after lift off." USAR.53 "Take-off Performance" details the requirements for determining take-off distances and climb to a safe height.

#### 3.2. Return-to-Land (RTL)

RTL is a critical autonomous mode designed for safe recovery of the UAV in various scenarios, including mission completion, low battery, or data link loss.
*   **Process:** The UAV navigates to a pre-defined "home" location, loiters at a specified altitude, then executes a controlled descent and landing approach. The landing sequence may involve a standard approach, or in some cases, a parachute deployment (USAR.U290 "UAV performance before parachute landing").
*   **Failsafe Integration:** This mode is often activated as a failsafe when the command and control data link is lost (USAR.U1613 "Command and control data link loss strategy"). The flight controller executes a pre-programmed "lost link" strategy to either return to home or perform an emergency landing.
*   **STANAG 4671 Reference:** USAR.1412 "Emergency recovery capability" describes the requirement for such a function to "mitigate the effects of critical failures with the intent of minimising the risk to third parties." USAR.U1490(f) "Landing" specifies the automatic landing mode, controlling approach, landing, and ground roll. USAR.75 "Landing Distance" outlines the performance requirements for landing.

#### 3.3. Automatic Flight Stabilization

This is the foundational autonomous capability, ensuring the UAV maintains a stable attitude, altitude, and heading, despite external disturbances like wind gusts.
*   **Process:** The flight controller continuously uses IMU and barometer data to measure the UAV's attitude (pitch, roll, yaw), altitude, and air speed. It then issues commands to the control surfaces (ailerons, elevator, rudder) and throttle to maintain the desired flight parameters.
*   **Sub-modes:** This mode often encompasses sub-modes like "Altitude Hold," "Loiter" (position hold), and "Heading Hold."
*   **STANAG 4671 Reference:** USAR.141(a) "Flight Characteristics: General" states that "When operated in the automatic control mode the UAV should be shown to have acceptable controllability, manoeuvrability and stability characteristics throughout the flight envelope protection...without requiring exceptional skill or alertness from the UAV crew." USAR.171 "Stability: General" details requirements for longitudinal, directional, and lateral stability. USAR.1329 "Flight control system" categorizes "automatic" mode where "UAV attitude, speed and flight path are fully controlled by the flight control system."

### 4. Self-Positioning Systems

Accurate self-positioning is fundamental for autonomous flight. The system utilizes a combination of sensors to provide robust and redundant positioning data.

#### 4.1. GPS (Global Positioning System)

*   **Function:** GPS is the primary sensor for global positioning, providing precise latitude, longitude, and altitude data. It also provides ground speed and course information.
*   **Integration:** The Cube Orange+ integrates multiple GPS receivers for redundancy and improved accuracy. RTK (Real-Time Kinematic) GPS can be employed for centimeter-level accuracy, essential for precise take-off and landing.
*   **Reliance for Navigation:** Crucial for waypoint navigation, geofencing, and returning to a specified home location.
*   **STANAG 4671 Reference:** While not explicitly stating GPS, USAR.1459(a)(1) "UAV onboard flight recorders" requires that flight recorders are supplied with "airspeed, altitude, and directional data obtained from sources that meet the accuracy requirements of USAR.1323, USAR.1325 and USAR.1327," which GPS significantly contributes to for navigation. USAR.1723(a)(4) "Flight and navigation data" requires UAV position to be continuously displayed on a map.

#### 4.2. Optical Flow Sensors

*   **Function:** Optical Flow sensors measure the apparent motion of the ground relative to the UAV, providing highly accurate horizontal velocity and displacement data, particularly at low altitudes. This is crucial for precise hovering and drift correction in GPS-denied or poor GPS signal environments.
*   **Integration:** Often coupled with a downward-facing lidar (Light Detection and Ranging) sensor to provide accurate altitude measurements, complementing the optical flow's horizontal data.
*   **Applications:** Ideal for operations requiring precise low-altitude maneuvers, such as automated landing, close-quarters inspection, or indoor flight where GPS is unavailable.
*   **Limitations:** Requires a textured ground surface to function effectively and is sensitive to lighting conditions.

#### 4.3. Sensor Fusion for Enhanced Positioning

The flight controller employs advanced sensor fusion algorithms (e.g., Extended Kalman Filter - EKF) to combine data from GPS, IMUs, barometers, and optical flow sensors. This fusion provides a more robust, accurate, and continuous estimate of the UAV's position, velocity, and attitude than any single sensor could provide alone. In case of a temporary loss of one sensor (e.g., GPS signal loss), the system can continue to navigate using other available data, enhancing reliability and safety.

### 5. Radio Control (RC) Mode

The RC mode provides a direct, real-time control interface for the UAV, enabling manual piloting by a human operator. This mode is essential for initial setup, flight testing, emergency intervention, and situations requiring immediate human dexterity.

#### 5.1. Remote Control Operation

*   **Control Link:** The UAV is controlled via a radio frequency (RF) link using a long-range transmitter. This link transmits pilot commands (e.g., stick movements for pitch, roll, yaw, throttle) to the flight controller.
*   **Override Capability:** The RC mode functions as an override to autonomous modes, allowing a human pilot to take immediate control of the UAV at any point during the flight (USAR.1329(d) specifies conditions for UAV crew intervention).
*   **Semi-Automatic Control:** The flight controller operates in a "semi-automatic" mode (USAR.1329(a)(2)), where the pilot commands outer loop parameters (like altitude, heading, airspeed), and the FCS executes the necessary control surface movements.

#### 5.2. Long-Range Transmitter Integration

*   **Range and Reliability:** The choice of a long-range transmitter ensures control over significant distances, vital for military or large-scale civilian operations. The reliability of this command and control data link is paramount.
*   **Data Link:** The RC link is part of the overall "command and control data link" (USAR.U1601(b)), which transmits UAV crew commands (uplink) and receives UAV status data (downlink).
*   **Latency:** The "time delays in the command and control data link (namely ‘latency’) shall be specified in the UAV System Flight Manual" (USAR.U1611(a)) to ensure safe operation.

#### 5.3. Safety and Failsafe Protocols

*   **Loss of RC Link:** In the event of a lost RC link, the flight controller immediately initiates pre-programmed failsafe procedures, such as activating the RTL mode or initiating a controlled landing. This is part of the overall "command and control data link loss strategy" (USAR.U1613).
*   **Intervention Mechanism:** The system is designed to allow for "manual abort functions" for automatic take-off and landing (USAR.U1492), emphasizing the human pilot's role in critical situations.
*   **Continuous Control:** USAR.U1617(c) states that "The UAV must be under continuous positive control at all times during switchover and when changing data links from the same UCS or it shall be shown that no positive control will not lead to unsafe conditions."

### 6. Certification Process by STANAG 4671

#### 6.1. Overview of STANAG 4671

STANAG 4671, "Unmanned Aerial Vehicle Systems Airworthiness Requirements (USAR)," is a NATO Standardization Agreement that establishes a baseline set of airworthiness standards for the design and construction of fixed-wing military UAVs, primarily those with a maximum take-off weight between 150 kg and 20,000 kg. Its aim is to ensure a common safety baseline across NATO nations and facilitate streamlined approval for UAVs to operate in non-segregated airspace of other NATO countries (STANAG page 5, AIM point 1 & AGREEMENT point 4).

The document is structured into two main books:
*   **Book 1 – Airworthiness Code:** Contains the mandatory technical airworthiness requirements (USAR paragraphs).
*   **Book 2 – Acceptable Means of Compliance (AMC):** Provides non-exclusive methods and guidance for demonstrating compliance with the USAR requirements.

#### 6.2. Certification Process and Applicability

The certification process to STANAG 4671 is equivalent to civil aircraft Type Certification, adapted for UAVs. It involves:
1.  **Defining the Type Certification Basis:** The Applicant (manufacturer/designer) proposes a certification basis using the applicable USAR paragraphs (Book 1) and their related AMCs (Book 2). This basis defines the specific requirements the UAV system must meet (STANAG page 9, TYPE CERTIFICATION (OR EQUIVALENT) PROCESS).
2.  **Demonstrating Compliance:** The Applicant must provide evidence (analysis, tests, simulations, service experience) to demonstrate that the UAV system complies with each requirement in its Type Certification Basis. This includes documentation such as a UAV System Flight Manual (USAR.1581) and Instructions for Continued Airworthiness (USAR.1529, Appendix G).
3.  **Certifying Authority Approval:** A National Certifying Authority reviews the proposed certification basis and the compliance evidence. Upon satisfaction, they issue a Military Type Certificate or an equivalent national document, stating the UAV System's compliance with STANAG 4671.
4.  **Special Conditions:** For UAV systems with "novel or unusual design features" or "unconventional intended use," the Certifying Authority may prescribe "Special Conditions" (USAR.U2(c)) to ensure adequate safety standards are met. This is particularly relevant for cutting-edge autonomous functionalities.

The overall goal is to ensure that the UAV system's design adequately "reduce the risk to people including UAV crew, maintainers and third parties to a level acceptable" (USAR.1309(a)). The severity of potential failure conditions is classified (e.g., Catastrophic, Hazardous, Major, Minor, No Safety Effect) and correlated with probability levels (e.g., Extremely Improbable, Remote, Probable, Frequent) (AMC.1309(b)(3) & (4)).

#### 6.3. Compliance Considerations for Key System Aspects

For an autonomous flight controller based on Cube Orange+, several STANAG 4671 subparts are highly relevant:

##### 6.3.1. General Airworthiness (Subpart A)
*   **USAR.1 Applicability:** Ensures the UAV falls within the scope (fixed-wing, 150-20,000 kg MTOW) or is otherwise accepted by the Certifying Authority.
*   **USAR.U2 Assumptions:** Outlines fundamental assumptions, including the design for command and control for all flight phases, and the use of Special Conditions for novel features or unconventional uses. This applies directly to the Cube Orange+'s advanced capabilities.
*   **USAR.U17 Design Usage Spectrum:** The Applicant must present the design usage spectrum, which ties the certification to specific missions and operational environments. This impacts all system design choices.

##### 6.3.2. UAV Flight Performance (Subpart B)
*   **USAR.21 Proof of Compliance:** Requires demonstration of compliance through tests or calculations for all weight and C.G. combinations. This is critical for auto take-off and landing.
*   **USAR.45 General (Performance):** Defines the atmospheric and engine performance conditions under which performance data must be determined for take-off, climb, and landing.
*   **USAR.53 Take-off Performance:** Specifies how take-off distances and climb to 15m (50ft) must be determined, directly relevant to the "Auto Take-Off" mode.
*   **USAR.75 Landing Distance:** Details requirements for determining horizontal landing distance, crucial for the "Return-to-Land" mode.
*   **USAR.141 Flight Characteristics: General:** Mandates acceptable controllability, maneuverability, and stability in automatic control modes, emphasizing the importance of the stabilization algorithms.
*   **USAR.171 Stability: General:** Requires the UAV (augmented by the FCS) to be stable longitudinally, directionally, and laterally, with smooth and convergent transient responses.

##### 6.3.3. UAV Structure (Subpart C)
*   **USAR.301 Loads:** Requires the structure to withstand limit and ultimate loads under various flight and ground conditions.
*   **USAR.333 Flight Envelope:** Defines the V-n diagram and gust envelope, against which the airframe and FCS protection features must be designed.
*   **USAR.U334 Flight envelope protection:** Mandates that the flight control system implements smooth and appropriate envelope protection compatible with structural limits and safe maneuvering. This directly applies to the Cube Orange+'s role in maintaining flight limits.

##### 6.3.4. Equipment and System Safety (Subpart F)
*   **USAR.1309 Equipment, systems and installations:** This is a cornerstone, requiring that all UAV systems and equipment reduce risk to an acceptable level. It mandates analysis of failure conditions (Catastrophic, Hazardous, Major, Minor) and their probabilities. This is crucial for the reliability of the Cube Orange+ and its integrated sensors.
*   **USAR.U1307 Environmental Control System (ECS):** Ensures critical avionics (like the Cube Orange+) maintain operational performance within their environmental limits, considering cooling needs.
*   **USAR.1323 Airspeed measuring device & USAR.1325 Static pressure measuring device:** These are critical inputs for the flight controller, and their accuracy and reliability are addressed.
*   **USAR.1329 Flight control system:** Detailed requirements for the FCS, including its modes (automatic, semi-automatic), crew intervention capabilities, and robustness against malfunctions. This is directly applicable to the Cube Orange+ and ArduPilot.
*   **USAR.1459 UAV onboard flight recorders:** Specifies requirements for flight data recording, including airspeed, altitude, and heading data sources, which are outputs of the positioning sensors.
*   **USAR.U1412 Emergency recovery capability:** Requires the system to integrate an emergency recovery capability, such as a flight termination system or pre-programmed autonomous recovery procedures, to mitigate critical failures. This underpins the importance of RTL and other failsafe mechanisms.

##### 6.3.5. Command and Control Data Link (Subpart H)
*   **USAR.U1601 General:** Defines the command and control data link's functions (uplink for commands, downlink for status data) and its necessity for UAV control. This directly applies to the long-range RC link.
*   **USAR.U1603 Command and control data link architecture:** Requires an architecture that prevents single failures from leading to hazardous events.
*   **USAR.U1605 Electromagnetic interference and compatibility (EMI/EMC):** Mandates protection against EMI/EMC to ensure reliable data link operation.
*   **USAR.U1607 Command and control data link performance and monitoring:** Requires specifying and monitoring the effective maximum range of the data link and providing warning cues for potential breakdowns.
*   **USAR.U1611 Command and control data link latency:** Specifies that time delays in the data link must not lead to unsafe conditions.
*   **USAR.U1613 Command and control data link loss strategy:** Critical for failsafe implementation, requiring an approved strategy in the UAV System Flight Manual, including autonomous reacquisition attempts and alerts for the UAV crew.

##### 6.3.6. UAV Control Station (Subpart I)
*   **USAR.U1701 General:** Requires the UCS to facilitate safe command and control of the UAV.
*   **USAR.U1723 Flight and navigation data:** Specifies minimum flight and navigational data that must be continuously displayed in the UCS (e.g., indicated airspeed, pressure altitude, heading/track, UAV position).
*   **USAR.U1731 General (Controls):** Requires controls to be logically located and identified, preventing confusion and inadvertent operation.
*   **USAR.U1732 Safety critical controls:** Emphasizes accessibility and design of controls requiring prompt UAV crew reaction during emergencies.
*   **USAR.U1849 Operating limitations indications:** Requires clear indications in the UCS about operational limitations.

#### 6.4. Documentation and Traceability

A key aspect of STANAG 4671 certification is comprehensive documentation. The Applicant must develop:
*   **UAV System Flight Manual (USAR.1581):** Contains all operating limitations, procedures (normal, abnormal, emergency, including details for auto take-off, RTL, and data link loss strategies), performance information, and payload loading instructions.
*   **Instructions for Continued Airworthiness (USAR.1529, Appendix G):** Specifies maintenance, inspection, and repair procedures to ensure the UAV remains airworthy throughout its lifespan.
*   **Design and Analysis Reports:** Detailed documentation of structural integrity, system safety assessments (FHA, PSSA, SSA - AMC.1309(b)), and compliance demonstrations for all hardware and software components.

Compliance with STANAG 4671 requires a systematic approach to design, analysis, and testing, with meticulous record-keeping and clear traceability between design features, operational procedures, and the specific airworthiness requirements.

### 7. Conclusion

The Cube Orange+ flight controller, combined with a robust software platform like ArduPilot and a comprehensive suite of sensors, provides a highly capable and reliable foundation for an autonomous fixed-wing UAV. By meticulously adhering to the stringent airworthiness requirements outlined in STANAG 4671, particularly in areas of flight performance, system safety, sensor integration, and data link management, the UAV system can achieve the necessary certification for safe and interoperable military operations in complex airspaces. The detailed documentation and continuous adherence to the standard's principles throughout the system's lifecycle are paramount to maintaining airworthiness.

### 8. References

*   **STANAG 4671 (Edition 1):** Unmanned Aerial Vehicle Systems Airworthiness Requirements (USAR), North Atlantic Treaty Organization (NATO) Standardization Agency, 3 September 2009.

### 9. Glossary

*   **AMC:** Acceptable Means of Compliance (STANAG 4671, Book 2)
*   **ArduPilot:** Open-source autopilot software used on various UAV platforms.
*   **Certifying Authority:** National body responsible for approving airworthiness certification.
*   **Cube Orange+:** High-performance flight controller from CubePilot.
*   **EKF:** Extended Kalman Filter, a sensor fusion algorithm.
*   **EMC:** Electromagnetic Compatibility (STANAG 4671, Annex A).
*   **EMI:** Electromagnetic Interference (STANAG 4671, Annex A).
*   **FC/FCS:** Flight Controller / Flight Control System.
*   **FHA:** Functional Hazard Assessment (STANAG 4671, Annex A).
*   **GPS:** Global Positioning System.
*   **IMU:** Inertial Measurement Unit (accelerometers, gyroscopes, magnetometers).
*   **Lidar:** Light Detection and Ranging, a ranging sensor.
*   **MTOW:** Maximum Take-Off Weight.
*   **Optical Flow:** Sensor measuring relative ground motion.
*   **RC:** Radio Control.
*   **RTL:** Return-to-Land.
*   **RTK:** Real-Time Kinematic (GPS).
*   **SSA:** System Safety Assessment.
*   **STANAG:** Standardization Agreement (NATO).
*   **UAV:** Unmanned Aerial Vehicle (STANAG 4671, Annex A).
*   **UCS:** UAV Control Station (STANAG 4671, Annex A).
*   **USAR:** Unmanned Aerial Vehicle Systems Airworthiness Requirements (STANAG 4671, Book 1).
*   **VR:** Rotation Speed (STANAG 4671, Annex A).
*   **VS0/VS1:** Stalling Speeds (STANAG 4671, Annex A).
*   **VMC:** Minimum Control Speed (STANAG 4671, Annex A).
*   **Vmin DEMO:** Minimum Demonstration Speed (STANAG 4671, Annex A).
*   **VNE:** Never Exceed Speed (STANAG 4671, Annex A).
*   **VNO:** Maximum Structural Cruising Speed (STANAG 4671, Annex A).