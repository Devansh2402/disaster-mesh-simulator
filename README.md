# DisasterMeshHub 📡

## 📌 Overview
DisasterMeshHub is an interactive, browser-based simulation platform designed to model how communication networks can survive in crisis zones when the traditional internet grid completely collapses. Instead of simulating static, pre-connected graphs, this engine models dynamic, asynchronous peer-to-peer (P2P) routing protocol interactions among mobile nodes in real-time.

---

## 🎯 Problem Statement
During major environmental crises, centralized communication nodes (like cellular towers and electrical substations) are often physically compromised or heavily congested. This leaves individuals isolated even if emergency services are nearby. This simulator explores how direct device-to-device protocols (like BLE and Wi-Fi Direct) can spontaneously form ad-hoc, infrastructure-free routing networks to carry lifesaving data through a crowd.

---

## 🚀 Features
*   **Asset Deployment Matrix:** Dynamically spawn and manually position heterogeneous network assets (Civilian Smartphones, Mobile Vehicle Hubs, and Long-Range LoRa Base Stations).
*   **Spontaneous Peer Discovery:** Nodes continuously map neighboring nodes and establish localized digital communication pipelines over the canvas without a network registry.
*   **Interactive Signal Blockers:** Draw custom environmental walls and rubble perimeters directly onto the canvas grid to simulate urban signal destruction.
*   **Store-and-Forward Queuing:** View active packet payloads cached inside a node’s isolated local memory map when it is cut off from alternative bridges.
*   **High-Contrast Command HUD:** Real-time visualization panels showing unique delivered emergency alerts, suppressed duplicate routing loops, active battery drain logs, and a live incident triage ledger.

---

## 🛠️ Tech Stack
*   **Frontend Pipeline:** Vanilla HTML5 Canvas API (optimized for 60 FPS hardware-accelerated pixel rendering).
*   **Styling & UI:** Modern CSS3 featuring flexible grid layouts, responsive media frameworks, and clean glassmorphism (`backdrop-filter`) visual design.
*   **Core Engine Language:** Pure JavaScript / ES6+ (Native state machines using structural Object-Oriented paradigms).
*   **Storage Simulation:** Native JS `Map` data architecture mimicking localized, embedded database queues (like SQLite).

---

## 🧠 Key Concepts Demonstrated
*   **Euclidean Spatial Proximity Math:** Replacing standard network polling structures with runtime coordinate distance sweeps ($d = \sqrt{\Delta x^2 + \Delta y^2}$) across moving bodies.
*   **Vector Line-of-Sight (LoS) Intersection Logic:** Utilizing ray-to-bounding-box segment math to mathematically verify if radio waves are blocked by environmental debris.
*   **Hash-Based Data Deduplication:** Halting network-wide **Broadcast Storms** by checking incoming message UUID hashes against local look-up registries to silently discard redundant packets.
*   **Path-Vector Loop Suppression:** Forbidding packets from traveling backward by validating target node signatures against the message’s historic `pathTrace` array registry.
*   **Deterministic Time-to-Live (TTL):** Implementing a strict hop-decay mechanism to safely purge stale messages from consumer hardware memory maps over time.
*   **Linear Power Degradation Modeling:** Simulating the realistic power penalty of routing computations by applying a battery capacity tax onto active smartphone assets.

---

## 🔮 Future Scope — Phase 2: Mobile Mesh App
The mathematical modeling and synchronization rules verified in this sandbox environment serve as the blueprint to transition this framework into a production-grade mobile mesh application:
*   **Native Multi-Peer Integration:** Implementing the discovery engine via native Android Wi-Fi Aware/Direct APIs and Apple Multipeer Connectivity frameworks.
*   **Offline Storage Engines:** Migrating the JavaScript memory maps directly to lightweight embedded SQLite databases for strict offline data logging.
*   **Asymmetric Cryptography:** Adding cryptographic signature verification to payloads to prevent malicious actors from spoofing emergency alerts or poisoning routing maps in the field.
