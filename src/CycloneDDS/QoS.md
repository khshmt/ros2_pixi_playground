# 🛸 CycloneDDS Quality of Service (QoS) Guide

In native CycloneDDS development, QoS isn't just a setting—it’s a **contract** between the DataWriter (Publisher) and the DataReader (Subscriber). If their contracts do not match, they will fail to discover each other.

---

## 1. Core QoS Policies

### A. Reliability: "Guarantee of Delivery"
Controls how the system behaves when packets are lost on the network.
* **`DDS_RELIABILITY_BEST_EFFORT`**: Samples are sent once. If the network drops them, they are gone.
    * *Best for:* High-frequency data (Camera frames, LiDAR, IMU).
* **`DDS_RELIABILITY_RELIABLE`**: The Writer expects an Acknowledgement (ACK). If it doesn't get one, it re-transmits.
    * *Best for:* Commands, State changes, Emergency Stops.

### B. Durability: "Data Longevity"
Controls if data "lives on" for subscribers that join the network late.
* **`DDS_DURABILITY_VOLATILE`**: No history is kept. If you weren't connected when the message was sent, you missed it.
* **`DDS_DURABILITY_TRANSIENT_LOCAL`**: The Writer keeps a copy of the last $N$ samples. When a new Reader starts, the Writer automatically sends them the "latest state."
* **`DDS_DURABILITY_TRANSIENT`**: The data is stored in a separate dedicated service (often called a "Durability Service" or "Cloud") rather than within the Publisher's memory.
* **`DDS_DURABILITY_PERSISTENT`**: Data is written to non-volatile storage (Disk/SSD).

### C. History: "The Local Buffer"
Manages the internal cache of the Reader or Writer.
* **`DDS_HISTORY_KEEP_LAST (N)`**: Only stores the $N$ most recent samples. Old ones are overwritten.
    * *Best for:* Real-time systems where "old data is bad data."
* **`DDS_HISTORY_KEEP_ALL`**: Keeps every sample until the system runs out of memory.

---

## 2. The Compatibility Rule (Requested vs. Offered)

For a connection to be established, the **Publisher (Offered)** must provide a service level that is **equal to or stronger than** what the **Subscriber (Requested)** asks for.

| Policy | Publisher (Offered) | Subscriber (Requested) | Connection? |
| :--- | :--- | :--- | :--- |
| **Reliability** | `RELIABLE` | `BEST_EFFORT` | ✅ **Yes** |
| **Reliability** | `BEST_EFFORT` | `RELIABLE` | ❌ **No** |
| **Durability** | `TRANSIENT_LOCAL` | `VOLATILE` | ✅ **Yes** |
| **Durability** | `VOLATILE` | `TRANSIENT_LOCAL` | ❌ **No** |

---

## 3. Optimized Combinations for Robotics

### 🚀 High-FPS Video / Heavy Sensors
**Goal:** Minimum latency, zero queue buildup, maximum throughput.

| Policy | Value | Reasoning |
| :--- | :--- | :--- |
| **Reliability** | `BEST_EFFORT` | Prevents "Head-of-Line" blocking if a frame is lost. |
| **History** | `KEEP_LAST` | We only care about the current state of the world. |
| **Durability** | `VOLATILE` | Don't waste bandwidth sending old frames to new nodes. |

---

## 4. Native CycloneDDS C Snippet (High-FPS Profile)

```c
// Create a QoS object
dds_qos_t *qos = dds_create_qos();

// Set for High Performance Camera Feed
dds_qset_reliability(qos, DDS_RELIABILITY_BEST_EFFORT, 0);
dds_qset_durability(qos, DDS_DURABILITY_VOLATILE);
dds_qset_history(qos, DDS_HISTORY_KEEP_LAST, 1);

// Apply to your Writer or Reader
dds_entity_t writer = dds_create_writer(participant, topic, qos, NULL);

// Clean up the helper object
dds_delete_qos(qos);
```

---

## More Info [here](https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html#qos-compatibilities:~:text=from%20the%20community.-,QoS%20compatibilities,-%EF%83%81)