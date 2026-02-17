# For a distributed Publisher-Subscriber system
---
### Network topology
Machine A (Camera)  ─────►  Machine B (Viewer)
   Publisher                  Subscriber

Both machines must be reachable through the same LAN or via direct IP routing.

---

### Configuration

DDS uses an XML configuration file to control networking behavior such as interface selection and peer discovery.

Each machine must have its own configuration file containing its local IP and the peer IP.

Example:
```
Machine A IP: 192.168.1.10

Machine B IP: 192.168.1.20
```
---

### Running the system

You must export the configuration path before starting any executable.

#### Linux
```bash
export CYCLONEDDS_URI=file://$PWD/CycloneDDS.xml
```
#### windows
```bash
$env:CYCLONEDDS_URI="file://CycloneDDS.xml"
```

---

### Start order

On Machine B (subscriber):
```bash
pixi run DDSFrameSubscriber
```
On Machine A (publisher):

```bash
pixi run DDSFramePublisher
```

After discovery completes, the video stream should appear automatically.